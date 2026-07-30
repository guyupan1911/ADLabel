#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mapping/camera/camera.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/grid_frame.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/mapping_utils/timestamp_aligner.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for sensor data");
DEFINE_string(lidar_metadata, "", "path to LiDAR frame metadata");
DEFINE_string(camera_metadata, "", "path to Camera frame metadata");
DEFINE_string(output_dir, "", "directory to save colored map outputs");
DEFINE_int64(max_time_diff_ms, 10,
             "maximum LiDAR/Camera timestamp difference in milliseconds");
DEFINE_uint64(max_frames, 0, "maximum number of frames, 0 means all");
DEFINE_double(bev_resolution, 0.05, "BEV resolution in meters per pixel");

namespace adlabel {
namespace mapping {
namespace {

bool IsProcessableLidarFrame(const Frame& frame) {
  return frame.has_timestamp_ns() && frame.has_refined_pose_3d() &&
         frame.has_sensor_to_imu_extrinsic() && frame.has_cloud_uri() &&
         !frame.cloud_uri().empty();
}

bool IsProcessableCameraFrame(const Frame& frame) {
  return frame.has_timestamp_ns() && frame.has_sensor_to_imu_extrinsic() &&
         frame.has_camera_calibration() && frame.has_camera_image_uri() &&
         !frame.camera_image_uri().empty();
}

bool IsFinitePoint(const PointXYZIRT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

void ConvertToBgr(cv::Mat* image) {
  CHECK(image != nullptr);
  if (image->channels() == 1) {
    cv::cvtColor(*image, *image, cv::COLOR_GRAY2BGR);
  } else if (image->channels() == 4) {
    cv::cvtColor(*image, *image, cv::COLOR_BGRA2BGR);
  }
  CHECK_EQ(image->channels(), 3);

  if (image->depth() != CV_8U) {
    cv::Mat normalized;
    cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    *image = normalized;
  }
}

cv::Vec3b BilinearSample(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  const double x = pixel.x();
  const double y = pixel.y();
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, image.cols - 1);
  const int y1 = std::min(y0 + 1, image.rows - 1);
  const double dx = x - x0;
  const double dy = y - y0;

  const cv::Vec3d c00 = image.at<cv::Vec3b>(y0, x0);
  const cv::Vec3d c10 = image.at<cv::Vec3b>(y0, x1);
  const cv::Vec3d c01 = image.at<cv::Vec3b>(y1, x0);
  const cv::Vec3d c11 = image.at<cv::Vec3b>(y1, x1);
  const cv::Vec3d color = (1.0 - dx) * (1.0 - dy) * c00 +
                          dx * (1.0 - dy) * c10 + (1.0 - dx) * dy * c01 +
                          dx * dy * c11;
  return {cv::saturate_cast<uint8_t>(color[0]),
          cv::saturate_cast<uint8_t>(color[1]),
          cv::saturate_cast<uint8_t>(color[2])};
}

bool InitializeTrajectory(const std::vector<Frame>& lidar_frames,
                          SimplePose3DInterpolator* pose_interpolator,
                          Eigen::Affine3d* T_map_ecef) {
  CHECK(pose_interpolator != nullptr);
  CHECK(T_map_ecef != nullptr);

  size_t pose_count = 0;
  bool has_map_origin = false;
  for (const Frame& frame : lidar_frames) {
    if (!frame.has_timestamp_ns() || !frame.has_refined_pose_3d()) {
      continue;
    }

    FrameData frame_data;
    frame_data.pose_ecef = Pose3D(frame.refined_pose_3d()).GetAffine3D();
    pose_interpolator->InsertTimestampedPose(frame.timestamp_ns(),
                                             Pose3D(frame_data.pose_ecef));
    if (!has_map_origin) {
      *T_map_ecef = frame_data.pose_ecef.inverse();
      has_map_origin = true;
    }
    ++pose_count;
  }

  LOG(INFO) << "initialized ECEF trajectory with " << pose_count << " poses";
  return pose_count >= 2 && has_map_origin;
}

bool InterpolateCameraPose(const SimplePose3DInterpolator& pose_interpolator,
                           int64_t camera_timestamp_ns,
                           Eigen::Affine3d* camera_pose_ecef) {
  CHECK(camera_pose_ecef != nullptr);
  Pose3D pose;
  if (!pose_interpolator.GetTimestampedPose(camera_timestamp_ns, &pose,
                                            false)) {
    LOG(WARNING) << "failed to interpolate Camera ECEF pose at "
                 << camera_timestamp_ns;
    return false;
  }
  *camera_pose_ecef = pose.GetAffine3D();
  return true;
}

size_t AppendColoredPoints(
    const FrameData::Cloud& lidar_cloud, const Eigen::Affine3d& T_camera_lidar,
    const Eigen::Affine3d& T_map_lidar, const BaseCamera& camera,
    const cv::Mat& camera_image,
    pcl::PointCloud<pcl::PointXYZRGB>* colored_map_cloud) {
  CHECK(colored_map_cloud != nullptr);
  size_t colored_point_count = 0;

  for (const PointXYZIRT& point : lidar_cloud) {
    if (!IsFinitePoint(point)) {
      continue;
    }

    const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
    const Eigen::Vector3d point_camera = T_camera_lidar * point_lidar;
    Eigen::Vector2d pixel;
    if (!camera.Project(point_camera, &pixel) ||
        !pixel.array().isFinite().all() || pixel.x() < 0.0 ||
        pixel.x() >= camera_image.cols || pixel.y() < 0.0 ||
        pixel.y() >= camera_image.rows) {
      continue;
    }

    const Eigen::Vector3d point_map = T_map_lidar * point_lidar;
    const cv::Vec3b bgr = BilinearSample(camera_image, pixel);
    pcl::PointXYZRGB colored_point;
    colored_point.x = static_cast<float>(point_map.x());
    colored_point.y = static_cast<float>(point_map.y());
    colored_point.z = static_cast<float>(point_map.z());
    colored_point.r = bgr[2];
    colored_point.g = bgr[1];
    colored_point.b = bgr[0];
    colored_map_cloud->push_back(colored_point);
    ++colored_point_count;
  }
  return colored_point_count;
}

bool RenderBev(const pcl::PointCloud<pcl::PointXYZRGB>& colored_map_cloud,
               double resolution, cv::Mat* bev_rgb, cv::Mat* bev_coverage) {
  CHECK(bev_rgb != nullptr);
  CHECK(bev_coverage != nullptr);
  if (colored_map_cloud.empty()) {
    return false;
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const pcl::PointXYZRGB& point : colored_map_cloud) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }

  GridFrame grid_frame;
  grid_frame.resolution = resolution;
  grid_frame.cols = std::max(
      1u,
      static_cast<unsigned int>(std::ceil((max_x - min_x) / resolution) + 1.0));
  grid_frame.rows = std::max(
      1u,
      static_cast<unsigned int>(std::ceil((max_y - min_y) / resolution) + 1.0));
  grid_frame.top_left_corner = {min_x, max_y};

  *bev_rgb = cv::Mat::zeros(static_cast<int>(grid_frame.rows),
                            static_cast<int>(grid_frame.cols), CV_8UC3);
  *bev_coverage = cv::Mat::zeros(static_cast<int>(grid_frame.rows),
                                 static_cast<int>(grid_frame.cols), CV_8UC1);
  cv::Mat sample_counts =
      cv::Mat::zeros(static_cast<int>(grid_frame.rows),
                     static_cast<int>(grid_frame.cols), CV_32SC1);

  for (const pcl::PointXYZRGB& point : colored_map_cloud) {
    unsigned int row = 0;
    unsigned int col = 0;
    if (!grid_frame.WorldToPixel({point.x, point.y}, &row, &col)) {
      continue;
    }

    int& sample_count =
        sample_counts.at<int>(static_cast<int>(row), static_cast<int>(col));
    cv::Vec3b& output =
        bev_rgb->at<cv::Vec3b>(static_cast<int>(row), static_cast<int>(col));
    const cv::Vec3b input(point.b, point.g, point.r);
    for (int channel = 0; channel < 3; ++channel) {
      const uint64_t sum = static_cast<uint64_t>(output[channel]) *
                               static_cast<uint64_t>(sample_count) +
                           input[channel];
      output[channel] = static_cast<uint8_t>(sum / (sample_count + 1));
    }
    ++sample_count;
    bev_coverage->at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) =
        255;
  }
  return true;
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_camera_metadata.empty()) << "--camera_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK_GE(FLAGS_max_time_diff_ms, 0);
  CHECK_GT(FLAGS_bev_resolution, 0.0);

  auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
  std::vector<Frame> lidar_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
  std::vector<Frame> camera_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_camera_metadata);
  CHECK(!lidar_frames.empty()) << "no LiDAR frames loaded";
  CHECK(!camera_frames.empty()) << "no Camera frames loaded";

  const auto by_timestamp = [](const Frame& left, const Frame& right) {
    return left.timestamp_ns() < right.timestamp_ns();
  };
  std::sort(lidar_frames.begin(), lidar_frames.end(), by_timestamp);
  std::sort(camera_frames.begin(), camera_frames.end(), by_timestamp);

  SimplePose3DInterpolator pose_interpolator;
  Eigen::Affine3d T_map_ecef = Eigen::Affine3d::Identity();
  if (!InitializeTrajectory(lidar_frames, &pose_interpolator, &T_map_ecef)) {
    LOG(ERROR) << "at least two refined LiDAR poses are required";
    return 1;
  }

  TimestampAligner aligner(FLAGS_max_time_diff_ms * 1000LL * 1000LL);
  aligner.AddSensorFrameRefs("lidar", lidar_frames);
  aligner.AddSensorFrameRefs("camera", camera_frames);
  const std::vector<AlignedFrames> aligned_frames =
      aligner.Align("lidar", true);
  CHECK(!aligned_frames.empty()) << "no aligned LiDAR/Camera frames";

  auto colored_map_cloud =
      pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  size_t processed_pairs = 0;
  size_t ground_point_count = 0;
  for (const AlignedFrames& aligned : aligned_frames) {
    if (FLAGS_max_frames > 0 && processed_pairs >= FLAGS_max_frames) {
      break;
    }

    const auto camera_ref = aligned.aligned_frames.find("camera");
    if (camera_ref == aligned.aligned_frames.end()) {
      continue;
    }
    const Frame& lidar_frame = lidar_frames[aligned.reference_frame.index];
    const Frame& camera_frame = camera_frames[camera_ref->second.index];
    if (!IsProcessableLidarFrame(lidar_frame) ||
        !IsProcessableCameraFrame(camera_frame)) {
      continue;
    }

    FrameData lidar_data;
    FrameData camera_data;
    if (!GenerateLidarFrameData(lidar_frame, &lidar_data, data_reader) ||
        !GenerateCameraFrameData(camera_frame, &camera_data, data_reader) ||
        lidar_data.ground_cloud == nullptr ||
        lidar_data.ground_cloud->empty()) {
      continue;
    }
    if (!InterpolateCameraPose(pose_interpolator, camera_frame.timestamp_ns(),
                               &camera_data.pose_ecef)) {
      continue;
    }

    ConvertToBgr(&camera_data.camera_image);
    const std::unique_ptr<BaseCamera> camera =
        CreateCamera(camera_data.camera_calibration);
    const Eigen::Affine3d T_camera_lidar =
        camera_data.transform_from_sensor_to_imu.inverse() *
        camera_data.pose_ecef.inverse() * lidar_data.pose_ecef *
        lidar_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d T_map_lidar = T_map_ecef * lidar_data.pose_ecef *
                                        lidar_data.transform_from_sensor_to_imu;

    ground_point_count += lidar_data.ground_cloud->size();
    const size_t colored_count = AppendColoredPoints(
        *lidar_data.ground_cloud, T_camera_lidar, T_map_lidar, *camera,
        camera_data.camera_image, colored_map_cloud.get());
    ++processed_pairs;
    LOG(INFO) << "colored frame " << lidar_frame.fid() << ": " << colored_count
              << "/" << lidar_data.ground_cloud->size()
              << " ground points, time_diff_ms="
              << static_cast<double>(camera_ref->second.time_diff_ns) / 1e6;
  }

  if (colored_map_cloud->empty()) {
    LOG(ERROR) << "no LiDAR points projected into aligned Camera images";
    return 1;
  }
  colored_map_cloud->width = static_cast<uint32_t>(colored_map_cloud->size());
  colored_map_cloud->height = 1;
  colored_map_cloud->is_dense = true;

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << error.message();

  const std::filesystem::path cloud_path = output_dir / "colored_map_cloud.pcd";
  if (pcl::io::savePCDFileBinary(cloud_path.string(), *colored_map_cloud) < 0) {
    LOG(ERROR) << "failed to save " << cloud_path.string();
    return 1;
  }

  cv::Mat bev_rgb;
  cv::Mat bev_coverage;
  if (!RenderBev(*colored_map_cloud, FLAGS_bev_resolution, &bev_rgb,
                 &bev_coverage)) {
    LOG(ERROR) << "failed to render colored map BEV";
    return 1;
  }

  const std::filesystem::path bev_rgb_path = output_dir / "bev_rgb.png";
  const std::filesystem::path bev_coverage_path =
      output_dir / "bev_coverage.png";
  if (!cv::imwrite(bev_rgb_path.string(), bev_rgb) ||
      !cv::imwrite(bev_coverage_path.string(), bev_coverage)) {
    LOG(ERROR) << "failed to save BEV images";
    return 1;
  }

  LOG(INFO) << "saved colored map: pairs=" << processed_pairs
            << ", colored_points=" << colored_map_cloud->size() << "/"
            << ground_point_count << ", cloud=" << cloud_path.string()
            << ", bev=" << bev_rgb_path.string();
  return 0;
}

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return adlabel::mapping::Run();
}
