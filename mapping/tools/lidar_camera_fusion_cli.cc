#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "mapping/camera/camera.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/mapping_utils/timestamp_aligner.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata, "",
              "lidar metadata path, relative to data_root or absolute");
DEFINE_string(camera_metadata, "",
              "camera metadata path, relative to data_root or absolute");
DEFINE_string(data_root, "", "root directory for sensor data");
DEFINE_string(output_dir, "", "directory to save projected images");
DEFINE_int64(max_time_diff_ms, 10,
             "maximum lidar-camera timestamp difference in milliseconds");
DEFINE_double(max_render_depth_m, 80.0,
              "maximum depth used by color rendering");
DEFINE_int32(point_radius, 1, "projected point radius in pixels");
DEFINE_uint64(max_frames, 0,
              "maximum matched frame pairs to process, 0 means all");
DEFINE_string(
    depth_metric, "euclidean",
    "depth metric for color mapping: 'z_axis' (camera z coordinate) or "
    "'euclidean' (straight-line distance to camera)");
DEFINE_bool(enable_motion_compensation, true,
            "enable motion compensation using LIO pose interpolation for "
            "timestamp differences");

namespace adlabel {
namespace mapping {
namespace {

bool IsProcessableLidarFrame(const Frame& frame) {
  if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
    LOG(WARNING) << "skip lidar frame without cloud_uri: " << frame.fid();
    return false;
  }
  if (!frame.has_sensor_to_imu_extrinsic()) {
    LOG(WARNING) << "skip lidar frame without sensor_to_imu_extrinsic: "
                 << frame.fid();
    return false;
  }
  return true;
}

bool IsProcessableCameraFrame(const Frame& frame) {
  if (!frame.has_camera_image_uri() || frame.camera_image_uri().empty()) {
    LOG(WARNING) << "skip camera frame without camera_image_uri: "
                 << frame.fid();
    return false;
  }
  if (!frame.has_sensor_to_imu_extrinsic()) {
    LOG(WARNING) << "skip camera frame without sensor_to_imu_extrinsic: "
                 << frame.fid();
    return false;
  }
  if (!frame.has_camera_calibration()) {
    LOG(WARNING) << "skip camera frame without camera_calibration: "
                 << frame.fid();
    return false;
  }
  return true;
}

bool IsFinitePoint(const PointXYZIRT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

cv::Scalar DepthColor(double depth_m) {
  const double t = Clamp(depth_m / FLAGS_max_render_depth_m, 0.0, 1.0);
  // Near points are red, far points are blue.
  const double hue = 240.0 * t;
  const double c = 1.0;
  const double x = c * (1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0));

  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  if (hue < 60.0) {
    r = c;
    g = x;
  } else if (hue < 120.0) {
    r = x;
    g = c;
  } else if (hue < 180.0) {
    g = c;
    b = x;
  } else {
    g = x;
    b = c;
  }
  return cv::Scalar(b * 255.0, g * 255.0, r * 255.0);
}

void ConvertToBgrIfNeeded(cv::Mat* image) {
  CHECK(image != nullptr);
  if (image->empty()) {
    return;
  }

  if (image->channels() == 1) {
    if (image->depth() != CV_8U) {
      cv::Mat normalized;
      cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(normalized, *image, cv::COLOR_GRAY2BGR);
    } else {
      cv::cvtColor(*image, *image, cv::COLOR_GRAY2BGR);
    }
    return;
  }
  if (image->channels() == 4) {
    cv::cvtColor(*image, *image, cv::COLOR_BGRA2BGR);
  }

  CHECK_EQ(image->channels(), 3) << "unsupported image channel count";
  if (image->depth() != CV_8U) {
    cv::Mat normalized;
    cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    *image = normalized;
  }
}

std::string MakeOutputFileName(const Frame& lidar_frame,
                               const Frame& camera_frame, size_t pair_index) {
  std::ostringstream stream;
  stream << "fusion_" << pair_index << "_lidar_" << lidar_frame.timestamp_ns()
         << "_camera_" << camera_frame.timestamp_ns() << ".png";
  return stream.str();
}

SimplePose3DInterpolator BuildLioPoseInterpolator(
    const std::vector<Frame>& lidar_frames) {
  SimplePose3DInterpolator interpolator;
  size_t valid_pose_count = 0;

  for (const auto& frame : lidar_frames) {
    if (!frame.has_lio_pose_3d()) {
      continue;
    }
    const Pose3D lio_pose(frame.lio_pose_3d());
    interpolator.InsertTimestampedPose(frame.timestamp_ns(), lio_pose);
    ++valid_pose_count;
  }

  LOG(INFO) << "built LIO pose interpolator with " << valid_pose_count
            << " poses from " << lidar_frames.size() << " lidar frames";
  return interpolator;
}

bool ComputeMotionCompensation(
    const SimplePose3DInterpolator& lio_pose_interpolator,
    int64_t lidar_timestamp_ns, int64_t camera_timestamp_ns,
    Eigen::Affine3d* T_compensation) {
  CHECK(T_compensation != nullptr);

  if (lidar_timestamp_ns == camera_timestamp_ns) {
    *T_compensation = Eigen::Affine3d::Identity();
    return true;
  }

  Pose3D lio_pose_at_lidar;
  Pose3D lio_pose_at_camera;

  if (!lio_pose_interpolator.GetTimestampedPose(lidar_timestamp_ns,
                                                &lio_pose_at_lidar, false)) {
    LOG(WARNING) << "failed to interpolate LIO pose at lidar timestamp "
                 << lidar_timestamp_ns;
    return false;
  }

  if (!lio_pose_interpolator.GetTimestampedPose(camera_timestamp_ns,
                                                &lio_pose_at_camera, false)) {
    LOG(WARNING) << "failed to interpolate LIO pose at camera timestamp "
                 << camera_timestamp_ns;
    return false;
  }

  *T_compensation = lio_pose_at_camera.GetAffine3D().inverse() *
                    lio_pose_at_lidar.GetAffine3D();

  return true;
}

size_t ProjectLidarToImage(const PointCloudXYZIRT& cloud,
                           const Eigen::Affine3d& T_lidar_to_camera,
                           const BaseCamera& camera, cv::Mat* image) {
  CHECK(image != nullptr);

  size_t projected_count = 0;
  for (const auto& point : cloud.points) {
    if (!IsFinitePoint(point)) {
      continue;
    }

    const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
    const Eigen::Vector3d point_camera = T_lidar_to_camera * point_lidar;

    Eigen::Vector2d pixel;
    if (!camera.ProjectWithoutRangeCheck(point_camera, &pixel)) {
      continue;
    }
    if (!std::isfinite(pixel.x()) || !std::isfinite(pixel.y())) {
      continue;
    }

    const int u = static_cast<int>(std::lround(pixel.x()));
    const int v = static_cast<int>(std::lround(pixel.y()));
    if (u < 0 || u >= image->cols || v < 0 || v >= image->rows) {
      continue;
    }

    // Compute depth based on the selected metric.
    const double depth = (FLAGS_depth_metric == "euclidean")
                             ? point_camera.norm()
                             : point_camera.z();

    cv::circle(*image, cv::Point(u, v), FLAGS_point_radius, DepthColor(depth),
               -1, cv::LINE_AA);
    ++projected_count;
  }
  return projected_count;
}

int Run() {
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_camera_metadata.empty()) << "--camera_metadata is required";
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK_GE(FLAGS_max_time_diff_ms, 0)
      << "--max_time_diff_ms must be non-negative";
  CHECK_GT(FLAGS_max_render_depth_m, 0.0)
      << "--max_render_depth_m must be positive";
  CHECK_GE(FLAGS_point_radius, 0) << "--point_radius must be non-negative";

  const int64_t max_time_diff_ns = FLAGS_max_time_diff_ms * 1000LL * 1000LL;

  auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
  std::vector<Frame> lidar_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
  std::vector<Frame> camera_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_camera_metadata);

  CHECK(!lidar_frames.empty())
      << "no lidar frames loaded from " << FLAGS_lidar_metadata;
  CHECK(!camera_frames.empty())
      << "no camera frames loaded from " << FLAGS_camera_metadata;

  LOG(INFO) << "loaded lidar_frames=" << lidar_frames.size()
            << ", camera_frames=" << camera_frames.size();

  SimplePose3DInterpolator lio_pose_interpolator;
  if (FLAGS_enable_motion_compensation) {
    lio_pose_interpolator = BuildLioPoseInterpolator(lidar_frames);
  }

  TimestampAligner timestamp_aligner(max_time_diff_ns);
  timestamp_aligner.AddSensorFrameRefs("lidar", lidar_frames);
  timestamp_aligner.AddSensorFrameRefs("camera", camera_frames);
  const std::vector<AlignedFrames> aligned_frames =
      timestamp_aligner.Align("lidar", true);
  CHECK(!aligned_frames.empty()) << "no lidar-camera frame pairs matched";

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error: " << error.message();

  size_t processed_pairs = 0;
  size_t saved_images = 0;
  for (size_t pair_index = 0; pair_index < aligned_frames.size();
       ++pair_index) {
    if (FLAGS_max_frames > 0 && processed_pairs >= FLAGS_max_frames) {
      break;
    }

    const AlignedFrames& aligned = aligned_frames[pair_index];
    const auto camera_ref_it = aligned.aligned_frames.find("camera");
    if (camera_ref_it == aligned.aligned_frames.end()) {
      continue;
    }

    const Frame& lidar_frame = lidar_frames[aligned.reference_frame.index];
    const Frame& camera_frame = camera_frames[camera_ref_it->second.index];

    if (!IsProcessableLidarFrame(lidar_frame) ||
        !IsProcessableCameraFrame(camera_frame)) {
      continue;
    }

    cv::Mat image;
    if (!data_reader->ReadImage(camera_frame.camera_image_uri(), &image)) {
      continue;
    }
    ConvertToBgrIfNeeded(&image);

    PointCloudXYZIRT::Ptr cloud(new PointCloudXYZIRT);
    if (!data_reader->ReadPointCloud(lidar_frame.cloud_uri(), cloud)) {
      continue;
    }

    std::unique_ptr<BaseCamera> camera =
        CreateCamera(camera_frame.camera_calibration());
    if (camera->Width() != static_cast<size_t>(image.cols) ||
        camera->Height() != static_cast<size_t>(image.rows)) {
      LOG(WARNING) << "camera calibration size " << camera->Width() << "x"
                   << camera->Height() << " differs from image size "
                   << image.cols << "x" << image.rows;
    }

    const Eigen::Affine3d T_lidar_to_imu =
        Pose3D(lidar_frame.sensor_to_imu_extrinsic()).GetAffine3D();
    const Eigen::Affine3d T_camera_to_imu =
        Pose3D(camera_frame.sensor_to_imu_extrinsic()).GetAffine3D();
    Eigen::Affine3d T_lidar_to_camera =
        T_camera_to_imu.inverse() * T_lidar_to_imu;

    Eigen::Affine3d T_motion_compensation = Eigen::Affine3d::Identity();
    if (FLAGS_enable_motion_compensation &&
        lidar_frame.timestamp_ns() != camera_frame.timestamp_ns()) {
      if (ComputeMotionCompensation(
              lio_pose_interpolator, lidar_frame.timestamp_ns(),
              camera_frame.timestamp_ns(), &T_motion_compensation)) {
        T_lidar_to_camera = T_lidar_to_camera * T_motion_compensation;

        const Eigen::Vector3d translation_diff =
            T_motion_compensation.translation();
        const Eigen::AngleAxisd rotation_diff(T_motion_compensation.rotation());
        LOG(INFO)
            << "motion compensation applied, time_diff_ms="
            << (camera_frame.timestamp_ns() - lidar_frame.timestamp_ns()) / 1e6
            << ", translation_norm_m=" << translation_diff.norm()
            << ", rotation_angle_deg=" << rotation_diff.angle() * 180.0 / M_PI;
      }
    }

    const size_t projected_count =
        ProjectLidarToImage(*cloud, T_lidar_to_camera, *camera, &image);

    const std::filesystem::path output_path =
        output_dir / MakeOutputFileName(lidar_frame, camera_frame, pair_index);
    if (!cv::imwrite(output_path.string(), image)) {
      LOG(ERROR) << "failed to save projection image: " << output_path.string();
      continue;
    }

    ++processed_pairs;
    ++saved_images;
    LOG(INFO) << "saved projection image: " << output_path.string()
              << ", projected_points=" << projected_count << "/"
              << cloud->points.size() << ", time_diff_ms="
              << static_cast<double>(camera_ref_it->second.time_diff_ns) / 1e6;
  }

  LOG(INFO) << "finished lidar-camera fusion visualization, processed_pairs="
            << processed_pairs << ", saved_images=" << saved_images;
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
