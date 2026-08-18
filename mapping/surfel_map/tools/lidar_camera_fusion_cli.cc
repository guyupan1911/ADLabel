#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
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
#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/frame_data.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/mapping_utils/timestamp_aligner.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "",
              "root directory for cloud_uri and camera_image_uri");
DEFINE_string(lidar_metadata, "", "absolute lidar metadata path");
DEFINE_string(camera_metadata, "", "absolute camera metadata path");
DEFINE_string(localization_gnss, "", "path to localization_gnss.txt");
DEFINE_string(output_dir, "", "directory to save visualization images");
DEFINE_double(camera_exposure_time_ms, 15.0,
              "camera exposure time; half is added to camera timestamp");

namespace adlabel {
namespace mapping {
namespace {

constexpr int64_t kMaxCameraLidarTimeDiffNs = 10LL * 1000LL * 1000LL;
constexpr double kMaxIntensity = 255.0;

struct GnssTrajectory {
  SimplePose3DInterpolator interpolator;
  int64_t min_timestamp_ns = std::numeric_limits<int64_t>::max();
  int64_t max_timestamp_ns = std::numeric_limits<int64_t>::min();

  bool GetPose(int64_t timestamp_ns, Eigen::Affine3d* world_T_imu) const {
    if (timestamp_ns < min_timestamp_ns || timestamp_ns > max_timestamp_ns) {
      return false;
    }
    Pose3D pose;
    if (!interpolator.GetTimestampedPose(timestamp_ns, &pose, false)) {
      return false;
    }
    *world_T_imu = pose.GetAffine3D();
    return true;
  }
};

GnssTrajectory LoadGnssTrajectory(const std::filesystem::path& path) {
  std::ifstream input(path);
  CHECK(input) << "failed to open " << path.string();

  std::string line;
  CHECK(std::getline(input, line)) << "empty GNSS trajectory";  // Header.

  GnssTrajectory trajectory;
  size_t pose_count = 0;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    double timestamp_msec = 0.0;
    std::string module_name;
    Eigen::Vector3d translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(row >> timestamp_msec >> module_name >> translation.x() >>
          translation.y() >> translation.z() >> qx >> qy >> qz >> qw)) {
      continue;
    }

    const int64_t timestamp_ns =
        static_cast<int64_t>(std::llround(timestamp_msec * 1e6));
    Eigen::Quaterniond rotation(qw, qx, qy, qz);
    rotation.normalize();
    trajectory.interpolator.InsertTimestampedPose(
        timestamp_ns, Pose3D(translation, rotation));
    trajectory.min_timestamp_ns =
        std::min(trajectory.min_timestamp_ns, timestamp_ns);
    trajectory.max_timestamp_ns =
        std::max(trajectory.max_timestamp_ns, timestamp_ns);
    ++pose_count;
  }

  CHECK_GE(pose_count, 2u) << "not enough GNSS poses";
  LOG(INFO) << "loaded " << pose_count << " GNSS poses";
  return trajectory;
}

bool LoadLidarCloud(const Frame& frame,
                    const std::shared_ptr<LocalDataReader>& data_reader,
                    PointCloudXYZIRT::Ptr cloud, Eigen::Affine3d* imu_T_lidar) {
  if (!frame.has_cloud_uri() || !frame.has_sensor_to_imu_extrinsic()) {
    return false;
  }
  *imu_T_lidar = Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();
  return data_reader->ReadPointCloud(frame.cloud_uri(), cloud);
}

bool DeskewCloud(const PointCloudXYZIRT& input,
                 const Eigen::Affine3d& imu_T_lidar,
                 const Eigen::Affine3d& world_T_imu_at_frame,
                 const GnssTrajectory& trajectory, PointCloudXYZIRT* output) {
  output->clear();
  output->reserve(input.size());

  const Eigen::Affine3d lidar_at_frame_T_world =
      (world_T_imu_at_frame * imu_T_lidar).inverse();
  for (const PointXYZIRT& point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.timestamp)) {
      continue;
    }

    const int64_t point_timestamp_ns =
        static_cast<int64_t>(std::llround(point.timestamp * 1e9));
    Eigen::Affine3d world_T_imu_at_point;
    if (!trajectory.GetPose(point_timestamp_ns, &world_T_imu_at_point)) {
      continue;
    }

    const Eigen::Vector3d compensated =
        lidar_at_frame_T_world * world_T_imu_at_point * imu_T_lidar *
        Eigen::Vector3d(point.x, point.y, point.z);
    PointXYZIRT compensated_point = point;
    compensated_point.x = compensated.x();
    compensated_point.y = compensated.y();
    compensated_point.z = compensated.z();
    output->push_back(compensated_point);
  }

  output->width = output->size();
  output->height = 1;
  return !output->empty();
}

cv::Scalar IntensityColor(float intensity) {
  const double normalized =
      std::isfinite(intensity)
          ? std::clamp(static_cast<double>(intensity) / kMaxIntensity, 0.0, 1.0)
          : 0.0;
  const int color_index = std::lround(std::sqrt(normalized) * (kMaxIntensity));

  static const cv::Mat turbo_lut = [] {
    cv::Mat values(1, 256, CV_8UC1);
    for (int i = 0; i < values.cols; ++i) {
      values.at<uint8_t>(0, i) = static_cast<uint8_t>(i);
    }
    cv::Mat colors;
    cv::applyColorMap(values, colors, cv::COLORMAP_TURBO);
    return colors;
  }();
  const cv::Vec3b& color = turbo_lut.at<cv::Vec3b>(0, color_index);
  return cv::Scalar(color[0], color[1], color[2]);
}

size_t ProjectCloud(const PointCloudXYZIRT& cloud,
                    const Eigen::Affine3d& camera_T_lidar,
                    const BaseCamera& camera, cv::Mat* image) {
  size_t projected_count = 0;
  for (const PointXYZIRT& point : cloud) {
    const Eigen::Vector3d point_camera =
        camera_T_lidar * Eigen::Vector3d(point.x, point.y, point.z);
    if (point_camera.z() <= 0.0) {
      continue;
    }

    Eigen::Vector2d pixel;
    if (!camera.Project(point_camera, &pixel)) {
      continue;
    }
    const int u = std::lround(pixel.x());
    const int v = std::lround(pixel.y());
    if (u < 0 || u >= image->cols || v < 0 || v >= image->rows) {
      continue;
    }

    const cv::Point center(u, v);
    cv::circle(*image, center, 1, IntensityColor(point.intensity), -1,
               cv::LINE_AA);
    ++projected_count;
  }
  return projected_count;
}

std::string OutputFileName(const Frame& lidar_frame,
                           const Frame& camera_frame) {
  return "lidar_" + std::to_string(lidar_frame.timestamp_ns()) + "_camera_" +
         std::to_string(camera_frame.timestamp_ns()) + ".png";
}

void DrawTimestampDifference(const std::string& title,
                             int64_t lidar_timestamp_ns,
                             int64_t camera_timestamp_ns, cv::Mat* image) {
  const double time_diff_ms =
      static_cast<double>(camera_timestamp_ns - lidar_timestamp_ns) / 1e6;
  const std::string text = cv::format("%s: camera_ts - lidar_ts = %+.3f ms",
                                      title.c_str(), time_diff_ms);
  const cv::Point origin(20, 40);
  cv::putText(*image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
  cv::putText(*image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_camera_metadata.empty()) << "--camera_metadata is required";
  CHECK(!FLAGS_localization_gnss.empty()) << "--localization_gnss is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK(std::isfinite(FLAGS_camera_exposure_time_ms));
  CHECK_GE(FLAGS_camera_exposure_time_ms, 0.0);

  const double camera_time_compensation_ms =
      FLAGS_camera_exposure_time_ms / 2.0;
  const int64_t camera_time_compensation_ns =
      static_cast<int64_t>(std::llround(camera_time_compensation_ms * 1e6));

  const std::filesystem::path data_root(FLAGS_data_root);
  const std::filesystem::path lidar_metadata_path(FLAGS_lidar_metadata);
  const std::filesystem::path camera_metadata_path(FLAGS_camera_metadata);
  const std::filesystem::path gnss_path(FLAGS_localization_gnss);
  const std::filesystem::path output_dir(FLAGS_output_dir);
  CHECK(lidar_metadata_path.is_absolute());
  CHECK(camera_metadata_path.is_absolute());

  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create " << output_dir.string();

  std::vector<Frame> lidar_frames =
      ReadMetaFile<Frame>(lidar_metadata_path.string());
  std::vector<Frame> camera_frames =
      ReadMetaFile<Frame>(camera_metadata_path.string());
  CHECK(!lidar_frames.empty());
  CHECK(!camera_frames.empty());
  const auto by_timestamp = [](const Frame& lhs, const Frame& rhs) {
    return lhs.timestamp_ns() < rhs.timestamp_ns();
  };
  std::sort(lidar_frames.begin(), lidar_frames.end(), by_timestamp);
  std::sort(camera_frames.begin(), camera_frames.end(), by_timestamp);

  const GnssTrajectory trajectory = LoadGnssTrajectory(gnss_path);
  auto data_reader = std::make_shared<LocalDataReader>(data_root.string());

  TimestampAligner aligner(kMaxCameraLidarTimeDiffNs);
  aligner.AddSensorFrameRefs("lidar", lidar_frames);
  aligner.AddSensorFrameRefs("camera", camera_frames);
  const std::vector<AlignedFrames> aligned_frames =
      aligner.Align("lidar", true);

  size_t output_count = 0;
  for (const AlignedFrames& aligned : aligned_frames) {
    const auto camera_ref = aligned.aligned_frames.find("camera");
    if (camera_ref == aligned.aligned_frames.end()) {
      continue;
    }
    const Frame& lidar_frame = lidar_frames[aligned.reference_frame.index];
    const Frame& camera_frame = camera_frames[camera_ref->second.index];
    const int64_t compensated_camera_timestamp_ns =
        camera_frame.timestamp_ns() + camera_time_compensation_ns;
    if (!camera_frame.has_camera_image_uri() ||
        !camera_frame.has_camera_calibration() ||
        !camera_frame.has_sensor_to_imu_extrinsic()) {
      continue;
    }

    Eigen::Affine3d world_T_imu_at_lidar;
    Eigen::Affine3d world_T_imu_at_camera_before;
    Eigen::Affine3d world_T_imu_at_camera_after;
    if (!trajectory.GetPose(lidar_frame.timestamp_ns(),
                            &world_T_imu_at_lidar) ||
        !trajectory.GetPose(camera_frame.timestamp_ns(),
                            &world_T_imu_at_camera_before) ||
        !trajectory.GetPose(compensated_camera_timestamp_ns,
                            &world_T_imu_at_camera_after)) {
      continue;
    }

    PointCloudXYZIRT::Ptr raw_cloud(new PointCloudXYZIRT);
    Eigen::Affine3d imu_T_lidar;
    if (!LoadLidarCloud(lidar_frame, data_reader, raw_cloud, &imu_T_lidar)) {
      continue;
    }

    FrameData camera_data;
    if (!GenerateCameraFrameData(camera_frame, &camera_data, data_reader)) {
      continue;
    }

    PointCloudXYZIRT deskewed_cloud;
    if (!DeskewCloud(*raw_cloud, imu_T_lidar, world_T_imu_at_lidar, trajectory,
                     &deskewed_cloud)) {
      continue;
    }

    const Eigen::Affine3d world_T_lidar = world_T_imu_at_lidar * imu_T_lidar;
    const Eigen::Affine3d world_T_camera_before =
        world_T_imu_at_camera_before * camera_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d world_T_camera_after =
        world_T_imu_at_camera_after * camera_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d camera_before_T_lidar =
        world_T_camera_before.inverse() * world_T_lidar;
    const Eigen::Affine3d camera_after_T_lidar =
        world_T_camera_after.inverse() * world_T_lidar;

    const std::unique_ptr<BaseCamera> camera =
        CreateCamera(camera_data.camera_calibration);
    cv::Mat before_image = camera_data.camera_image.clone();
    cv::Mat after_image = camera_data.camera_image.clone();
    const size_t before_projected_count = ProjectCloud(
        deskewed_cloud, camera_before_T_lidar, *camera, &before_image);
    const size_t after_projected_count = ProjectCloud(
        deskewed_cloud, camera_after_T_lidar, *camera, &after_image);
    DrawTimestampDifference("before", lidar_frame.timestamp_ns(),
                            camera_frame.timestamp_ns(), &before_image);
    const std::string after_title =
        cv::format("after (+%.3f ms)", camera_time_compensation_ms);
    DrawTimestampDifference(after_title, lidar_frame.timestamp_ns(),
                            compensated_camera_timestamp_ns, &after_image);

    cv::Mat comparison_image;
    cv::hconcat(before_image, after_image, comparison_image);
    const std::filesystem::path output_path =
        output_dir / OutputFileName(lidar_frame, camera_frame);
    CHECK(cv::imwrite(output_path.string(), comparison_image))
        << "failed to write " << output_path.string();
    ++output_count;
    LOG(INFO) << "saved " << output_path.string()
              << ", before_projected_points=" << before_projected_count
              << ", after_projected_points=" << after_projected_count;
  }

  LOG(INFO) << "saved " << output_count << " images";
  return output_count == 0 ? 1 : 0;
}

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return adlabel::mapping::Run();
}
