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
#include "mapping/mapping_utils/stitch_utils.h"
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
DEFINE_int32(point_radius, 1, "projected point radius in pixels");
DEFINE_uint64(max_frames, 0,
              "maximum matched frame pairs to process, 0 means all");
DEFINE_string(
    depth_metric, "euclidean",
    "depth metric for color mapping: 'z_axis' (camera z coordinate) or "
    "'euclidean' (straight-line distance to camera)");

namespace adlabel {
namespace mapping {
namespace {

constexpr double kMaxRenderDepthM = 80.0;
constexpr double kMaxRenderIntensity = 255.0;

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
  if (!frame.has_refined_pose_3d()) {
    LOG(WARNING) << "skip lidar frame without refined_pose_3d: " << frame.fid();
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
  const double t = Clamp(depth_m / kMaxRenderDepthM, 0.0, 1.0);
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

cv::Scalar IntensityColor(double intensity) {
  const double t = Clamp(intensity / kMaxRenderIntensity, 0.0, 1.0);
  return DepthColor((1.0 - t) * kMaxRenderDepthM);
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

SimplePose3DInterpolator BuildPoseEcefInterpolator(
    const std::vector<Frame>& lidar_frames) {
  SimplePose3DInterpolator interpolator;
  size_t valid_pose_count = 0;

  for (const auto& frame : lidar_frames) {
    if (!frame.has_refined_pose_3d()) {
      continue;
    }
    const Pose3D pose_ecef(frame.refined_pose_3d());
    interpolator.InsertTimestampedPose(frame.timestamp_ns(), pose_ecef);
    ++valid_pose_count;
  }

  LOG(INFO) << "built ECEF pose interpolator with " << valid_pose_count
            << " poses from " << lidar_frames.size() << " lidar frames";
  return interpolator;
}

bool InterpolateCameraPoseEcef(
    const SimplePose3DInterpolator& pose_ecef_interpolator,
    int64_t camera_timestamp_ns, FrameData* camera_frame_data) {
  CHECK(camera_frame_data != nullptr);

  Pose3D camera_pose_ecef;
  if (!pose_ecef_interpolator.GetTimestampedPose(camera_timestamp_ns,
                                                 &camera_pose_ecef, false)) {
    LOG(WARNING) << "failed to interpolate ECEF pose at camera timestamp "
                 << camera_timestamp_ns;
    return false;
  }

  camera_frame_data->pose_ecef = camera_pose_ecef.GetAffine3D();
  return true;
}

Eigen::Affine3d ComputeLidarToCameraTransform(
    const FrameData& lidar_frame_data, const FrameData& camera_frame_data) {
  return camera_frame_data.transform_from_sensor_to_imu.inverse() *
         camera_frame_data.pose_ecef.inverse() * lidar_frame_data.pose_ecef *
         lidar_frame_data.transform_from_sensor_to_imu;
}

size_t ProjectLidarToImages(const FrameData::Cloud& cloud,
                            const Eigen::Affine3d& T_lidar_to_camera,
                            const BaseCamera& camera, cv::Mat* depth_image,
                            cv::Mat* intensity_image) {
  CHECK(depth_image != nullptr);
  CHECK(intensity_image != nullptr);
  CHECK_EQ(depth_image->size(), intensity_image->size());

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
    if (u < 0 || u >= depth_image->cols || v < 0 || v >= depth_image->rows) {
      continue;
    }

    // Compute depth based on the selected metric.
    const double depth = (FLAGS_depth_metric == "euclidean")
                             ? point_camera.norm()
                             : point_camera.z();

    cv::circle(*depth_image, cv::Point(u, v), FLAGS_point_radius,
               DepthColor(depth), -1, cv::LINE_AA);
    const double intensity =
        std::isfinite(point.intensity) ? point.intensity : 0.0;
    cv::circle(*intensity_image, cv::Point(u, v), FLAGS_point_radius,
               IntensityColor(intensity), -1, cv::LINE_AA);
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

  const SimplePose3DInterpolator pose_ecef_interpolator =
      BuildPoseEcefInterpolator(lidar_frames);

  TimestampAligner timestamp_aligner(max_time_diff_ns);
  timestamp_aligner.AddSensorFrameRefs("lidar", lidar_frames);
  timestamp_aligner.AddSensorFrameRefs("camera", camera_frames);
  const std::vector<AlignedFrames> aligned_frames =
      timestamp_aligner.Align("lidar", true);
  CHECK(!aligned_frames.empty()) << "no lidar-camera frame pairs matched";

  const std::filesystem::path output_dir(FLAGS_output_dir);
  const std::filesystem::path depth_output_dir = output_dir / "depth";
  const std::filesystem::path intensity_output_dir = output_dir / "intensity";
  std::error_code error;
  std::filesystem::create_directories(depth_output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << depth_output_dir.string()
                << ", error: " << error.message();
  std::filesystem::create_directories(intensity_output_dir, error);
  CHECK(!error) << "failed to create output_dir: "
                << intensity_output_dir.string()
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

    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(lidar_frame, &lidar_frame_data, data_reader)) {
      continue;
    }
    if (lidar_frame_data.ground_cloud == nullptr ||
        lidar_frame_data.ground_cloud->empty()) {
      LOG(WARNING) << "skip lidar frame without ground points: "
                   << lidar_frame.fid();
      continue;
    }

    FrameData camera_frame_data;
    if (!GenerateCameraFrameData(camera_frame, &camera_frame_data,
                                 data_reader)) {
      continue;
    }
    ConvertToBgrIfNeeded(&camera_frame_data.camera_image);

    std::unique_ptr<BaseCamera> camera =
        CreateCamera(camera_frame_data.camera_calibration);
    if (camera->Width() !=
            static_cast<size_t>(camera_frame_data.camera_image.cols) ||
        camera->Height() !=
            static_cast<size_t>(camera_frame_data.camera_image.rows)) {
      LOG(WARNING) << "camera calibration size " << camera->Width() << "x"
                   << camera->Height() << " differs from image size "
                   << camera_frame_data.camera_image.cols << "x"
                   << camera_frame_data.camera_image.rows;
    }

    if (!InterpolateCameraPoseEcef(pose_ecef_interpolator,
                                   camera_frame.timestamp_ns(),
                                   &camera_frame_data)) {
      continue;
    }

    const Eigen::Affine3d T_lidar_to_camera =
        ComputeLidarToCameraTransform(lidar_frame_data, camera_frame_data);
    cv::Mat depth_image = camera_frame_data.camera_image.clone();
    cv::Mat intensity_image = camera_frame_data.camera_image.clone();
    const size_t projected_count =
        ProjectLidarToImages(*lidar_frame_data.ground_cloud, T_lidar_to_camera,
                             *camera, &depth_image, &intensity_image);

    const std::string output_file_name =
        MakeOutputFileName(lidar_frame, camera_frame, pair_index);
    const std::filesystem::path depth_output_path =
        depth_output_dir / output_file_name;
    const std::filesystem::path intensity_output_path =
        intensity_output_dir / output_file_name;
    if (!cv::imwrite(depth_output_path.string(), depth_image)) {
      LOG(ERROR) << "failed to save depth projection image: "
                 << depth_output_path.string();
      continue;
    }
    if (!cv::imwrite(intensity_output_path.string(), intensity_image)) {
      LOG(ERROR) << "failed to save intensity projection image: "
                 << intensity_output_path.string();
      continue;
    }

    ++processed_pairs;
    saved_images += 2;
    LOG(INFO) << "saved projection images: " << output_file_name
              << ", projected_points=" << projected_count << "/"
              << lidar_frame_data.ground_cloud->points.size()
              << ", time_diff_ms="
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
