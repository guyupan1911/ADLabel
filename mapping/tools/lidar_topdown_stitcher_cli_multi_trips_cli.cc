#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_roots, "", "comma-separated bag dump root directories");
DEFINE_string(output_dir, "",
              "directory to save fused topdown intensity image");
DEFINE_double(resolution, 0.05, "topdown image resolution, meters per pixel");
DEFINE_double(margin_meters, 50.0, "extra margin around trajectory bounds");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kLidarMetadataPath[] =
    "metadata/lidar/lidar_plusai_unified.meta";

struct LocalFramePose {
  Frame frame;
  Eigen::Vector2d origin_xy{0.0, 0.0};
  std::shared_ptr<LocalDataReader> data_reader;
};

Eigen::Matrix3d EcefToEnuRotation(double lat_deg, double lon_deg) {
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  Eigen::Matrix3d rotation;
  rotation << -sin_lon, cos_lon, 0.0, -sin_lat * cos_lon, -sin_lat * sin_lon,
      cos_lat, cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  return rotation;
}

Eigen::Vector3d EcefToEnu(const Eigen::Vector3d& point_ecef,
                          const Eigen::Vector3d& origin_ecef,
                          const Eigen::Matrix3d& ecef_to_enu_rotation) {
  return ecef_to_enu_rotation * (point_ecef - origin_ecef);
}

int Run() {
  CHECK(!FLAGS_data_roots.empty()) << "--data_roots is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
  CHECK_GE(FLAGS_margin_meters, 0.0) << "--margin_meters must be non-negative";

  std::vector<std::filesystem::path> data_roots;
  std::stringstream data_roots_stream(FLAGS_data_roots);
  std::string data_root_item;
  while (std::getline(data_roots_stream, data_root_item, ',')) {
    if (!data_root_item.empty()) {
      data_roots.emplace_back(data_root_item);
    }
  }
  CHECK(!data_roots.empty()) << "--data_roots has no valid entries";

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error: " << error.message();

  Eigen::Vector3d enu_origin_ecef = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ecef_to_enu_rotation = Eigen::Matrix3d::Identity();
  bool has_enu_origin = false;
  std::vector<LocalFramePose> local_frames;

  for (const auto& data_root : data_roots) {
    const std::filesystem::path lidar_metadata_path =
        data_root / kLidarMetadataPath;
    const std::filesystem::path data_reader_root = data_root.parent_path();
    auto data_reader =
        std::make_shared<LocalDataReader>(data_reader_root.string());
    LOG(INFO) << "data_root=" << data_root.string()
              << ", local_data_reader_root=" << data_reader_root.string();

    const std::vector<Frame> frames =
        ReadMetaFile<Frame>(lidar_metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from "
                           << lidar_metadata_path.string();

    size_t processable_count = 0;
    for (const auto& frame : frames) {
      if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
        LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
        continue;
      }
      if (!frame.has_refined_pose_3d()) {
        LOG(WARNING) << "skip frame without refined_pose_3d: " << frame.fid();
        continue;
      }
      if (!frame.has_sensor_to_imu_extrinsic()) {
        LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: "
                     << frame.fid();
        continue;
      }

      const Eigen::Affine3d pose_ecef =
          Pose3D(frame.refined_pose_3d()).GetAffine3D();
      const Eigen::Vector3d pose_translation_ecef = pose_ecef.translation();
      if (!has_enu_origin) {
        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        GeographicLib::Geocentric::WGS84().Reverse(
            pose_translation_ecef.x(), pose_translation_ecef.y(),
            pose_translation_ecef.z(), lat, lon, height);
        enu_origin_ecef = pose_translation_ecef;
        ecef_to_enu_rotation = EcefToEnuRotation(lat, lon);
        has_enu_origin = true;
      }

      LocalFramePose local_frame;
      local_frame.frame = frame;
      local_frame.origin_xy = EcefToEnu(pose_translation_ecef, enu_origin_ecef,
                                        ecef_to_enu_rotation)
                                  .head<2>();
      local_frame.data_reader = data_reader;
      local_frames.push_back(local_frame);
      ++processable_count;
    }

    LOG(INFO) << "loaded " << frames.size() << " lidar frames from "
              << lidar_metadata_path.string()
              << ", processable=" << processable_count;
  }

  CHECK(!local_frames.empty())
      << "no processable lidar frames in any data_root";
  LOG(INFO) << "fusing " << local_frames.size() << " lidar frames from "
            << data_roots.size() << " trips, origin timestamp="
            << local_frames.front().frame.timestamp_ns();

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& local_frame : local_frames) {
    min_x = std::min(min_x, local_frame.origin_xy.x());
    max_x = std::max(max_x, local_frame.origin_xy.x());
    min_y = std::min(min_y, local_frame.origin_xy.y());
    max_y = std::max(max_y, local_frame.origin_xy.y());
  }

  GridFrame grid_frame;
  grid_frame.resolution = FLAGS_resolution;
  grid_frame.cols = std::max(
      1u, static_cast<unsigned int>(std::ceil(
              (max_x - min_x + 2.0 * FLAGS_margin_meters) / FLAGS_resolution)));
  grid_frame.rows = std::max(
      1u, static_cast<unsigned int>(std::ceil(
              (max_y - min_y + 2.0 * FLAGS_margin_meters) / FLAGS_resolution)));
  grid_frame.top_left_corner = {
      0.5 * (min_x + max_x) -
          static_cast<double>(grid_frame.cols) * 0.5 * FLAGS_resolution,
      0.5 * (min_y + max_y) +
          static_cast<double>(grid_frame.rows) * 0.5 * FLAGS_resolution,
  };

  LidarLosslessMapNode node;
  node.Init(grid_frame,
            LidarLosslessMapNode::IntensityMappingMode::kLogarithmic,
            LidarLosslessMapNode::IntensityAggregationMode::kMean);

  size_t processed_frames = 0;
  for (const auto& local_frame : local_frames) {
    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(local_frame.frame, &lidar_frame_data,
                                local_frame.data_reader)) {
      LOG(ERROR) << "failed to generate lidar frame data: "
                 << local_frame.frame.fid();
      continue;
    }

    const Eigen::Affine3d T_ecef_lidar =
        lidar_frame_data.pose_ecef *
        lidar_frame_data.transform_from_sensor_to_imu;
    for (const auto& point : lidar_frame_data.raw_cloud->points) {
      const Eigen::Vector3d p_ecef =
          T_ecef_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      const Eigen::Vector3d p_enu =
          EcefToEnu(p_ecef, enu_origin_ecef, ecef_to_enu_rotation);
      const float clamped_intensity =
          std::max(0.0f, std::min(255.0f, point.intensity));
      node.SetValue(p_enu, local_frame.frame.sensor_name(),
                    static_cast<unsigned char>(clamped_intensity));
    }

    ++processed_frames;
    if (processed_frames % 50 == 0) {
      LOG(INFO) << "processed " << processed_frames << " lidar frames";
    }
  }

  if (processed_frames == 0) {
    LOG(ERROR) << "no lidar frames were processed";
    return 1;
  }

  const std::filesystem::path output_path =
      output_dir / "topdown_intensity_image.png";
  cv::Mat intensity_image;
  node.GetIntensityImage(&intensity_image, 1);
  if (!cv::imwrite(output_path.string(), intensity_image)) {
    LOG(ERROR) << "failed to write image: " << output_path.string();
    return 1;
  }

  LOG(INFO) << "saved fused topdown intensity image to " << output_path.string()
            << ", frames=" << processed_frames << ", image=" << grid_frame.cols
            << "x" << grid_frame.rows
            << ", occupancy=" << node.GetOccupancyRatio() * 100.0 << "%";
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
