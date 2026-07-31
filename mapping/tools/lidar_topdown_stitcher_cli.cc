#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for LocalDataReader");
DEFINE_string(lidar_metadata, "", "path to lidar frame metadata file");
DEFINE_string(output_dir, "", "directory to save topdown intensity image");
DEFINE_double(resolution, 0.05, "topdown image resolution, meters per pixel");
DEFINE_int32(max_frames, 0,
             "maximum number of initial valid frames to process; 0 means all");
DEFINE_string(aggregation_mode, "mean",
              "intensity aggregation mode for stitching: mean or max");

namespace adlabel {
namespace mapping {
namespace {

constexpr double kTrajectoryMarginMeters = 30.0;

LidarLosslessMapNode::IntensityAggregationMode ParseAggregationMode(
    const std::string& mode) {
  if (mode == "mean") {
    return LidarLosslessMapNode::IntensityAggregationMode::kMean;
  }
  if (mode == "max") {
    return LidarLosslessMapNode::IntensityAggregationMode::kMax;
  }
  LOG(FATAL) << "unsupported --aggregation_mode=" << mode
             << ", expected mean or max";
  return LidarLosslessMapNode::IntensityAggregationMode::kMean;
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
  CHECK_GE(FLAGS_max_frames, 0) << "--max_frames must not be negative";
  const LidarLosslessMapNode::IntensityAggregationMode aggregation_mode =
      ParseAggregationMode(FLAGS_aggregation_mode);

  const std::filesystem::path data_root(FLAGS_data_root);
  const std::filesystem::path lidar_metadata_path(FLAGS_lidar_metadata);
  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error: " << error.message();

  auto data_reader = std::make_shared<LocalDataReader>(data_root.string());
  LOG(INFO) << "local_data_reader_root=" << data_root.string()
            << ", lidar_metadata=" << lidar_metadata_path.string();

  std::vector<Frame> frames = ReadMetaFile<Frame>(lidar_metadata_path.string());
  CHECK(!frames.empty()) << "no frames loaded from "
                         << lidar_metadata_path.string();
  std::sort(frames.begin(), frames.end(),
            [](const Frame& lhs, const Frame& rhs) {
              return lhs.timestamp_ns() < rhs.timestamp_ns();
            });

  std::vector<Frame> local_frames;
  for (const auto& frame : frames) {
    if (FLAGS_max_frames > 0 &&
        local_frames.size() >= static_cast<size_t>(FLAGS_max_frames)) {
      break;
    }
    if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
      LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
      continue;
    }
    if (!frame.has_refined_pose_3d()) {
      LOG(WARNING) << "skip frame without refined_pose_3d: " << frame.fid();
      continue;
    }
    if (!frame.has_lidar_nn_uri() || frame.lidar_nn_uri().empty()) {
      LOG(WARNING) << "skip frame without lidar nn uri" << frame.fid();
      continue;
    }
    if (!frame.has_sensor_to_imu_extrinsic()) {
      LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: "
                   << frame.fid();
      continue;
    }

    local_frames.push_back(frame);
  }
  CHECK(!local_frames.empty())
      << "no processable lidar frames in " << lidar_metadata_path.string();

  LOG(INFO) << "loaded " << frames.size()
            << " lidar frames, processable=" << local_frames.size()
            << ", origin timestamp=" << local_frames.front().timestamp_ns();

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  const Eigen::Affine3d T_first_imu_ecef =
      Pose3D(local_frames.front().refined_pose_3d()).GetAffine3D().inverse();
  for (const Frame& frame : local_frames) {
    const Eigen::Affine3d T_first_imu_current_imu =
        T_first_imu_ecef * Pose3D(frame.refined_pose_3d()).GetAffine3D();
    const Eigen::Vector3d& position = T_first_imu_current_imu.translation();
    min_x = std::min(min_x, position.x());
    max_x = std::max(max_x, position.x());
    min_y = std::min(min_y, position.y());
    max_y = std::max(max_y, position.y());
  }
  min_x -= kTrajectoryMarginMeters;
  max_x += kTrajectoryMarginMeters;
  min_y -= kTrajectoryMarginMeters;
  max_y += kTrajectoryMarginMeters;

  GridFrame grid_frame;
  grid_frame.resolution = FLAGS_resolution;
  grid_frame.cols =
      std::max(1u, static_cast<unsigned int>(
                       std::ceil((max_x - min_x) / FLAGS_resolution) + 1.0));
  grid_frame.rows =
      std::max(1u, static_cast<unsigned int>(
                       std::ceil((max_y - min_y) / FLAGS_resolution) + 1.0));
  grid_frame.top_left_corner = {min_x, max_y};
  LOG(INFO) << "trajectory bounds with " << kTrajectoryMarginMeters
            << "m margin: x=[" << min_x << ", " << max_x << "], y=[" << min_y
            << ", " << max_y << "], image=" << grid_frame.cols << "x"
            << grid_frame.rows;

  LidarLosslessMapNode node;
  node.Init(grid_frame,
            LidarLosslessMapNode::IntensityMappingMode::kLogarithmic,
            aggregation_mode);

  size_t processed_frames = 0;
  for (const Frame& frame : local_frames) {
    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
      LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
      continue;
    }

    const Eigen::Affine3d T_first_imu_lidar =
        T_first_imu_ecef * lidar_frame_data.pose_ecef *
        lidar_frame_data.transform_from_sensor_to_imu;
    for (const auto& point : lidar_frame_data.ground_cloud->points) {
      const Eigen::Vector3d p_first_imu =
          T_first_imu_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      const float clamped_intensity =
          std::max(0.0f, std::min(255.0f, point.intensity));
      node.SetValue(p_first_imu, frame.sensor_name(),
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

  LOG(INFO) << "saved topdown intensity image to " << output_path.string()
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
