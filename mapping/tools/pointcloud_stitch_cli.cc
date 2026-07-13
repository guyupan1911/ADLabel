#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata,
    "20260503T173334_pdb-l4e-c0002_016_40to60/metadata/"
    "lidar/em4_front_lidar.meta", "path to lidar metadata, relative to data_root or absolute");
DEFINE_string(data_root, "data/plus_mapping", "root directory for sensor data");
DEFINE_string(output_dir, "data/plus_mapping/pointcloud_stitch",
    "directory to save stitched cloud.pcd");
DEFINE_double(frame_leaf_size, 0.4, "voxel grid leaf size before stitching, in meters");
DEFINE_double(output_leaf_size, 0.4, "voxel grid leaf size after stitching, in meters");
DEFINE_double(distance, 0.0,
    "trajectory split distance in meters, 0 disables splitting");

using namespace adlabel::mapping;

namespace {

constexpr double kSingleFrameRoiForwardRangeM = 100.0;
constexpr double kSingleFrameRoiLateralRangeM = 100.0;

const Pose3DMessage* GetLocalizationPose(const Frame& frame) {
  return frame.has_refined_pose_3d() ? &frame.refined_pose_3d() : nullptr;
}

const char* LocalizationPoseName() {
  return "refined_pose_3d";
}

PointCloudXYZIRT::Ptr PassThroughCloud(const PointCloudXYZIRT::ConstPtr& cloud,
                                       const std::string& field_name,
                                       double min_value,
                                       double max_value) {
  pcl::PassThrough<PointXYZIRT> pass_through;
  pass_through.setInputCloud(cloud);
  pass_through.setFilterFieldName(field_name);
  pass_through.setFilterLimits(
      static_cast<float>(min_value), static_cast<float>(max_value));

  PointCloudXYZIRT::Ptr filtered(new PointCloudXYZIRT);
  pass_through.filter(*filtered);
  return filtered;
}

PointCloudXYZIRT::Ptr CropSingleFrameCloud(const PointCloudXYZIRT::ConstPtr& cloud) {
  PointCloudXYZIRT::Ptr filtered = PassThroughCloud(
      cloud, "x", -kSingleFrameRoiForwardRangeM, kSingleFrameRoiForwardRangeM);
  filtered = PassThroughCloud(
      filtered, "y", -kSingleFrameRoiLateralRangeM, kSingleFrameRoiLateralRangeM);

  // LOG(INFO) << "single-frame ROI crop: " << cloud->size()
  //           << " -> " << filtered->size()
  //           << " points, x=[" << -kSingleFrameRoiForwardRangeM
  //           << ", " << kSingleFrameRoiForwardRangeM
  //           << "], y=[" << -kSingleFrameRoiLateralRangeM
  //           << ", " << kSingleFrameRoiLateralRangeM
  //           << "], z=unlimited";
  return filtered;
}

PointCloudXYZIRT::Ptr DownsampleCloud(const PointCloudXYZIRT::ConstPtr& cloud, double leaf_size) {
  if (leaf_size <= 0.0) {
    PointCloudXYZIRT::Ptr copy(new PointCloudXYZIRT);
    *copy = *cloud;
    return copy;
  }

  pcl::VoxelGrid<PointXYZIRT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size));

  PointCloudXYZIRT::Ptr downsampled(new PointCloudXYZIRT);
  voxel_grid.filter(*downsampled);
  return downsampled;
}

double PoseDistance2D(const Eigen::Affine3d& left, const Eigen::Affine3d& right) {
  const Eigen::Vector3d delta = right.translation() - left.translation();
  return std::hypot(delta.x(), delta.y());
}

std::string SegmentCloudFileName(size_t segment_index) {
  std::ostringstream stream;
  stream << "cloud_" << std::setw(3) << std::setfill('0') << segment_index << ".pcd";
  return stream.str();
}

bool SaveStitchedCloud(const PointCloudXYZIRT::ConstPtr& stitched_cloud,
                       size_t loaded_frame_count,
                       const std::filesystem::path& output_path) {
  if (stitched_cloud->empty()) {
    LOG(WARNING) << "skip empty stitched cloud: " << output_path.string();
    return false;
  }

  PointCloudXYZIRT::Ptr output_cloud = DownsampleCloud(stitched_cloud, FLAGS_output_leaf_size);
  if (pcl::io::savePCDFileBinary(output_path.string(), *output_cloud) < 0) {
    LOG(ERROR) << "failed to save stitched cloud: " << output_path.string();
    return false;
  }

  LOG(INFO) << "stitched " << loaded_frame_count
            << " frames, before final downsample: " << stitched_cloud->size()
            << ", saved: " << output_cloud->size()
            << " points to " << output_path.string();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK_GE(FLAGS_distance, 0.0) << "--distance must be non-negative";

  auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
  auto lidar_frames = data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
  LOG(INFO) << "lidar_frames size: " << lidar_frames.size();
  LOG(INFO) << "localization pose source: " << LocalizationPoseName();
  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    LOG(ERROR) << "failed to create output_dir: " << FLAGS_output_dir
               << ", error: " << error.message();
    return 1;
  }

  const bool split_by_distance = FLAGS_distance > 0.0;

  PointCloudXYZIRT::Ptr stitched_cloud(new PointCloudXYZIRT);
  size_t loaded_frame_count = 0;
  size_t saved_segment_count = 0;
  double current_segment_distance = 0.0;
  Eigen::Affine3d first_imu_pose_inv = Eigen::Affine3d::Identity();
  bool has_first_imu_pose = false;
  Eigen::Affine3d last_imu_pose = Eigen::Affine3d::Identity();
  bool has_last_imu_pose = false;

  for (const auto& frame : lidar_frames) {
    if (!frame.has_cloud_uri()) {
      LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
      continue;
    }
    const Pose3DMessage* localization_pose = GetLocalizationPose(frame);
    if (localization_pose == nullptr) {
      LOG(WARNING) << "skip frame without "
                   << LocalizationPoseName()
                   << ": " << frame.fid();
      continue;
    }
    if (!frame.has_sensor_to_imu_extrinsic()) {
      LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: " << frame.fid();
      continue;
    }

    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
      LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
      continue;
    }
    PointCloudXYZIRT::Ptr cloud = lidar_frame_data.raw_cloud;
    LOG(INFO) << "loaded filtered cloud " << cloud->size()
              << " points from " << frame.cloud_uri();
    cloud = CropSingleFrameCloud(cloud);

    const Pose3D current_pose(*localization_pose);
    const Eigen::Affine3d current_imu_pose = current_pose.GetAffine3D();
    if (split_by_distance && has_last_imu_pose && loaded_frame_count > 0) {
      const double frame_distance = PoseDistance2D(last_imu_pose, current_imu_pose);
      if (current_segment_distance + frame_distance > FLAGS_distance) {
        const std::filesystem::path segment_output_path =
            output_dir / SegmentCloudFileName(saved_segment_count);
        if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count, segment_output_path)) {
          return 1;
        }
        ++saved_segment_count;

        stitched_cloud.reset(new PointCloudXYZIRT);
        loaded_frame_count = 0;
        current_segment_distance = 0.0;
        has_first_imu_pose = false;
        has_last_imu_pose = false;
        LOG(INFO) << "start new point cloud segment at frame " << frame.fid();
      } else {
        current_segment_distance += frame_distance;
      }
    }

    if (!has_first_imu_pose) {
      first_imu_pose_inv = current_imu_pose.inverse();
      has_first_imu_pose = true;
      LOG(INFO) << "use frame " << frame.fid()
                << " as stitch origin"
                << (split_by_distance ? " for current segment" : "");
    }

    const Eigen::Affine3d lidar_to_first_imu =
        first_imu_pose_inv * current_imu_pose * lidar_frame_data.transform_from_sensor_to_imu;

    PointCloudXYZIRT::Ptr transformed(new PointCloudXYZIRT);
    pcl::transformPointCloud(*cloud, *transformed, lidar_to_first_imu.cast<float>());

    PointCloudXYZIRT::Ptr downsampled = DownsampleCloud(transformed, FLAGS_frame_leaf_size);
    *stitched_cloud += *downsampled;
    ++loaded_frame_count;
    last_imu_pose = current_imu_pose;
    has_last_imu_pose = true;

    LOG(INFO) << "frame " << frame.fid()
              << " filtered: " << cloud->size()
              << " transformed: " << transformed->size()
              << " downsampled: " << downsampled->size()
              << " stitched total: " << stitched_cloud->size();
  }

  if (stitched_cloud->empty()) {
    LOG(ERROR) << "no points stitched, nothing to save";
    return 1;
  }

  const std::filesystem::path output_path =
      split_by_distance
          ? output_dir / SegmentCloudFileName(saved_segment_count)
          : output_dir / "cloud.pcd";
  if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count, output_path)) {
    return 1;
  }
  if (split_by_distance) {
    ++saved_segment_count;
    LOG(INFO) << "saved " << saved_segment_count
              << " point cloud segments with split distance "
              << FLAGS_distance << " meters";
  }

  return 0;
}
