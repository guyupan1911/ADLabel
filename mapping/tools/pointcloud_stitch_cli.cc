#include <cmath>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata,
              "20260503T173334_pdb-l4e-c0002_016_40to60/metadata/"
              "lidar/em4_front_lidar.meta",
              "path to lidar metadata, relative to data_root or absolute");
DEFINE_string(data_root, "data/plus_mapping", "root directory for sensor data");
DEFINE_string(output_dir, "data/plus_mapping/pointcloud_stitch",
              "directory to save stitched cloud.pcd");
DEFINE_double(distance, 0.0,
              "trajectory split distance in meters, 0 disables splitting");

using namespace adlabel::mapping;

namespace {

double PoseDistance2D(const Eigen::Affine3d& left,
                      const Eigen::Affine3d& right) {
  const Eigen::Vector3d delta = right.translation() - left.translation();
  return std::hypot(delta.x(), delta.y());
}

std::string SegmentCloudFileName(size_t segment_index) {
  std::ostringstream stream;
  stream << "cloud_" << std::setw(3) << std::setfill('0') << segment_index
         << ".pcd";
  return stream.str();
}

bool SaveStitchedCloud(const PointCloudXYZIRT::ConstPtr& stitched_cloud,
                       size_t loaded_frame_count,
                       const std::filesystem::path& output_path) {
  if (stitched_cloud->empty()) {
    LOG(WARNING) << "skip empty stitched cloud: " << output_path.string();
    return false;
  }

  if (pcl::io::savePCDFileBinary(output_path.string(), *stitched_cloud) < 0) {
    LOG(ERROR) << "failed to save stitched cloud: " << output_path.string();
    return false;
  }

  LOG(INFO) << "stitched " << loaded_frame_count << " frames, saved "
            << stitched_cloud->size() << " points to " << output_path.string();
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
  LOG(INFO) << "localization pose source: pose_utm";
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
    if (!frame.has_lio_pose_3d()) {
      LOG(WARNING) << "skip frame without lio_pose_3d: " << frame.fid();
      continue;
    }
    if (!frame.has_sensor_to_imu_extrinsic()) {
      LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: "
                   << frame.fid();
      continue;
    }

    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
      LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
      continue;
    }
    if (lidar_frame_data.ground_cloud == nullptr) {
      LOG(WARNING) << "skip frame without valid ground labels: " << frame.fid();
      continue;
    }

    const PointCloudXYZIRT::ConstPtr cloud = lidar_frame_data.ground_cloud;
    LOG(INFO) << "loaded " << cloud->size() << " ground points from "
              << frame.cloud_uri();

    const Eigen::Affine3d& current_imu_pose = lidar_frame_data.pose_utm;
    if (split_by_distance && has_last_imu_pose && loaded_frame_count > 0) {
      const double frame_distance =
          PoseDistance2D(last_imu_pose, current_imu_pose);
      if (current_segment_distance + frame_distance > FLAGS_distance) {
        const std::filesystem::path segment_output_path =
            output_dir / SegmentCloudFileName(saved_segment_count);
        if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count,
                               segment_output_path)) {
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
      LOG(INFO) << "use frame " << frame.fid() << " as stitch origin"
                << (split_by_distance ? " for current segment" : "");
    }

    const Eigen::Affine3d lidar_to_first_imu =
        first_imu_pose_inv * current_imu_pose *
        lidar_frame_data.transform_from_sensor_to_imu;

    PointCloudXYZIRT::Ptr transformed(new PointCloudXYZIRT);
    pcl::transformPointCloud(*cloud, *transformed,
                             lidar_to_first_imu.cast<float>());

    *stitched_cloud += *transformed;
    ++loaded_frame_count;
    last_imu_pose = current_imu_pose;
    has_last_imu_pose = true;

    LOG(INFO) << "frame " << frame.fid() << " input: " << cloud->size()
              << " transformed: " << transformed->size()
              << " stitched total: " << stitched_cloud->size();
  }

  if (stitched_cloud->empty()) {
    LOG(ERROR) << "no points stitched, nothing to save";
    return 1;
  }

  const std::filesystem::path output_path =
      split_by_distance ? output_dir / SegmentCloudFileName(saved_segment_count)
                        : output_dir / "cloud.pcd";
  if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count, output_path)) {
    return 1;
  }
  if (split_by_distance) {
    ++saved_segment_count;
    LOG(INFO) << "saved " << saved_segment_count
              << " point cloud segments with split distance " << FLAGS_distance
              << " meters";
  }

  return 0;
}
