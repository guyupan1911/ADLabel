#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pcl/common/transforms.h>
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
DEFINE_double(frame_leaf_size, 0.2, "voxel grid leaf size before stitching, in meters");
DEFINE_double(output_leaf_size, 0.2, "voxel grid leaf size after stitching, in meters");

using namespace adlabel::mapping;

namespace {

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

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
  auto lidar_frames = data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
  LOG(INFO) << "lidar_frames size: " << lidar_frames.size();

  PointCloudXYZIRT::Ptr stitched_cloud(new PointCloudXYZIRT);
  size_t loaded_frame_count = 0;
  Eigen::Affine3d first_imu_pose_inv = Eigen::Affine3d::Identity();
  bool has_first_imu_pose = false;

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

    PointCloudXYZIRT::Ptr downsampled = DownsampleCloud(cloud, FLAGS_frame_leaf_size);

    const Pose3D lio_pose(frame.lio_pose_3d());
    if (!has_first_imu_pose) {
      first_imu_pose_inv = lio_pose.GetAffine3D().inverse();
      has_first_imu_pose = true;
      LOG(INFO) << "use frame " << frame.fid() << " as stitch origin";
    }

    const Eigen::Affine3d lidar_to_first_imu =
        first_imu_pose_inv * lio_pose.GetAffine3D() * lidar_frame_data.T_sensor_to_imu;

    PointCloudXYZIRT::Ptr transformed(new PointCloudXYZIRT);
    pcl::transformPointCloud(*downsampled, *transformed, lidar_to_first_imu.cast<float>());
    *stitched_cloud += *transformed;
    ++loaded_frame_count;

    LOG(INFO) << "frame " << frame.fid()
              << " filtered: " << cloud->size()
              << " downsampled: " << downsampled->size()
              << " stitched total: " << stitched_cloud->size();
  }

  if (stitched_cloud->empty()) {
    LOG(ERROR) << "no points stitched, nothing to save";
    return 1;
  }

  PointCloudXYZIRT::Ptr output_cloud = DownsampleCloud(stitched_cloud, FLAGS_output_leaf_size);

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    LOG(ERROR) << "failed to create output_dir: " << FLAGS_output_dir
               << ", error: " << error.message();
    return 1;
  }

  const std::filesystem::path output_path = output_dir / "cloud.pcd";
  if (pcl::io::savePCDFileBinary(output_path.string(), *output_cloud) < 0) {
    LOG(ERROR) << "failed to save stitched cloud: " << output_path.string();
    return 1;
  }

  LOG(INFO) << "stitched " << loaded_frame_count
            << " frames, before final downsample: " << stitched_cloud->size()
            << ", saved: " << output_cloud->size()
            << " points to " << output_path.string();
  return 0;
}
