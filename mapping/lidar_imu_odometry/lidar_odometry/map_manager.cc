#include "mapping/lidar_imu_odometry/lidar_odometry/map_manager.h"

#include <memory>

#include "mapping/registration/small_gicp_adapter.h"

namespace adlabel {
namespace mapping {

namespace {

constexpr double kDownsampleResolution = 0.25;
constexpr double kVoxelMapResolution = 1.0;

}  // namespace

Eigen::Affine3d MapManager::AlignScanToMap(
    const PointCloudXYZIRT& cloud, SmallGicpRegistrationResult& align_result) {
  align_result = SmallGicpRegistrationResult();

  SmallGicpPointCloudPtr source_cloud = ToSmallGicpPointCloud(cloud);
  auto downsampled_source_cloud =
      VoxelGridDownsample(*source_cloud, kDownsampleResolution);
  auto source_cloud_kdtree = BuildKdTree(downsampled_source_cloud);
  EstimateCovariances(*downsampled_source_cloud, *source_cloud_kdtree);

  if (voxel_map_ == nullptr) {
    voxel_map_ =
        std::make_shared<SmallGicpIncrementalVoxelMap>(kVoxelMapResolution);
    // voxel_map_->set_search_offsets(27);
    voxel_map_->insert(*downsampled_source_cloud);
    return T_world_lidar_;
  }

  align_result = AlignGicp(*voxel_map_, *downsampled_source_cloud,
                           Eigen::Isometry3d(T_world_lidar_.matrix()));
  T_world_lidar_ = Eigen::Affine3d(align_result.T_target_source.matrix());
  voxel_map_->insert(*downsampled_source_cloud, align_result.T_target_source);

  return T_world_lidar_;
}

}  // namespace mapping
}  // namespace adlabel
