#include "mapping/lidar_imu_odometry/lidar_odometry/map_manager.h"

#include "mapping/registration/small_gicp_adapter.h"

namespace adlabel {
namespace mapping {

Eigen::Affine3d MapManager::AlignScanToMap(
    const PointCloudXYZIRT& cloud, SmallGicpRegistrationResult& align_result) {
  align_result = SmallGicpRegistrationResult();

  SmallGicpPointCloudPtr source_cloud = ToSmallGicpPointCloud(cloud);
  auto downsampled_source_cloud = VoxelGridDownsample(*source_cloud, 0.25);
  auto source_cloud_kdtree = BuildKdTree(downsampled_source_cloud);
  EstimateCovariances(*downsampled_source_cloud, *source_cloud_kdtree);

  // Use the first frame as the reference frame.
  if (reference_cloud_ == nullptr) {
    reference_cloud_ = downsampled_source_cloud;
    reference_cloud_kdtree_ = source_cloud_kdtree;
    return Eigen::Affine3d::Identity();
  }

  align_result = AlignGicp(*reference_cloud_, *downsampled_source_cloud,
                           *reference_cloud_kdtree_);
  reference_cloud_ = downsampled_source_cloud;
  reference_cloud_kdtree_ = source_cloud_kdtree;

  T_world_reference_ = T_world_reference_ * align_result.T_target_source;
  return T_world_reference_;
}

}  // namespace mapping
}  // namespace adlabel
