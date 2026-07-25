#pragma once

#include <Eigen/Geometry>

#include "mapping/common/pcl_types.h"
#include "mapping/registration/small_gicp_types.h"

namespace adlabel {
namespace mapping {

struct SmallGicpRegistrationOptions {
  double max_correspondence_distance = 1.0;
  double rotation_epsilon = 1.7453292519943296e-3;  // 0.1 degree.
  double translation_epsilon = 1e-3;
  int max_iterations = 20;
  bool verbose = false;
};

// Converts an ADLabel point cloud to small_gicp's point cloud representation.
// Only finite XYZ coordinates are copied; all other point fields are discarded.
SmallGicpPointCloudPtr ToSmallGicpPointCloud(const PointCloudXYZIRT& cloud);

// Downsamples a small_gicp point cloud with an isotropic voxel grid.
// Returns nullptr when leaf_size is not finite or is not positive.
SmallGicpPointCloudPtr VoxelGridDownsample(const SmallGicpPointCloud& cloud,
                                           double leaf_size);

// Builds a KD-Tree with small_gicp's TBB builder.
// The point coordinates and cloud size must not change after this call.
SmallGicpKdTreePtr BuildKdTree(const SmallGicpPointCloudPtr& cloud);

// Estimates per-point covariances with TBB and stores them in cloud.
void EstimateCovariances(SmallGicpPointCloud& cloud, SmallGicpKdTree& kdtree,
                         int num_neighbors = 20);

// Aligns source to target with GICP and TBB-based parallel reduction.
// Both clouds must contain estimated covariances, and target_kdtree must be
// built from target. The returned transformation maps source into target.
SmallGicpRegistrationResult AlignGicp(
    const SmallGicpPointCloud& target, const SmallGicpPointCloud& source,
    const SmallGicpKdTree& target_kdtree,
    const Eigen::Isometry3d& initial_target_source =
        Eigen::Isometry3d::Identity(),
    const SmallGicpRegistrationOptions& options =
        SmallGicpRegistrationOptions());

// Aligns source to an incremental voxel map. Source must have covariances.
SmallGicpRegistrationResult AlignGicp(
    const SmallGicpIncrementalVoxelMap& target_map,
    const SmallGicpPointCloud& source,
    const Eigen::Isometry3d& initial_target_source =
        Eigen::Isometry3d::Identity(),
    const SmallGicpRegistrationOptions& options =
        SmallGicpRegistrationOptions());

}  // namespace mapping
}  // namespace adlabel
