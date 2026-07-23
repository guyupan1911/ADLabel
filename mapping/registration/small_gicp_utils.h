#pragma once

#include <Eigen/Geometry>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_result.hpp>

#include "mapping/common/pcl_types.h"

namespace adlabel {
namespace mapping {

using SmallGicpKdTree = small_gicp::KdTree<small_gicp::PointCloud>;

struct SmallGicpRegistrationOptions {
  double max_correspondence_distance = 1.0;
  double rotation_epsilon = 1.7453292519943296e-3;  // 0.1 degree.
  double translation_epsilon = 1e-3;
  int max_iterations = 20;
  bool verbose = false;
};

struct SmallGicpRegistrationResult {
  small_gicp::RegistrationResult registration;
  double inlier_ratio = 0.0;
};

// Converts an ADLabel point cloud to small_gicp's point cloud representation.
// Only finite XYZ coordinates are copied; all other point fields are discarded.
small_gicp::PointCloud::Ptr ToSmallGicpPointCloud(
    const PointCloudXYZIRT& cloud);

// Downsamples a small_gicp point cloud with an isotropic voxel grid.
// Returns nullptr when leaf_size is not finite or is not positive.
small_gicp::PointCloud::Ptr VoxelGridDownsample(
    const small_gicp::PointCloud& cloud, double leaf_size);

// Builds a KD-Tree with small_gicp's TBB builder.
// The point coordinates and cloud size must not change after this call.
SmallGicpKdTree::Ptr BuildKdTree(const small_gicp::PointCloud::Ptr& cloud);

// Estimates per-point covariances with TBB and stores them in cloud.
void EstimateCovariances(small_gicp::PointCloud& cloud, SmallGicpKdTree& kdtree,
                         int num_neighbors = 20);

// Aligns source to target with GICP and TBB-based parallel reduction.
// Both clouds must contain estimated covariances, and target_kdtree must be
// built from target. The returned transformation maps source into target.
SmallGicpRegistrationResult AlignGicp(
    const small_gicp::PointCloud& target, const small_gicp::PointCloud& source,
    const SmallGicpKdTree& target_kdtree,
    const Eigen::Isometry3d& initial_target_source =
        Eigen::Isometry3d::Identity(),
    const SmallGicpRegistrationOptions& options =
        SmallGicpRegistrationOptions());

}  // namespace mapping
}  // namespace adlabel
