#pragma once

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "mapping/common/pcl_types.h"

namespace adlabel {
namespace mapping {

using SmallGicpKdTree = small_gicp::KdTree<small_gicp::PointCloud>;

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

}  // namespace mapping
}  // namespace adlabel
