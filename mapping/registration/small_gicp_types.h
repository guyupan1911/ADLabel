#pragma once

#include <cstddef>

#include <Eigen/Geometry>
#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace adlabel {
namespace mapping {

using SmallGicpPointCloud = small_gicp::PointCloud;
using SmallGicpPointCloudPtr = SmallGicpPointCloud::Ptr;
using SmallGicpKdTree = small_gicp::KdTree<SmallGicpPointCloud>;
using SmallGicpKdTreePtr = SmallGicpKdTree::Ptr;
using SmallGicpIncrementalVoxelMap =
    small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>;
using SmallGicpIncrementalVoxelMapPtr = SmallGicpIncrementalVoxelMap::Ptr;

struct SmallGicpRegistrationResult {
  Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();
  bool converged = false;
  std::size_t iterations = 0;
  double inlier_ratio = 0.0;
  double error = 0.0;
};

}  // namespace mapping
}  // namespace adlabel
