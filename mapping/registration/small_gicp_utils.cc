#include "mapping/registration/small_gicp_utils.h"

#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <small_gicp/ann/kdtree_tbb.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>

namespace adlabel {
namespace mapping {

small_gicp::PointCloud::Ptr ToSmallGicpPointCloud(
    const PointCloudXYZIRT& cloud) {
  auto output = std::make_shared<small_gicp::PointCloud>();
  output->points.reserve(cloud.size());

  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    output->points.emplace_back(static_cast<double>(point.x),
                                static_cast<double>(point.y),
                                static_cast<double>(point.z), 1.0);
  }

  return output;
}

small_gicp::PointCloud::Ptr VoxelGridDownsample(
    const small_gicp::PointCloud& cloud, double leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0) {
    return nullptr;
  }

  return small_gicp::voxelgrid_sampling_tbb(cloud, leaf_size);
}

SmallGicpKdTree::Ptr BuildKdTree(const small_gicp::PointCloud::Ptr& cloud) {
  if (!cloud || cloud->empty()) {
    return nullptr;
  }

  return std::make_shared<SmallGicpKdTree>(cloud,
                                           small_gicp::KdTreeBuilderTBB());
}

void EstimateCovariances(small_gicp::PointCloud& cloud, SmallGicpKdTree& kdtree,
                         int num_neighbors) {
  if (cloud.empty() || num_neighbors <= 0) {
    return;
  }

  small_gicp::estimate_covariances_tbb(cloud, kdtree, num_neighbors);
}

}  // namespace mapping
}  // namespace adlabel
