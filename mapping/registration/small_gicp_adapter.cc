#include "mapping/registration/small_gicp_adapter.h"

#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <small_gicp/ann/kdtree_tbb.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>

namespace adlabel {
namespace mapping {

SmallGicpPointCloudPtr ToSmallGicpPointCloud(const PointCloudXYZIRT& cloud) {
  auto output = std::make_shared<SmallGicpPointCloud>();
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

SmallGicpPointCloudPtr VoxelGridDownsample(const SmallGicpPointCloud& cloud,
                                           double leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0) {
    return nullptr;
  }

  return small_gicp::voxelgrid_sampling_tbb(cloud, leaf_size);
}

SmallGicpKdTreePtr BuildKdTree(const SmallGicpPointCloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return nullptr;
  }

  return std::make_shared<SmallGicpKdTree>(cloud,
                                           small_gicp::KdTreeBuilderTBB());
}

void EstimateCovariances(SmallGicpPointCloud& cloud, SmallGicpKdTree& kdtree,
                         int num_neighbors) {
  if (cloud.empty() || num_neighbors <= 0) {
    return;
  }

  small_gicp::estimate_covariances_tbb(cloud, kdtree, num_neighbors);
}

SmallGicpRegistrationResult AlignGicp(
    const SmallGicpPointCloud& target, const SmallGicpPointCloud& source,
    const SmallGicpKdTree& target_kdtree,
    const Eigen::Isometry3d& initial_target_source,
    const SmallGicpRegistrationOptions& options) {
  small_gicp::Registration<small_gicp::GICPFactor,
                           small_gicp::ParallelReductionTBB>
      registration;
  registration.rejector.max_dist_sq =
      options.max_correspondence_distance * options.max_correspondence_distance;
  registration.criteria.rotation_eps = options.rotation_epsilon;
  registration.criteria.translation_eps = options.translation_epsilon;
  registration.optimizer.max_iterations = options.max_iterations;
  registration.optimizer.verbose = options.verbose;

  const small_gicp::RegistrationResult internal_result =
      registration.align(target, source, target_kdtree, initial_target_source);

  SmallGicpRegistrationResult result;
  result.T_target_source = internal_result.T_target_source;
  result.converged = internal_result.converged;
  result.iterations = internal_result.iterations;
  result.error = internal_result.error;
  if (!source.empty()) {
    result.inlier_ratio = static_cast<double>(internal_result.num_inliers) /
                          static_cast<double>(source.size());
  }
  return result;
}

}  // namespace mapping
}  // namespace adlabel
