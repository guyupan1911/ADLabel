#pragma once

#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "mapping/loop_closure/proto/ndt_d2d_config.pb.h"
#include "mapping/loop_closure/voxel_grid_covariance.h"

namespace adlabel {
namespace mapping {

class NdtD2D {
 public:
  explicit NdtD2D(const NdtD2DConfig& config) : config_(config) {
    int size = config_.resolutions_size();
    source_grids_.resize(size);
    source_grid_threads_.resize(size);
    target_grids_.resize(size);
    target_grid_threads_.resize(size);
  }

  void SetInputSource(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source);
  void SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target);

  bool Align(const Eigen::Affine3d& init_pose, double* inlier_ratio,
             int* iterations, Eigen::Affine3d* pose);

  void CompuateCovariance(Eigen::Matrix3d* orientation_covariance,
                          Eigen::Matrix3d* position_covariance);

 private:
  bool IsVoxelGridCovarianceValid(VoxelGridCovariance* voxels);
  int AlignOnce(const size_t resolution_index);
  void BuildProblem(const size_t resolution_index, const double neighbor_radius,
                    const int neighbor_count, ceres::Problem* problem,
                    double* inlier_ratio);

  std::vector<VoxelGridCovariance> source_grids_;
  std::vector<std::thread> source_grid_threads_;
  std::vector<VoxelGridCovariance> target_grids_;
  std::vector<std::thread> target_grid_threads_;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud_;

  NdtD2DConfig config_;
  double inlier_ratio_;
  Eigen::Quaterniond aligned_rotation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d aligned_translation_ = Eigen::Vector3d::Zero();
  static constexpr double kCovEigValueInflationRatio = 1e-3;
};

}  // namespace mapping
}  // namespace adlabel