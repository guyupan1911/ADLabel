#include "mapping/loop_closure/ndt_d2d.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>

#include "mapping/loop_closure/d2d_cost_functor.h"

namespace adlabel {
namespace mapping {

namespace {

ceres::LossFunction* CreateLossFunction(const NdtD2DLossType loss_type) {
  switch (loss_type) {
    case NDT_D2D_NULL_LOSS:
      return nullptr;
    case NDT_D2D_ARCTAN_LOSS:
      return new ceres::ArctanLoss(2.0);
    case NDT_D2D_CAUCHY_LOSS:
      return new ceres::CauchyLoss(1.0);
    default:
      return nullptr;
  }
}

}  // namespace

void NdtD2D::SetInputSource(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source) {
  source_cloud_ = source;

  for (int i = 0; i < config_.resolutions_size(); ++i) {
    source_grid_threads_[i] = std::thread([this, i]() {
      double resolution = config_.resolutions(i).resolution();
      source_grids_[i].setLeafSize(resolution, resolution, resolution);
      source_grids_[i].setCovEigValueInflationRatio(kCovEigValueInflationRatio);
      source_grids_[i].setInputCloud(source_cloud_);
      source_grids_[i].filter(true);
    });
  }
}

void NdtD2D::SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target) {
  target_cloud_ = target;

  for (int i = 0; i < config_.resolutions_size(); ++i) {
    target_grid_threads_[i] = std::thread([this, i]() {
      double resolution = config_.resolutions(i).resolution();
      target_grids_[i].setLeafSize(resolution, resolution, resolution);
      target_grids_[i].setCovEigValueInflationRatio(kCovEigValueInflationRatio);
      target_grids_[i].setInputCloud(target_cloud_);
      target_grids_[i].filter(true);
    });
  }
}

bool NdtD2D::Align(const Eigen::Affine3d& init_pose, double* inlier_ratio,
                   int* iterations, Eigen::Affine3d* pose) {
  aligned_rotation_ = Eigen::Quaterniond(init_pose.linear());
  aligned_translation_ = init_pose.translation();

  int iter = 0;
  for (int i = 0; i < config_.resolutions_size(); ++i) {
    if (source_grid_threads_[i].joinable()) {
      source_grid_threads_[i].join();
    }
    if (target_grid_threads_[i].joinable()) {
      target_grid_threads_[i].join();
    }

    if (!IsVoxelGridCovarianceValid(&source_grids_[i]) ||
        !IsVoxelGridCovarianceValid(&target_grids_[i])) {
      return false;
    }

    for (iter = 0; iter < config_.max_num_iterations(); ++iter) {
      // Terminate after converged
      constexpr int kMinSolverIterations = 2;
      if (AlignOnce(i) < kMinSolverIterations) {
        break;
      }
    }
  }

  if (inlier_ratio) {
    *inlier_ratio = inlier_ratio_;
  }
  if (iterations) {
    *iterations = iter;
  }

  CHECK(pose);
  pose->linear() = aligned_rotation_.toRotationMatrix();
  pose->translation() = aligned_translation_;

  return true;
}

void NdtD2D::CompuateCovariance(Eigen::Matrix3d* orientation_covariance,
                                Eigen::Matrix3d* position_covariance) {
  ceres::Problem problem;
  BuildProblem(config_.covariance_resolution_index(),
               config_.covariance_neighbor_radius(),
               config_.covariance_neighbor_count(),
               &problem, nullptr);

  const int residual_block_count = problem.NumResidualBlocks();
  if (residual_block_count == 0) {
    orientation_covariance->setIdentity();
    position_covariance->setIdentity();
    return;
  }

  ceres::Covariance::Options cov_options;
  cov_options.num_threads = config_.num_threads();
  cov_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(cov_options);
  
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(aligned_rotation_.coeffs().data(),
                                 aligned_rotation_.coeffs().data());
  covariance_blocks.emplace_back(aligned_translation_.data(),
                                 aligned_translation_.data());

  if (!covariance.Compute(covariance_blocks, &problem)) {
    LOG(ERROR) << "Failed to compute covariance. Discarding this loop pair.";
    orientation_covariance->setIdentity();
    position_covariance->setIdentity();
    return;
  }

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation_cov;
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> translation_cov;

  covariance.GetCovarianceBlockInTangentSpace(aligned_rotation_.coeffs().data(),
                                              aligned_rotation_.coeffs().data(),
                                              rotation_cov.data());
  covariance.GetCovarianceBlockInTangentSpace(aligned_translation_.data(),
                                              aligned_translation_.data(),
                                              translation_cov.data());



  constexpr double kCovarianceCoeff = 1e-2;
  *orientation_covariance =
      rotation_cov * residual_block_count * kCovarianceCoeff;
  *position_covariance =
      translation_cov * residual_block_count * kCovarianceCoeff;
}

bool NdtD2D::IsVoxelGridCovarianceValid(
    VoxelGridCovariance* voxels) {
  CHECK(voxels != nullptr);
  for (const auto& item : voxels->getLeaves()) {
    if (item.second.getPointCount() >= voxels->getMinPointPerVoxel()) {
      return true;
    }
  }
  return false;
}

int NdtD2D::AlignOnce(const size_t resolution_index) {
  ceres::Problem problem;
  double inlier_ratio = 0.0;

  BuildProblem(resolution_index,
               config_.resolutions(resolution_index).resolution(),
               0/*use all neighbors*/, &problem, &inlier_ratio);
  
  if (problem.NumResidualBlocks() == 0) {
    inlier_ratio_ = 0.0;
    return 0;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = config_.num_threads();
  options.initial_trust_region_radius = 1e6;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  // LOG(INFO) << "resolution: " << config_.resolutions(resolution_index).resolution() << "\n"
  //           << summary.BriefReport();

  inlier_ratio_ = inlier_ratio;
  return summary.iterations.size();
}

void NdtD2D::BuildProblem(const size_t resolution_index,
                          const double neighbor_radius,
                          const int neighbor_count, ceres::Problem* problem,
                          double* inlier_ratio) {
    const NdtD2DLossType loss_type =
        config_.resolutions(resolution_index).loss();

    Eigen::Affine3d aligned_pose;
    aligned_pose.linear() = aligned_rotation_.toRotationMatrix();
    aligned_pose.translation() = aligned_translation_;
    
    uint32_t inlier_count = 0;
    uint32_t total_count = 0;

    for (const auto& item : source_grids_[resolution_index].getLeaves()) {
      const auto& source_leaf = item.second;
      if (source_leaf.getPointCount() <
          source_grids_[resolution_index].getMinPointPerVoxel()) {
        continue;
      }

      pcl::PointXYZ mean(source_leaf.getMean().x(), source_leaf.getMean().y(),
                         source_leaf.getMean().z());
      pcl::PointXYZ transformed_mean = pcl::transformPoint(mean, aligned_pose);
      std::vector<VoxelGridCovariance::LeafConstPtr> neighbors;
      std::vector<float> dists;
      target_grids_[resolution_index].radiusSearch(
        transformed_mean, neighbor_radius, neighbors, dists, neighbor_count);
      ++total_count;

      bool is_inlier = false;
      for (const auto& target_leaf : neighbors) {
        if (target_leaf->getPointCount() <
            target_grids_[resolution_index].getMinPointPerVoxel()) {
          continue;
        }
        is_inlier=true;
        ceres::CostFunction* d2d_cost_function = D2DCostFunctor::Create(
          source_leaf.getMean(), source_leaf.getCov(),
          target_leaf->getMean(), target_leaf->getCov());
        problem->AddResidualBlock(
          d2d_cost_function, CreateLossFunction(loss_type),
          aligned_rotation_.coeffs().data(),
          aligned_translation_.data());
      }
      if (is_inlier) {
        inlier_count++;
      }
    }

    if (problem->NumResidualBlocks() == 0) {
      *inlier_ratio = 0;
      return;
    }

    problem->SetParameterization(aligned_rotation_.coeffs().data(),
                                 new ceres::EigenQuaternionParameterization());
    if (inlier_ratio) {
      *inlier_ratio =
        static_cast<double>(inlier_count) / static_cast<double>(total_count); 
    }
}

}
}