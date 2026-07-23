#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace adlabel {
namespace mapping {

class D2DCostFunctor {
 public:
  D2DCostFunctor(const Eigen::Vector3d& mean_i,
                 const Eigen::Matrix3d& covariance_i,
                 const Eigen::Vector3d& mean_j,
                 const Eigen::Matrix3d& covariance_j)
      : mean_i_(mean_i),
        covariance_i_(covariance_i),
        mean_j_(mean_j),
        covariance_j_(covariance_j) {}

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation,
                  T* residuals) const {
    Eigen::Map<const Eigen::Quaternion<T>> estimate_rotation(rotation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> estimate_translation(translation);

    Eigen::Matrix<T, 3, 1> mean_delta =
        estimate_rotation * mean_i_.template cast<T>() + estimate_translation -
        mean_j_.template cast<T>();
    const Eigen::Matrix<T, 3, 3> rotation_matrix =
        estimate_rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> covariance = rotation_matrix *
                                            covariance_i_.template cast<T>() *
                                            rotation_matrix.transpose() +
                                        covariance_j_.template cast<T>();

    // cholesky decomposition
    Eigen::Matrix<T, 3, 3> L = Eigen::Matrix<T, 3, 3>::Zero();
    L(0, 0) = ceres::sqrt(covariance(0, 0));
    L(1, 0) = covariance(0, 1) / L(0, 0);
    L(1, 1) = ceres::sqrt(covariance(1, 1) - L(1, 0) * L(1, 0));
    L(2, 0) = covariance(0, 2) / L(0, 0);
    L(2, 1) = (covariance(1, 2) - L(1, 0) * L(2, 0)) / L(1, 1);
    L(2, 2) =
        ceres::sqrt(covariance(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));

    // cov = L * L^T
    // cov^-1 = L^-T * L^-1
    // sqrt info = L^-1
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = L.inverse() * mean_delta;
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& mean_i,
                                     const Eigen::Matrix3d& covariance_i,
                                     const Eigen::Vector3d& mean_j,
                                     const Eigen::Matrix3d& covariance_j) {
    return (new ceres::AutoDiffCostFunction<D2DCostFunctor, 3, 4, 3>(
        new D2DCostFunctor(mean_i, covariance_i, mean_j, covariance_j)));
  }

 private:
  const Eigen::Vector3d mean_i_;
  const Eigen::Matrix3d covariance_i_;
  const Eigen::Vector3d mean_j_;
  const Eigen::Matrix3d covariance_j_;
};

}  // namespace mapping
}  // namespace adlabel