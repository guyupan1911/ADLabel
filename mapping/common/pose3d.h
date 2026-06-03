#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mapping/protos/pose.pb.h"

namespace adlabel {
namespace mapping {

class Pose3D {
  public:
    Pose3D() : translation_(Eigen::Vector3d::Zero()), quaternion_(Eigen::Quaterniond::Identity()) {}

    Pose3D(const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion)
        : translation_(translation), quaternion_(quaternion) {}

    Pose3D(double x, double y, double z, const Eigen::Quaterniond& quaternion)
        : translation_(Eigen::Vector3d(x, y, z)), quaternion_(quaternion) {}

    explicit Pose3D(const Pose3DMessage& pose3d)
        : translation_(Eigen::Vector3d(pose3d.x(), pose3d.y(), pose3d.z())),
          quaternion_(Eigen::Quaterniond(pose3d.qw(), pose3d.qx(), pose3d.qy(), pose3d.qz())) {}

    Pose3D(double x, double y, double z, double qw, double qx, double qy, double qz)
        : translation_(Eigen::Vector3d(x, y, z)), quaternion_(Eigen::Quaterniond(qw, qx, qy, qz)) {}

    explicit Pose3D(const Eigen::Affine3d& affine3d)
        : translation_(affine3d.translation()), quaternion_(affine3d.rotation()) {}

    Pose3D(const Pose3D& pose)
        : translation_(pose.GetTranslation()), quaternion_(pose.GetQuaternion()) {}

    Pose3D& operator=(const Pose3D& pose) {
        if (this == &pose) {
            return *this;
        }
        translation_ = pose.GetTranslation();
        quaternion_ = pose.GetQuaternion();
        return *this;
    }

    /*
     *  Get Pose 3D Message
     */
    Pose3DMessage GetPose3DMessage() const;
    /*
     *  Get Transformation Matrix
     */
    Eigen::Matrix4d GetTransformation() const;

    /*
     *  Get Translation Matrix
     */
    Eigen::Vector3d GetTranslation() const;

    /*
     *  Get Rotation Matrix
     */
    Eigen::Matrix3d GetRotation() const;

    /*
     *  Get Inverse Rotation Matrix
     */
    Eigen::Matrix3d GetRotationInv() const;

    /*
     *  Get Quaternion
     */
    Eigen::Quaterniond GetQuaternion() const;

    /*
     *  Get Affine3d
     */
    Eigen::Affine3d GetAffine3D() const;

  private:
    Eigen::Vector3d translation_;
    Eigen::Quaterniond quaternion_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping
}  // namespace adlabel
