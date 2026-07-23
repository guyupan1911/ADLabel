

#include "mapping/common/pose3d.h"

#include <fstream>

namespace adlabel {
namespace mapping {

Eigen::Matrix4d Pose3D::GetTransformation() const {
  Eigen::Matrix4d transform;
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
  transform.block<3, 1>(0, 3) = translation_;
  return transform;
}

Pose3DMessage Pose3D::GetPose3DMessage() const {
  Pose3DMessage pose;
  pose.set_x(translation_(0));
  pose.set_y(translation_(1));
  pose.set_z(translation_(2));
  pose.set_qw(quaternion_.w());
  pose.set_qx(quaternion_.x());
  pose.set_qy(quaternion_.y());
  pose.set_qz(quaternion_.z());
  return pose;
}

Eigen::Vector3d Pose3D::GetTranslation() const { return translation_; }

Eigen::Matrix3d Pose3D::GetRotation() const {
  return quaternion_.toRotationMatrix();
}

Eigen::Matrix3d Pose3D::GetRotationInv() const {
  return GetRotation().transpose().eval();
}

Eigen::Quaterniond Pose3D::GetQuaternion() const { return quaternion_; }

Eigen::Affine3d Pose3D::GetAffine3D() const {
  Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
  affine3d.linear() = quaternion_.toRotationMatrix();
  affine3d.translation() = translation_;
  return affine3d;
}

}  // namespace mapping
}  // namespace adlabel
