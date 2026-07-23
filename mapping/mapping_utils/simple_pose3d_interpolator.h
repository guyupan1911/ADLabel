#pragma once

#include <cstdint>
#include <iterator>
#include <map>

#include <glog/logging.h>

#include "mapping/common/pose3d.h"

namespace adlabel {
namespace mapping {

class SimplePose3DInterpolator {
 public:
  void InsertTimestampedPose(const int64_t timestamp_ns, const Pose3D& pose) {
    timestamped_poses_.insert_or_assign(timestamp_ns, pose);
  }

  bool GetTimestampedPose(const int64_t timestamp_ns, Pose3D* result,
                          bool enable_extrapolation = false) const {
    CHECK(result != nullptr);

    auto it = timestamped_poses_.find(timestamp_ns);
    if (it != timestamped_poses_.end()) {
      *result = it->second;
      return true;
    }

    if (timestamped_poses_.size() < 2) {
      return false;
    }

    auto upper = timestamped_poses_.upper_bound(timestamp_ns);

    if (upper == timestamped_poses_.end()) {
      if (!enable_extrapolation) {
        LOG(INFO) << "pose interpolate fail, timestamp: " << timestamp_ns
                  << " exceeds most recent ts: " << std::prev(upper)->first;
        return false;
      }
      auto end = std::prev(upper);
      auto begin = std::prev(end);
      *result = Extrapolate(timestamp_ns, begin->first, begin->second,
                            end->first, end->second);
    } else if (upper == timestamped_poses_.begin()) {
      if (!enable_extrapolation) {
        LOG(INFO) << "pose interpolate fail, timestamp: " << timestamp_ns
                  << " is before the earliest ts: "
                  << timestamped_poses_.begin()->first;
        return false;
      }
      auto begin = upper;
      auto end = std::next(upper);
      *result = Extrapolate(timestamp_ns, begin->first, begin->second,
                            end->first, end->second);
    } else {
      auto end = upper;
      auto begin = std::prev(upper);
      *result = Interpolate(timestamp_ns, begin->first, begin->second,
                            end->first, end->second);
    }
    return true;
  }

  bool GetTimeStampedPose(const int64_t timestamp_ns, Pose3D* result,
                          bool enable_extrapolation = false) const {
    return GetTimestampedPose(timestamp_ns, result, enable_extrapolation);
  }

 private:
  static double TimeDiffRatio(const int64_t t0, const int64_t t,
                              const int64_t t1) {
    return static_cast<double>(t - t0) / static_cast<double>(t1 - t0);
  }

  static Pose3D Interpolate(const int64_t t, const int64_t t0, const Pose3D& p0,
                            const int64_t t1, const Pose3D& p1) {
    const double w = TimeDiffRatio(t0, t, t1);
    const Eigen::Vector3d translation =
        p0.GetTranslation() + (p1.GetTranslation() - p0.GetTranslation()) * w;
    const Eigen::Quaterniond q = p0.GetQuaternion().normalized().slerp(
        w, p1.GetQuaternion().normalized());
    return Pose3D(translation, q);
  }

  static Pose3D Extrapolate(const int64_t t, const int64_t t0, const Pose3D& p0,
                            const int64_t t1, const Pose3D& p1) {
    const double w = TimeDiffRatio(t0, t, t1);
    const Eigen::Vector3d translation =
        p0.GetTranslation() + (p1.GetTranslation() - p0.GetTranslation()) * w;

    // Extrapolate rotation via AngleAxis to avoid SLERP's [0,1] constraint.
    const Eigen::Quaterniond q0 = p0.GetQuaternion().normalized();
    const Eigen::Quaterniond q1 = p1.GetQuaternion().normalized();
    const Eigen::AngleAxisd delta(q0.inverse() * q1);
    const Eigen::Quaterniond q = q0 * Eigen::Quaterniond(Eigen::AngleAxisd(
                                          delta.angle() * w, delta.axis()));

    return Pose3D(translation, q);
  }

  std::map<int64_t, Pose3D> timestamped_poses_;
};

}  // namespace mapping
}  // namespace adlabel
