#include "mapping/loop_closure/pose_graph_optimizer.h"

#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

#include "mapping/protos/pose.pb.h"

namespace adlabel {
namespace mapping {
namespace {

using gtsam::symbol_shorthand::X;

gtsam::Pose3 ToGtsamPose(const Pose3DMessage& pose) {
  Eigen::Quaterniond q(pose.qw(), pose.qx(), pose.qy(), pose.qz());
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(pose.x(), pose.y(), pose.z()));
}

void FillPoseMessage(const gtsam::Pose3& pose, Pose3DMessage* msg) {
  CHECK(msg != nullptr);
  const gtsam::Point3 t = pose.translation();
  const Eigen::Quaterniond q = pose.rotation().toQuaternion().normalized();
  msg->set_x(t.x());
  msg->set_y(t.y());
  msg->set_z(t.z());
  msg->set_qw(q.w());
  msg->set_qx(q.x());
  msg->set_qy(q.y());
  msg->set_qz(q.z());
}

double RotationErrorRad(const gtsam::Rot3& rotation) {
  const Eigen::Quaterniond q = rotation.toQuaternion().normalized();
  return Eigen::AngleAxisd(q).angle();
}

void AccumulatePoseResidual(const gtsam::Pose3& measured_relative,
                            const gtsam::Pose3& predicted_relative,
                            double* translation_residual_sum,
                            double* rotation_residual_sum) {
  CHECK(translation_residual_sum != nullptr);
  CHECK(rotation_residual_sum != nullptr);
  const gtsam::Pose3 error = measured_relative.inverse() * predicted_relative;
  *translation_residual_sum += error.translation().norm();
  *rotation_residual_sum += RotationErrorRad(error.rotation());
}

}  // namespace

void PoseGraphOptimizer::AddFrame(const Frame& frame) {
  if (!frame.has_refined_pose_3d() || !frame.has_fid() || frame.fid().empty()) {
    return;
  }

  frames_[frame.fid()] = frame;
  if (frame.has_trip_id() && !frame.trip_id().empty()) {
    trips_[frame.trip_id()].insert(frame.fid());
  }
}

void PoseGraphOptimizer::BuildProblem() {
  graph_ = gtsam::NonlinearFactorGraph();
  initial_ = gtsam::Values();
  frame_id_to_key_.clear();
  between_factor_count_ = 0;
  loop_closure_factor_count_ = 0;

  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector3() << kPriorTranslationSigmaM, kPriorTranslationSigmaM,
       kPriorTranslationSigmaM)
          .finished());
  const auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << kBetweenRotationSigmaRad, kBetweenRotationSigmaRad,
       kBetweenRotationSigmaRad, kBetweenTranslationSigmaM,
       kBetweenTranslationSigmaM, kBetweenTranslationSigmaM)
          .finished());
  const auto loop_closure_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << kLoopClosureRotationSigmaRad,
       kLoopClosureRotationSigmaRad, kLoopClosureRotationSigmaRad,
       kLoopClosureTranslationSigmaM, kLoopClosureTranslationSigmaM,
       kLoopClosureTranslationSigmaM)
          .finished());

  for (const auto& frame_id_and_frame : frames_) {
    const std::string& frame_id = frame_id_and_frame.first;
    const Frame& frame = frame_id_and_frame.second;
    const gtsam::Key key = X(frame_id_to_key_.size());
    const gtsam::Pose3 pose = ToGtsamPose(frame.refined_pose_3d());
    frame_id_to_key_[frame_id] = key;
    initial_.insert(key, pose);
    graph_.add(gtsam::GPSFactor(key, pose.translation(), prior_noise));
  }

  std::set<std::pair<std::string, std::string>> inserted_edges;
  const auto add_between_factor = [&](const std::string& from_id,
                                      const std::string& to_id) {
    if (from_id.empty() || to_id.empty() || from_id == to_id) {
      return;
    }
    if (!inserted_edges.insert({from_id, to_id}).second) {
      return;
    }

    const auto from_frame_iter = frames_.find(from_id);
    const auto to_frame_iter = frames_.find(to_id);
    const auto from_key_iter = frame_id_to_key_.find(from_id);
    const auto to_key_iter = frame_id_to_key_.find(to_id);
    if (from_frame_iter == frames_.end() || to_frame_iter == frames_.end() ||
        from_key_iter == frame_id_to_key_.end() ||
        to_key_iter == frame_id_to_key_.end()) {
      return;
    }

    const gtsam::Pose3 from_pose =
        ToGtsamPose(from_frame_iter->second.refined_pose_3d());
    const gtsam::Pose3 to_pose =
        ToGtsamPose(to_frame_iter->second.refined_pose_3d());
    const gtsam::Pose3 relative_pose = from_pose.between(to_pose);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from_key_iter->second, to_key_iter->second, relative_pose,
        between_noise));
    ++between_factor_count_;
  };

  for (const auto& trip_and_frame_ids : trips_) {
    const std::set<std::string>& frame_ids = trip_and_frame_ids.second;
    for (const std::string& frame_id : frame_ids) {
      const Frame& frame = frames_.at(frame_id);
      if (frame.has_next_id() && frame_ids.count(frame.next_id()) > 0) {
        add_between_factor(frame_id, frame.next_id());
      }
      if (frame.has_prev_id() && frame_ids.count(frame.prev_id()) > 0) {
        add_between_factor(frame.prev_id(), frame_id);
      }
    }
  }

  for (const auto& frame_id_and_frame : frames_) {
    const std::string& from_id = frame_id_and_frame.first;
    const Frame& frame = frame_id_and_frame.second;
    const auto from_key_iter = frame_id_to_key_.find(from_id);
    if (from_key_iter == frame_id_to_key_.end()) {
      continue;
    }

    for (const MatchedFrame& matched_frame : frame.matched_frames()) {
      if (matched_frame.inlier_ratio() < kMinLoopClosureInlierRatio) {
        continue;
      }
      if (!matched_frame.has_frame_id() || matched_frame.frame_id().empty() ||
          matched_frame.frame_id() == from_id ||
          !matched_frame.has_relative_pose()) {
        continue;
      }
      const auto to_key_iter = frame_id_to_key_.find(matched_frame.frame_id());
      if (to_key_iter == frame_id_to_key_.end()) {
        continue;
      }

      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          to_key_iter->second, from_key_iter->second,
          ToGtsamPose(matched_frame.relative_pose()), loop_closure_noise));
      ++loop_closure_factor_count_;
    }
  }
}

PoseGraphResidualSummary PoseGraphOptimizer::ComputeResidualSummary(
    const gtsam::Values& values) const {
  PoseGraphResidualSummary summary;
  double prior_translation_residual_sum = 0.0;
  double translation_residual_sum = 0.0;
  double rotation_residual_sum = 0.0;
  double loop_translation_residual_sum = 0.0;

  for (const auto& frame_id_and_frame : frames_) {
    const std::string& frame_id = frame_id_and_frame.first;
    const Frame& frame = frame_id_and_frame.second;
    const auto key_iter = frame_id_to_key_.find(frame_id);
    if (key_iter == frame_id_to_key_.end()) {
      continue;
    }

    const gtsam::Point3 predicted_translation =
        values.at<gtsam::Pose3>(key_iter->second).translation();
    const Eigen::Vector3d measured_translation(frame.refined_pose_3d().x(),
                                               frame.refined_pose_3d().y(),
                                               frame.refined_pose_3d().z());
    prior_translation_residual_sum +=
        (Eigen::Vector3d(predicted_translation.x(), predicted_translation.y(),
                         predicted_translation.z()) -
         measured_translation)
            .norm();
  }
  if (!frames_.empty()) {
    summary.mean_prior_translation_residual =
        prior_translation_residual_sum / frames_.size();
  }
  std::set<std::pair<std::string, std::string>> inserted_edges;
  const auto accumulate_between_factor = [&](const std::string& from_id,
                                             const std::string& to_id) {
    if (from_id.empty() || to_id.empty() || from_id == to_id) {
      return;
    }
    if (!inserted_edges.insert({from_id, to_id}).second) {
      return;
    }

    const auto from_frame_iter = frames_.find(from_id);
    const auto to_frame_iter = frames_.find(to_id);
    const auto from_key_iter = frame_id_to_key_.find(from_id);
    const auto to_key_iter = frame_id_to_key_.find(to_id);
    if (from_frame_iter == frames_.end() || to_frame_iter == frames_.end() ||
        from_key_iter == frame_id_to_key_.end() ||
        to_key_iter == frame_id_to_key_.end()) {
      return;
    }

    const gtsam::Pose3 measured_relative =
        ToGtsamPose(from_frame_iter->second.refined_pose_3d())
            .between(ToGtsamPose(to_frame_iter->second.refined_pose_3d()));
    const gtsam::Pose3 predicted_relative =
        values.at<gtsam::Pose3>(from_key_iter->second)
            .between(values.at<gtsam::Pose3>(to_key_iter->second));
    AccumulatePoseResidual(measured_relative, predicted_relative,
                           &translation_residual_sum, &rotation_residual_sum);
    ++summary.between_factor_count;
  };

  for (const auto& trip_and_frame_ids : trips_) {
    const std::set<std::string>& frame_ids = trip_and_frame_ids.second;
    for (const std::string& frame_id : frame_ids) {
      const Frame& frame = frames_.at(frame_id);
      if (frame.has_next_id() && frame_ids.count(frame.next_id()) > 0) {
        accumulate_between_factor(frame_id, frame.next_id());
      }
      if (frame.has_prev_id() && frame_ids.count(frame.prev_id()) > 0) {
        accumulate_between_factor(frame.prev_id(), frame_id);
      }
    }
  }

  for (const auto& frame_id_and_frame : frames_) {
    const std::string& from_id = frame_id_and_frame.first;
    const Frame& frame = frame_id_and_frame.second;
    const auto from_key_iter = frame_id_to_key_.find(from_id);
    if (from_key_iter == frame_id_to_key_.end()) {
      continue;
    }

    for (const MatchedFrame& matched_frame : frame.matched_frames()) {
      if (matched_frame.inlier_ratio() < kMinLoopClosureInlierRatio) {
        continue;
      }
      if (!matched_frame.has_frame_id() || matched_frame.frame_id().empty() ||
          matched_frame.frame_id() == from_id ||
          !matched_frame.has_relative_pose()) {
        continue;
      }
      const auto to_key_iter = frame_id_to_key_.find(matched_frame.frame_id());
      if (to_key_iter == frame_id_to_key_.end()) {
        continue;
      }

      const gtsam::Pose3 measured_relative =
          ToGtsamPose(matched_frame.relative_pose());
      const gtsam::Pose3 predicted_relative =
          values.at<gtsam::Pose3>(to_key_iter->second)
              .between(values.at<gtsam::Pose3>(from_key_iter->second));
      double loop_translation_residual = 0.0;
      double loop_rotation_residual = 0.0;
      AccumulatePoseResidual(measured_relative, predicted_relative,
                             &loop_translation_residual,
                             &loop_rotation_residual);
      translation_residual_sum += loop_translation_residual;
      rotation_residual_sum += loop_rotation_residual;
      loop_translation_residual_sum += loop_translation_residual;
      ++summary.loop_closure_factor_count;
    }
  }

  const int factor_count =
      summary.between_factor_count + summary.loop_closure_factor_count;
  if (factor_count > 0) {
    summary.mean_translation_residual = translation_residual_sum / factor_count;
    summary.mean_rotation_residual = rotation_residual_sum / factor_count;
  }
  if (summary.loop_closure_factor_count > 0) {
    summary.mean_loop_closure_residual =
        loop_translation_residual_sum / summary.loop_closure_factor_count;
  }
  return summary;
}

bool PoseGraphOptimizer::Optimize(PoseGraphOptimizationResult* result) {
  CHECK(result != nullptr);
  *result = PoseGraphOptimizationResult();
  result->optimized_frames.reserve(frames_.size());
  for (const auto& frame_id_and_frame : frames_) {
    result->optimized_frames.push_back(frame_id_and_frame.second);
  }

  BuildProblem();

  if (frame_id_to_key_.size() < 2) {
    return false;
  }

  result->node_count = static_cast<int>(frame_id_to_key_.size());
  result->prior_factor_count = result->node_count;
  result->between_factor_count = between_factor_count_;
  result->loop_closure_factor_count = loop_closure_factor_count_;
  result->initial_residuals = ComputeResidualSummary(initial_);
  result->initial_graph_error = graph_.error(initial_);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_);
  const gtsam::Values optimized = optimizer.optimize();

  result->optimized_residuals = ComputeResidualSummary(optimized);
  result->optimized_graph_error = graph_.error(optimized);

  for (Frame& frame : result->optimized_frames) {
    const auto key_iter = frame_id_to_key_.find(frame.fid());
    if (key_iter == frame_id_to_key_.end()) {
      continue;
    }
    FillPoseMessage(optimized.at<gtsam::Pose3>(key_iter->second),
                    frame.mutable_refined_pose_3d());
  }

  return true;
}

}  // namespace mapping
}  // namespace adlabel
