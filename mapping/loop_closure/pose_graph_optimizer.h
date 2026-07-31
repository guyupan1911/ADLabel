#pragma once

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "mapping/protos/frame.pb.h"

namespace adlabel {
namespace mapping {

struct LoopClosureMeasurement {
  // Relative pose that transforms points from from_frame into to_frame:
  // T_to_from = T_world_to.inverse() * T_world_from.
  std::string from_frame_id;
  std::string to_frame_id;
  Eigen::Affine3d relative_pose = Eigen::Affine3d::Identity();
};

struct PoseGraphResidualSummary {
  double mean_prior_translation_residual = 0.0;
  double mean_translation_residual = 0.0;
  double mean_rotation_residual = 0.0;
  double mean_loop_closure_residual = 0.0;
  int between_factor_count = 0;
  int loop_closure_factor_count = 0;
};

struct PoseGraphOptimizationResult {
  std::vector<Frame> optimized_frames;
  PoseGraphResidualSummary initial_residuals;
  PoseGraphResidualSummary optimized_residuals;
  int node_count = 0;
  int prior_factor_count = 0;
  int between_factor_count = 0;
  int loop_closure_factor_count = 0;
  double initial_graph_error = 0.0;
  double optimized_graph_error = 0.0;
};

class PoseGraphOptimizer {
 public:
  void AddFrame(const Frame& frame);

  bool Optimize(PoseGraphOptimizationResult* result);

  const std::map<std::string, gtsam::Key>& frame_id_to_key() const {
    return frame_id_to_key_;
  }

 private:
  static constexpr double kPriorTranslationSigmaM = 10000;
  static constexpr double kBetweenTranslationSigmaM = 0.1;
  static constexpr double kBetweenRotationSigmaRad = 0.01;
  static constexpr double kLoopClosureTranslationSigmaM = 0.1;
  static constexpr double kLoopClosureRotationSigmaRad = 0.01;
  static constexpr double kMinLoopClosureInlierRatio = 0.7;

  void BuildProblem();
  PoseGraphResidualSummary ComputeResidualSummary(
      const gtsam::Values& values) const;

  int between_factor_count_ = 0;
  int loop_closure_factor_count_ = 0;
  std::map<std::string, gtsam::Key> frame_id_to_key_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  std::unordered_map<std::string, std::set<std::string>> trips_;
  std::unordered_map<std::string, Frame> frames_;
};

}  // namespace mapping
}  // namespace adlabel
