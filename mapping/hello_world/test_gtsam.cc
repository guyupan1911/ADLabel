#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

DEFINE_string(output_dir, "/tmp", "Directory to save output files");

// Estimate a simple 2-pose trajectory from odometry measurements.
//
// Ground truth:
//   x0 = identity
//   x1 = 1m forward along X axis
//
// We add a prior on x0 and a between factor x0->x1, then optimize.
int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  using gtsam::symbol_shorthand::X;

  // --- Build factor graph ---
  gtsam::NonlinearFactorGraph graph;

  // Prior on x0: identity pose, tight noise
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  graph.addPrior(X(0), gtsam::Pose3(), prior_noise);

  // Odometry x0 -> x1: 1m forward, small noise
  auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  gtsam::Pose3 odom(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(0), X(1), odom, odom_noise));

  LOG(INFO) << "Factor graph:";
  LOG(INFO) << "  factors = " << graph.size();

  // --- Initial values (add some noise to make optimization non-trivial) ---
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());
  initial.insert(X(1),
                 gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.9, 0.1, 0.0)));

  LOG(INFO) << "Initial estimate x1 translation: "
            << initial.at<gtsam::Pose3>(X(1)).translation().transpose();

  // --- Optimize ---
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
  gtsam::Values result = optimizer.optimize();

  gtsam::Pose3 x0 = result.at<gtsam::Pose3>(X(0));
  gtsam::Pose3 x1 = result.at<gtsam::Pose3>(X(1));

  LOG(INFO) << "Optimized x0 translation: " << x0.translation().transpose();
  LOG(INFO) << "Optimized x1 translation: " << x1.translation().transpose();
  LOG(INFO) << "Final error: " << graph.error(result);

  // Verify x1 is close to (1, 0, 0)
  gtsam::Point3 t1 = x1.translation();
  CHECK_NEAR(t1.x(), 1.0, 1e-3);
  CHECK_NEAR(t1.y(), 0.0, 1e-3);
  CHECK_NEAR(t1.z(), 0.0, 1e-3);

  LOG(INFO) << "GTSAM hello world PASSED";
  return 0;
}
