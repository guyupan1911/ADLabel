#include <glog/logging.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include "localization/gtsam_tutorials/trajectory_io.h"

using namespace gtsam;

int main(int argc, char**argv) {
  google::InitGoogleLogging(argv[0]);

  // construct nonlinear factor graph
  NonlinearFactorGraph graph;

  // add prior factor
  Pose2 prior_mean(0.0, 0.0, 0.0);
  auto prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, prior_mean, prior_noise);

  // add odometry factors
  Pose2 odometry(2.0, 0.0, 0.0);
  auto odometry_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, odometry, odometry_noise);
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, odometry, odometry_noise);
  graph.print("\nFactor Grapg\n");

  // initialize variables
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  
  initial.print("\nInitial Estimate\n");

  // optimize using LM
  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setVerbosityLM("SUMMARY");
  LOG(INFO) << "initial error = " << graph.error(initial);
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  LOG(INFO) << "final error = " << graph.error(result);
  result.print("\nFinal Result\n");

  // calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);

  const std::string trajectory_csv =
    "data/gtsam_results/odometry_trajectory.csv";
  adlabel::gtsam_tutorials::SavePose2ValuesCsv(
      trajectory_csv,
      {
          {"initial", &initial, nullptr},
          {"result", &result, &marginals},
      });
  LOG(INFO) << "Saved trajectory CSV: " << trajectory_csv;

  LOG(INFO) << "x1 covariance:\n" << marginals.marginalCovariance(1) << "\n";
  LOG(INFO) << "x2 covariance:\n" << marginals.marginalCovariance(2) << "\n";
  LOG(INFO) << "x3 covariance:\n" << marginals.marginalCovariance(3) << "\n";

  return 0;
}
