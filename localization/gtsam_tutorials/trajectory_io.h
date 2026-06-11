#pragma once

#include <string>
#include <utility>
#include <vector>

#include <gtsam/nonlinear/Values.h>

namespace gtsam {
class Marginals;
}  // namespace gtsam

namespace adlabel {
namespace gtsam_tutorials {

struct Pose2ValuesCsvEntry {
  std::string name;
  const gtsam::Values* values = nullptr;
  const gtsam::Marginals* marginals = nullptr;
};

void SavePose2ValuesCsv(const std::string& path,
                        const std::vector<Pose2ValuesCsvEntry>& trajectories);

}  // namespace gtsam_tutorials
}  // namespace adlabel
