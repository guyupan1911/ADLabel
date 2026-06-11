#include "localization/gtsam_tutorials/trajectory_io.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>

namespace adlabel {
namespace gtsam_tutorials {
namespace {

gtsam::KeyVector SortedKeys(const gtsam::Values& values) {
  gtsam::KeyVector keys = values.keys();
  std::sort(keys.begin(), keys.end());
  return keys;
}

void EnsureDirectory(const std::string& dir) {
  if (dir.empty()) {
    return;
  }

  std::string current;
  if (dir.front() == '/') {
    current = "/";
  }

  size_t start = dir.front() == '/' ? 1 : 0;
  while (start <= dir.size()) {
    const size_t end = dir.find('/', start);
    const std::string part =
        dir.substr(start, end == std::string::npos ? std::string::npos : end - start);
    if (!part.empty()) {
      if (current.size() > 1 && current.back() != '/') {
        current += '/';
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error("Failed to create directory " + current + ": " +
                                 std::strerror(errno));
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
}

void EnsureParentDirectory(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return;
  }
  EnsureDirectory(path.substr(0, slash));
}

}  // namespace

void SavePose2ValuesCsv(const std::string& path,
                        const std::vector<Pose2ValuesCsvEntry>& trajectories) {
  EnsureParentDirectory(path);

  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Failed to open trajectory CSV for writing: " + path);
  }

  ofs << "name,key,x,y,z,theta,qx,qy,qz,qw,cov_xx,cov_xy,cov_yy,cov_tt\n";
  for (const auto& trajectory : trajectories) {
    if (trajectory.values == nullptr) {
      continue;
    }
    for (const gtsam::Key key : SortedKeys(*trajectory.values)) {
      const auto& pose = trajectory.values->at<gtsam::Pose2>(key);
      ofs << trajectory.name << ',' << key << ',' << pose.x() << ',' << pose.y()
          << ",0," << pose.theta() << ",,,,";
      if (trajectory.marginals != nullptr) {
        const auto covariance = trajectory.marginals->marginalCovariance(key);
        ofs << ',' << covariance(0, 0) << ',' << covariance(0, 1) << ','
            << covariance(1, 1) << ',' << covariance(2, 2);
      } else {
        ofs << ",,,,";
      }
      ofs << '\n';
    }
  }
}

}  // namespace gtsam_tutorials
}  // namespace adlabel
