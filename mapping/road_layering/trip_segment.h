#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace adlabel {
namespace mapping {

struct BoundingBox2D {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
};

struct TripSegment {
  std::string id;
  std::string trip_id;
  std::vector<std::string> frame_ids;
  std::vector<Eigen::Affine3d> frame_poses;  // local enu

  BoundingBox2D bbox;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  double dz = 0.0;
  double ds = 0.0;
  double grade = 0.0;
  bool is_ramp = false;
};

}  // namespace mapping
}  // namespace adlabel
