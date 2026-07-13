#pragma once

#include <map>
#include <set>
#include <string>

#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "mapping/protos/frame.pb.h"

namespace adlabel {
namespace mapping {

class MatchingPairSearcher {
 public:
  void AddFrame(const Frame& frame);
  // void FindFramePairs();

 private:
  std::map<std::string, std::set<std::string>> trips_;
  std::map<std::string, Frame> frames_;

  bool has_enu_origin_ = false;
  GeographicLib::LocalCartesian local_cartesian_;
  std::map<std::string, Eigen::Vector3d> enu_coords_;
};

}  // namespace mapping
}  // namespace adlabel
