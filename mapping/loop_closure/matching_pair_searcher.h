#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "mapping/protos/frame.pb.h"
#include "mapping/protos/frame_pair.pb.h"

namespace adlabel {
namespace mapping {

class MatchingPairSearcher {
 public:
  using FrameIdPair = std::pair<std::string, std::string>;

  void AddFrame(const Frame& frame);
  void FindFramePairs(std::vector<FramePair>* frame_pairs);

 private:
  void FindFramePairsBetweenTwoTrips(
    const std::set<std::string>& trip_from_frames,
    const std::set<std::string>& trip_to_frames,
    std::vector<FrameIdPair>* frame_pairs);

  void GenerateFramePairs(
    const std::vector<FrameIdPair>& frame_id_pairs,
    std::vector<FramePair>* frame_pairs);

  void CollectLocalFrames(
    const std::string& reference_frame_id,
    std::vector<Frame>* local_frames,
    std::vector<Pose3DMessage>* local_relative_poses);

  std::map<std::string, std::set<std::string>> trips_;
  std::map<std::string, Frame> frames_;

  bool has_enu_origin_ = false;
  GeographicLib::LocalCartesian local_cartesian_;
  std::map<std::string, Eigen::Vector3d> enu_coords_;
};

}  // namespace mapping
}  // namespace adlabel
