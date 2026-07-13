#include "mapping/loop_closure/matching_pair_searcher.h"

#include <glog/logging.h>

namespace adlabel {
namespace mapping {

void MatchingPairSearcher::AddFrame(const Frame& frame) {
  const auto& trip_id = frame.trip_id();
  const auto& fid = frame.fid();
  // LOG(INFO) << "trip id: " << trip_id;
  // LOG(INFO) << "fid: " << fid;

  if (!frame.has_refined_pose_3d()) {
    LOG(INFO) << "skip frame wo refined_pose_3d";
    return;
  }

  const auto& pose_ecef = frame.refined_pose_3d();
  double lat = 0.0;
  double lon = 0.0;
  double height = 0.0;
  GeographicLib::Geocentric::WGS84().Reverse(
      pose_ecef.x(), pose_ecef.y(), pose_ecef.z(), lat, lon, height);

  if (!has_enu_origin_) {
    local_cartesian_.Reset(lat, lon, height);
    has_enu_origin_ = true;
  }

  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  local_cartesian_.Forward(lat, lon, height, east, north, up);

  trips_[trip_id].emplace(fid);
  frames_[fid] = frame;
  enu_coords_[fid] = Eigen::Vector3d(east, north, up);
  LOG(INFO) << "enu: " << enu_coords_[fid];
}

}  // namespace mapping
}  // namespace adlabel