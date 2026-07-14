#include "mapping/loop_closure/matching_pair_searcher.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"

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
}

void MatchingPairSearcher::FindFramePairs(std::vector<FramePair>* frame_pairs) {
  std::vector<FrameIdPair> frame_id_pairs;
  for (auto iter_trip_from = trips_.begin();
       iter_trip_from != trips_.end(); ++iter_trip_from) {
    for (auto iter_trip_to = std::next(iter_trip_from);
         iter_trip_to != trips_.end(); ++iter_trip_to) {
      auto& trip_from_frames = iter_trip_from->second;
      auto& trip_to_frames = iter_trip_to->second;
      FindFramePairsBetweenTwoTrips(
        trip_from_frames, trip_to_frames, &frame_id_pairs);
    }
  }

  GenerateFramePairs(frame_id_pairs, frame_pairs);

  LOG(INFO) << "frame_id_pairs size: " << frame_id_pairs.size();
}

void MatchingPairSearcher::FindFramePairsBetweenTwoTrips(
    const std::set<std::string>& trip_from_frames,
    const std::set<std::string>& trip_to_frames,
    std::vector<FrameIdPair>* frame_pairs) {

  std::vector<std::pair<double, FrameIdPair>> pairs_with_distance;

  double previous_matching_cumulative_dis =
        std::numeric_limits<double>::lowest();
  for (const auto& frame_from : trip_from_frames) {
    if (frames_.at(frame_from).cumulative_distance() -
      previous_matching_cumulative_dis < 5.0) {
      continue;
    }

    double min_distance_xy = std::numeric_limits<double>::max();
    std::string min_distance_frame_to_id;
    for (const auto& frame_to : trip_to_frames) {
      double distance_xy =
        (enu_coords_.at(frame_from).head<2>() -
          enu_coords_.at(frame_to).head<2>()).norm();
      if (distance_xy < min_distance_xy) {
        min_distance_xy = distance_xy;
        min_distance_frame_to_id = frame_to;
      }
    }
    if (min_distance_xy < 40.0 && !min_distance_frame_to_id.empty()) {
      pairs_with_distance.emplace_back(
        min_distance_xy, std::make_pair(frame_from, min_distance_frame_to_id));
      previous_matching_cumulative_dis =
        frames_.at(frame_from).cumulative_distance();
    }
  }

  // remove duplicated frame pairs
  std::vector<FrameIdPair> exist_frame_pairs;
  std::sort(pairs_with_distance.begin(), pairs_with_distance.end(),
            [](const auto& pair1, const auto& pair2) {
              return pair1.first < pair2.first;
            });
  for (const auto& frame_pair : pairs_with_distance) {
    const auto& frame_1 = frame_pair.second.first;
    const auto& frame_2 = frame_pair.second.second;

    bool is_duplicated = false;
    for (const auto& frame_pair_exist : exist_frame_pairs) {
      const auto& frame_1_exist = frame_pair_exist.first;
      const auto& frame_2_exist = frame_pair_exist.second;
      double cumulative_distance_1 =
        std::abs(frames_.at(frame_1).cumulative_distance() -
                  frames_.at(frame_1_exist).cumulative_distance());
      double cumulative_distance_2 =
        std::abs(frames_.at(frame_2).cumulative_distance() -
                  frames_.at(frame_2_exist).cumulative_distance());
      if (cumulative_distance_1 < 20.0 || cumulative_distance_2 < 20.0) {
        is_duplicated = true;
        break;
      }
    }
    if (!is_duplicated) {
      exist_frame_pairs.push_back(frame_pair.second);
      frame_pairs->push_back(frame_pair.second);
    }
  }
}

void MatchingPairSearcher::GenerateFramePairs(
    const std::vector<FrameIdPair>& frame_id_pairs,
    std::vector<FramePair>* frame_pairs) {
  
  for (const auto& frame_id_pair : frame_id_pairs) {
    FramePair frame_pair;

    // collect from local
    frame_pair.mutable_from_frame()->CopyFrom(frames_.at(frame_id_pair.first));
    std::vector<Frame> local_frames;
    std::vector<Pose3DMessage> local_relative_poses;
    CollectLocalFrames(frame_id_pair.first, &local_frames, &local_relative_poses);
    CHECK(local_frames.size() == local_relative_poses.size());
    // LOG(INFO) << "from local_frame size: " << local_frames.size();
    for (size_t i = 0; i < local_frames.size(); ++i) {
      frame_pair.add_from_local_frames()->CopyFrom(local_frames[i]);
      frame_pair.add_from_relative_poses()->CopyFrom(local_relative_poses[i]);
    }

    // collect to local
    frame_pair.mutable_to_frame()->CopyFrom(frames_.at(frame_id_pair.second));
    local_frames.clear();
    local_relative_poses.clear();
    CollectLocalFrames(frame_id_pair.second, &local_frames, &local_relative_poses);
    CHECK(local_frames.size() == local_relative_poses.size());
    // LOG(INFO) << "to local_frame size: " << local_frames.size();
    for (size_t i = 0; i < local_frames.size(); ++i) {
      frame_pair.add_to_local_frames()->CopyFrom(local_frames[i]);
      frame_pair.add_to_relative_poses()->CopyFrom(local_relative_poses[i]);
    }

    frame_pairs->push_back(frame_pair);
  }
}

void MatchingPairSearcher::CollectLocalFrames(
    const std::string& reference_frame_id,
    std::vector<Frame>* local_frames,
    std::vector<Pose3DMessage>* local_relative_poses) {
  const auto& reference_frame = frames_.at(reference_frame_id);
  Pose3D reference_pose(reference_frame.refined_pose_3d());

  // collect previous
  auto iter = frames_.find(reference_frame.prev_id());
  Eigen::Vector3d prev_coords = enu_coords_.at(reference_frame.fid());
  while (iter != frames_.end()) {
    double diff_cumu_dis =
      std::abs(iter->second.cumulative_distance() - reference_frame.cumulative_distance());
    if (diff_cumu_dis > 10.0) {
      break;
    }
    
    double distance = (enu_coords_.at(iter->second.fid()) - prev_coords).norm();
    if (distance > 1.0) {
      local_frames->push_back(iter->second);
      prev_coords = enu_coords_.at(iter->second.fid());
      Pose3D current_pose = Pose3D(iter->second.refined_pose_3d());
      Eigen::Affine3d T_reference_current = 
        reference_pose.GetAffine3D().inverse() * current_pose.GetAffine3D();
      local_relative_poses->push_back(
        Pose3D(T_reference_current).GetPose3DMessage());
    }
    iter = frames_.find(iter->second.prev_id());
  }

  // collect next
  iter = frames_.find(reference_frame.next_id());
  prev_coords = enu_coords_.at(reference_frame.fid());
  while (iter != frames_.end()) {
    double diff_cumu_dis =
      std::abs(iter->second.cumulative_distance() - reference_frame.cumulative_distance());
    if (diff_cumu_dis > 10.0) {
      break;
    }
    
    double distance = (enu_coords_.at(iter->second.fid()) - prev_coords).norm();
    if (distance > 1.0) {
      local_frames->push_back(iter->second);
      prev_coords = enu_coords_.at(iter->second.fid());
      Pose3D current_pose = Pose3D(iter->second.refined_pose_3d());
      Eigen::Affine3d T_reference_current = 
        reference_pose.GetAffine3D().inverse() * current_pose.GetAffine3D();
      local_relative_poses->push_back(
        Pose3D(T_reference_current).GetPose3DMessage());
    }
    iter = frames_.find(iter->second.next_id());
  }
}

}  // namespace mapping
}  // namespace adlabel