#include "mapping/road_layering/road_layering.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <glog/logging.h>

#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/geographic_transform.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr double kSegmentLengthMeters = 10.0;
constexpr double kRampGradeThreshold = 0.03;
constexpr double kSegmentSearchRadiusMeters = 6.0;
constexpr double kSameLevelMaxZDifferenceMeters = 0.8;
constexpr double kCannotLinkMaxXyDistanceMeters = 3.0;
constexpr double kCannotLinkMinZDifferenceMeters = 2.0;
constexpr std::size_t kMinMatchedPoseCount = 2;

struct SameLevelEdge {
  int first = 0;
  int second = 0;
  double score = 0.0;
};

std::uint64_t MakeNodePairKey(int first, int second) {
  const std::uint32_t smaller =
      static_cast<std::uint32_t>(std::min(first, second));
  const std::uint32_t larger =
      static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(smaller) << 32) | larger;
}

std::uint64_t MakeDirectedPairKey(int from, int to) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(from)) << 32) |
         static_cast<std::uint32_t>(to);
}

class UnionFind {
 public:
  explicit UnionFind(std::size_t size) : parent_(size), cluster_members_(size) {
    std::iota(parent_.begin(), parent_.end(), 0);
    for (std::size_t i = 0; i < size; ++i) {
      cluster_members_[i].push_back(static_cast<int>(i));
    }
  }

  int Find(int node) {
    if (parent_[node] != node) {
      parent_[node] = Find(parent_[node]);
    }
    return parent_[node];
  }

  int Union(int first, int second) {
    int first_root = Find(first);
    int second_root = Find(second);
    if (first_root == second_root) {
      return first_root;
    }

    if (cluster_members_[first_root].size() <
        cluster_members_[second_root].size()) {
      std::swap(first_root, second_root);
    }
    parent_[second_root] = first_root;
    cluster_members_[first_root].insert(cluster_members_[first_root].end(),
                                        cluster_members_[second_root].begin(),
                                        cluster_members_[second_root].end());
    cluster_members_[second_root].clear();
    return first_root;
  }

  const std::vector<int>& Members(int root) {
    return cluster_members_[Find(root)];
  }

 private:
  std::vector<int> parent_;
  std::vector<std::vector<int>> cluster_members_;
};

bool HasCannotLink(UnionFind& union_find, int first_root, int second_root,
                   const std::unordered_set<std::uint64_t>& cannot_link_pairs) {
  for (int first_node : union_find.Members(first_root)) {
    for (int second_node : union_find.Members(second_root)) {
      if (cannot_link_pairs.count(MakeNodePairKey(first_node, second_node)) >
          0) {
        return true;
      }
    }
  }
  return false;
}

bool ExpandedBoundingBoxesOverlap(const BoundingBox2D& first,
                                  const BoundingBox2D& second,
                                  double expansion) {
  return first.min_x - expansion <= second.max_x &&
         first.max_x + expansion >= second.min_x &&
         first.min_y - expansion <= second.max_y &&
         first.max_y + expansion >= second.min_y;
}

double Median(std::vector<double> values) {
  CHECK(!values.empty());
  const std::size_t middle_index = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle_index, values.end());
  const double upper_middle = values[middle_index];
  if (values.size() % 2 != 0) {
    return upper_middle;
  }

  const double lower_middle =
      *std::max_element(values.begin(), values.begin() + middle_index);
  return 0.5 * (lower_middle + upper_middle);
}

const char* LayerRelationName(LayerRelation relation) {
  switch (relation) {
    case LayerRelation::kSameLevel:
      return "SAME_LEVEL";
    case LayerRelation::kCannotLink:
      return "CANNOT_LINK";
    case LayerRelation::kNoRelation:
      return "NO_RELATION";
  }
  return "UNKNOWN";
}

}  // namespace

void RoadLayering::AddFrame(const Frame& frame) {
  if (!frame.has_refined_pose_3d()) {
    return;
  }

  const auto& trip_id = frame.trip_id();
  const auto& fid = frame.fid();

  if (!has_enu_origin_) {
    const Eigen::Vector3d origin_ecef =
        Pose3D(frame.refined_pose_3d()).GetTranslation();
    T_enu_ecef_ = MakeEcefToEnuTransform(origin_ecef);
    has_enu_origin_ = true;
  }
  frames_by_fid_[fid] = frame;
  frame_ids_by_trip_[trip_id].push_back(fid);
}

void RoadLayering::Run() {
  for (const auto& [trip_id, frame_ids] : frame_ids_by_trip_) {
    LOG(INFO) << "trip_id: " << trip_id << " frames size: " << frame_ids.size();
    SegmentTripFrames(frame_ids);
  }
  BuildSegmentRelations();
  ClusterRoadSurfaces();
}

void RoadLayering::SegmentTripFrames(
    const std::vector<std::string>& frame_ids) {
  if (frame_ids.empty()) {
    return;
  }

  CHECK(has_enu_origin_);

  std::vector<std::string> sorted_frame_ids = frame_ids;
  std::sort(sorted_frame_ids.begin(), sorted_frame_ids.end());

  const Frame& first_frame = frames_by_fid_.at(sorted_frame_ids.front());
  CHECK(first_frame.has_cumulative_distance())
      << "frame missing cumulative_distance: " << first_frame.fid();
  const std::string& trip_id = first_frame.trip_id();
  const double trip_start_distance = first_frame.cumulative_distance();

  std::vector<TripSegment>& segments = trip_segments_[trip_id];
  segments.clear();
  std::size_t current_distance_bin = 0;
  bool has_current_segment = false;

  for (const std::string& frame_id : sorted_frame_ids) {
    const Frame& frame = frames_by_fid_.at(frame_id);
    CHECK(frame.has_cumulative_distance())
        << "frame missing cumulative_distance: " << frame.fid();

    const double distance_from_start =
        frame.cumulative_distance() - trip_start_distance;
    const std::size_t distance_bin = static_cast<std::size_t>(
        std::max(0.0, std::floor(distance_from_start / kSegmentLengthMeters)));
    if (!has_current_segment || distance_bin != current_distance_bin) {
      TripSegment segment;
      segment.id = trip_id + "_" + std::to_string(segments.size());
      segment.trip_id = trip_id;
      segments.push_back(std::move(segment));
      current_distance_bin = distance_bin;
      has_current_segment = true;
    }

    const Eigen::Affine3d pose_ecef =
        Pose3D(frame.refined_pose_3d()).GetAffine3D();
    const Eigen::Affine3d pose_enu = T_enu_ecef_ * pose_ecef;

    TripSegment& segment = segments.back();
    segment.frame_ids.push_back(frame_id);
    segment.frame_poses.push_back(pose_enu);
  }

  for (TripSegment& segment : segments) {
    if (segment.frame_poses.empty()) {
      continue;
    }

    segment.bbox.min_x = std::numeric_limits<double>::max();
    segment.bbox.max_x = std::numeric_limits<double>::lowest();
    segment.bbox.min_y = std::numeric_limits<double>::max();
    segment.bbox.max_y = std::numeric_limits<double>::lowest();
    segment.center.setZero();
    for (const Eigen::Affine3d& pose : segment.frame_poses) {
      const Eigen::Vector3d position = pose.translation();
      segment.center += position;
      segment.bbox.min_x = std::min(segment.bbox.min_x, position.x());
      segment.bbox.max_x = std::max(segment.bbox.max_x, position.x());
      segment.bbox.min_y = std::min(segment.bbox.min_y, position.y());
      segment.bbox.max_y = std::max(segment.bbox.max_y, position.y());
    }
    segment.center /= static_cast<double>(segment.frame_poses.size());

    segment.dz = segment.frame_poses.back().translation().z() -
                 segment.frame_poses.front().translation().z();

    const Frame& segment_first_frame =
        frames_by_fid_.at(segment.frame_ids.front());
    const Frame& segment_last_frame =
        frames_by_fid_.at(segment.frame_ids.back());
    segment.ds = segment_last_frame.cumulative_distance() -
                 segment_first_frame.cumulative_distance();
    segment.grade = segment.ds > 0.0 ? segment.dz / segment.ds : 0.0;
    segment.is_ramp = std::abs(segment.grade) >= kRampGradeThreshold;

    // LOG(INFO) << "trip segment: " << segment.id << ", dz: " << segment.dz
    //           << ", ds: " << segment.ds << ", grade: " << segment.grade
    //           << ", is_ramp: " << segment.is_ramp;
  }

  LOG(INFO) << " add " << segments.size() << " trip_segments";
}

SegmentRelation RoadLayering::EstimateLayerRelation(
    const TripSegment& first, const TripSegment& second) const {
  SegmentRelation result;
  result.first_segment_id = first.id;
  result.second_segment_id = second.id;
  result.min_xy_distance = std::numeric_limits<double>::max();
  result.median_z_difference = std::numeric_limits<double>::max();

  std::vector<double> signed_z_differences;
  for (const Eigen::Affine3d& first_pose : first.frame_poses) {
    double nearest_xy_distance = std::numeric_limits<double>::max();
    double nearest_signed_z_difference = 0.0;

    for (const Eigen::Affine3d& second_pose : second.frame_poses) {
      const Eigen::Vector3d difference =
          first_pose.translation() - second_pose.translation();
      const double xy_distance = std::hypot(difference.x(), difference.y());
      if (xy_distance < nearest_xy_distance) {
        nearest_xy_distance = xy_distance;
        nearest_signed_z_difference = -difference.z();
      }
    }

    result.min_xy_distance =
        std::min(result.min_xy_distance, nearest_xy_distance);
    if (nearest_xy_distance <= kSegmentSearchRadiusMeters) {
      signed_z_differences.push_back(nearest_signed_z_difference);
    }
  }

  result.matched_pose_count = signed_z_differences.size();
  if (result.matched_pose_count < kMinMatchedPoseCount) {
    return result;
  }

  result.signed_z_difference = Median(std::move(signed_z_differences));
  result.median_z_difference = std::abs(result.signed_z_difference);
  if (result.min_xy_distance <= kSegmentSearchRadiusMeters &&
      result.median_z_difference <= kSameLevelMaxZDifferenceMeters) {
    result.relation = LayerRelation::kSameLevel;
    const double xy_score =
        1.0 - result.min_xy_distance / kSegmentSearchRadiusMeters;
    const double z_score =
        1.0 - result.median_z_difference / kSameLevelMaxZDifferenceMeters;
    result.score = 0.3 * xy_score + 0.7 * z_score;
  } else if (result.min_xy_distance <= kCannotLinkMaxXyDistanceMeters &&
             result.median_z_difference >= kCannotLinkMinZDifferenceMeters) {
    result.relation = LayerRelation::kCannotLink;
  }
  return result;
}

void RoadLayering::BuildSegmentRelations() {
  segment_relations_.clear();

  std::vector<const TripSegment*> level_segments;
  for (const auto& trip_and_segments : trip_segments_) {
    for (const TripSegment& segment : trip_and_segments.second) {
      if (!segment.is_ramp) {
        level_segments.push_back(&segment);
      }
    }
  }

  std::size_t same_level_count = 0;
  std::size_t cannot_link_count = 0;
  for (std::size_t first_index = 0; first_index < level_segments.size();
       ++first_index) {
    for (std::size_t second_index = first_index + 1;
         second_index < level_segments.size(); ++second_index) {
      const TripSegment& first = *level_segments[first_index];
      const TripSegment& second = *level_segments[second_index];
      if (first.trip_id == second.trip_id) {
        continue;
      }
      if (!ExpandedBoundingBoxesOverlap(first.bbox, second.bbox,
                                        kSegmentSearchRadiusMeters)) {
        continue;
      }

      SegmentRelation relation = EstimateLayerRelation(first, second);
      if (relation.relation == LayerRelation::kSameLevel) {
        ++same_level_count;
      } else if (relation.relation == LayerRelation::kCannotLink) {
        ++cannot_link_count;
      }

      LOG(INFO) << "segment pair: " << relation.first_segment_id << " <-> "
                << relation.second_segment_id
                << ", min_xy: " << relation.min_xy_distance
                << ", signed_dz: " << relation.signed_z_difference
                << ", median_dz: " << relation.median_z_difference
                << ", matched_poses: " << relation.matched_pose_count
                << ", score: " << relation.score
                << ", relation: " << LayerRelationName(relation.relation);
      segment_relations_.push_back(std::move(relation));
    }
  }

  LOG(INFO) << "segment relation summary: level_segments="
            << level_segments.size()
            << ", candidate_pairs=" << segment_relations_.size()
            << ", same_level=" << same_level_count
            << ", cannot_link=" << cannot_link_count;
}

void RoadLayering::ClusterRoadSurfaces() {
  road_surfaces_.clear();

  std::vector<const TripSegment*> level_segments;
  for (const auto& trip_and_segments : trip_segments_) {
    for (const TripSegment& segment : trip_and_segments.second) {
      if (!segment.is_ramp) {
        level_segments.push_back(&segment);
      }
    }
  }
  std::sort(level_segments.begin(), level_segments.end(),
            [](const TripSegment* first, const TripSegment* second) {
              return first->id < second->id;
            });

  if (level_segments.empty()) {
    LOG(INFO) << "no level segments to cluster";
    return;
  }

  std::unordered_map<std::string, int> node_by_segment_id;
  for (std::size_t node = 0; node < level_segments.size(); ++node) {
    node_by_segment_id[level_segments[node]->id] = static_cast<int>(node);
  }

  std::vector<SameLevelEdge> same_level_edges;
  std::unordered_set<std::uint64_t> cannot_link_pairs;
  for (const SegmentRelation& relation : segment_relations_) {
    const auto first_node_iter =
        node_by_segment_id.find(relation.first_segment_id);
    const auto second_node_iter =
        node_by_segment_id.find(relation.second_segment_id);
    if (first_node_iter == node_by_segment_id.end() ||
        second_node_iter == node_by_segment_id.end()) {
      continue;
    }

    if (relation.relation == LayerRelation::kSameLevel) {
      same_level_edges.push_back(
          {first_node_iter->second, second_node_iter->second, relation.score});
    } else if (relation.relation == LayerRelation::kCannotLink) {
      cannot_link_pairs.insert(
          MakeNodePairKey(first_node_iter->second, second_node_iter->second));
    }
  }

  for (const auto& trip_and_segments : trip_segments_) {
    const std::vector<TripSegment>& segments = trip_and_segments.second;
    for (std::size_t i = 1; i < segments.size(); ++i) {
      const TripSegment& previous = segments[i - 1];
      const TripSegment& current = segments[i];
      if (previous.is_ramp || current.is_ramp) {
        continue;
      }
      same_level_edges.push_back({node_by_segment_id.at(previous.id),
                                  node_by_segment_id.at(current.id), 1.0});
    }
  }

  std::sort(same_level_edges.begin(), same_level_edges.end(),
            [](const SameLevelEdge& first, const SameLevelEdge& second) {
              if (first.score != second.score) {
                return first.score > second.score;
              }
              if (first.first != second.first) {
                return first.first < second.first;
              }
              return first.second < second.second;
            });

  UnionFind union_find(level_segments.size());
  std::size_t merged_edge_count = 0;
  std::size_t rejected_edge_count = 0;
  for (const SameLevelEdge& edge : same_level_edges) {
    const int first_root = union_find.Find(edge.first);
    const int second_root = union_find.Find(edge.second);
    if (first_root == second_root) {
      continue;
    }
    if (HasCannotLink(union_find, first_root, second_root, cannot_link_pairs)) {
      ++rejected_edge_count;
      continue;
    }
    union_find.Union(first_root, second_root);
    ++merged_edge_count;
  }

  std::map<int, std::vector<std::string>> segment_ids_by_root;
  for (std::size_t node = 0; node < level_segments.size(); ++node) {
    const int root = union_find.Find(static_cast<int>(node));
    segment_ids_by_root[root].push_back(level_segments[node]->id);
  }

  std::unordered_map<int, std::size_t> surface_index_by_root;
  for (auto& root_and_segment_ids : segment_ids_by_root) {
    const int root = root_and_segment_ids.first;
    std::vector<std::string>& segment_ids = root_and_segment_ids.second;
    std::sort(segment_ids.begin(), segment_ids.end());

    std::vector<double> segment_heights;
    for (const std::string& segment_id : segment_ids) {
      const int node = node_by_segment_id.at(segment_id);
      segment_heights.push_back(level_segments[node]->center.z());
    }

    RoadSurface surface;
    surface.representative_height = Median(std::move(segment_heights));
    surface.segment_ids = std::move(segment_ids);
    surface_index_by_root[root] = road_surfaces_.size();
    road_surfaces_.push_back(std::move(surface));
  }

  std::vector<std::vector<std::size_t>> upper_surfaces(road_surfaces_.size());
  std::vector<int> indegree(road_surfaces_.size(), 0);
  std::unordered_set<std::uint64_t> surface_layer_edges;
  for (const SegmentRelation& relation : segment_relations_) {
    if (relation.relation != LayerRelation::kCannotLink ||
        relation.signed_z_difference == 0.0) {
      continue;
    }

    const int first_node = node_by_segment_id.at(relation.first_segment_id);
    const int second_node = node_by_segment_id.at(relation.second_segment_id);
    const std::size_t first_surface =
        surface_index_by_root.at(union_find.Find(first_node));
    const std::size_t second_surface =
        surface_index_by_root.at(union_find.Find(second_node));
    if (first_surface == second_surface) {
      continue;
    }

    const std::size_t lower_surface =
        relation.signed_z_difference > 0.0 ? first_surface : second_surface;
    const std::size_t upper_surface =
        relation.signed_z_difference > 0.0 ? second_surface : first_surface;
    const std::uint64_t edge_key = MakeDirectedPairKey(
        static_cast<int>(lower_surface), static_cast<int>(upper_surface));
    if (!surface_layer_edges.insert(edge_key).second) {
      continue;
    }
    upper_surfaces[lower_surface].push_back(upper_surface);
    ++indegree[upper_surface];
  }

  std::set<std::pair<double, std::size_t>> ready_surfaces;
  for (std::size_t i = 0; i < road_surfaces_.size(); ++i) {
    if (indegree[i] == 0) {
      ready_surfaces.insert({road_surfaces_[i].representative_height, i});
    }
  }

  std::vector<std::size_t> ordered_surface_indices;
  while (!ready_surfaces.empty()) {
    const std::size_t surface_index = ready_surfaces.begin()->second;
    ready_surfaces.erase(ready_surfaces.begin());
    ordered_surface_indices.push_back(surface_index);

    for (std::size_t upper_surface : upper_surfaces[surface_index]) {
      --indegree[upper_surface];
      if (indegree[upper_surface] == 0) {
        ready_surfaces.insert(
            {road_surfaces_[upper_surface].representative_height,
             upper_surface});
      }
    }
  }

  if (ordered_surface_indices.size() != road_surfaces_.size()) {
    LOG(WARNING) << "surface layer relations contain a cycle, order remaining "
                    "surfaces by representative height";
    std::vector<std::size_t> remaining_surfaces;
    std::vector<bool> already_ordered(road_surfaces_.size(), false);
    for (std::size_t surface_index : ordered_surface_indices) {
      already_ordered[surface_index] = true;
    }
    for (std::size_t i = 0; i < road_surfaces_.size(); ++i) {
      if (!already_ordered[i]) {
        remaining_surfaces.push_back(i);
      }
    }
    std::sort(remaining_surfaces.begin(), remaining_surfaces.end(),
              [this](std::size_t first, std::size_t second) {
                return road_surfaces_[first].representative_height <
                       road_surfaces_[second].representative_height;
              });
    ordered_surface_indices.insert(ordered_surface_indices.end(),
                                   remaining_surfaces.begin(),
                                   remaining_surfaces.end());
  }

  std::vector<RoadSurface> ordered_surfaces;
  ordered_surfaces.reserve(road_surfaces_.size());
  for (std::size_t surface_index : ordered_surface_indices) {
    RoadSurface surface = std::move(road_surfaces_[surface_index]);
    surface.id = "surface_" + std::to_string(ordered_surfaces.size());
    LOG(INFO) << surface.id << " segments=" << surface.segment_ids.size()
              << ", representative_height=" << surface.representative_height;
    ordered_surfaces.push_back(std::move(surface));
  }
  road_surfaces_ = std::move(ordered_surfaces);

  LOG(INFO) << "surface clustering summary: level_segments="
            << level_segments.size()
            << ", same_level_edges=" << same_level_edges.size()
            << ", cannot_links=" << cannot_link_pairs.size()
            << ", merged_edges=" << merged_edge_count
            << ", rejected_edges=" << rejected_edge_count
            << ", layer_edges=" << surface_layer_edges.size()
            << ", surfaces=" << road_surfaces_.size();
}

}  // namespace mapping
}  // namespace adlabel
