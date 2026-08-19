#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "mapping/protos/frame.pb.h"
#include "mapping/road_layering/trip_segment.h"

namespace adlabel {
namespace mapping {

enum class LayerRelation {
  kSameLevel,
  kCannotLink,
  kNoRelation,
};

struct SegmentRelation {
  std::string first_segment_id;
  std::string second_segment_id;
  LayerRelation relation = LayerRelation::kNoRelation;
  double score = 0.0;
  double min_xy_distance = 0.0;
  double signed_z_difference = 0.0;
  double median_z_difference = 0.0;
  std::size_t matched_pose_count = 0;
};

struct RoadSurface {
  std::string id;
  std::vector<std::string> segment_ids;
  double representative_height = 0.0;
};

class RoadLayering {
 public:
  void AddFrame(const Frame& frame);

  void Run();

  const std::unordered_map<std::string, std::vector<TripSegment>>&
  trip_segments() const {
    return trip_segments_;
  }

  const Frame& GetFrame(const std::string& fid) const {
    return frames_by_fid_.at(fid);
  }

  const std::vector<SegmentRelation>& segment_relations() const {
    return segment_relations_;
  }

  const std::vector<RoadSurface>& road_surfaces() const {
    return road_surfaces_;
  }

 private:
  void SegmentTripFrames(const std::vector<std::string>& frame_ids);
  void BuildSegmentRelations();
  void ClusterRoadSurfaces();
  SegmentRelation EstimateLayerRelation(const TripSegment& first,
                                        const TripSegment& second) const;

  std::unordered_map<std::string, Frame> frames_by_fid_;
  std::unordered_map<std::string, std::vector<std::string>> frame_ids_by_trip_;
  std::unordered_map<std::string, std::vector<TripSegment>> trip_segments_;
  std::vector<SegmentRelation> segment_relations_;
  std::vector<RoadSurface> road_surfaces_;
  bool has_enu_origin_ = false;
  Eigen::Affine3d T_enu_ecef_ = Eigen::Affine3d::Identity();
};

}  // namespace mapping
}  // namespace adlabel
