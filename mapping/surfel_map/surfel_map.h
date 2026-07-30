#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mapping/lidar_topdown/grid_frame.h"

namespace adlabel {
namespace mapping {

struct VoxelKey {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const noexcept;
};

struct SurfelMapOptions {
  double voxel_resolution = 0.20;
  int texture_grid_size = 4;
  uint32_t minimum_point_count = 3;
  size_t maximum_camera_candidates = 8;
};

struct CameraCandidate {
  uint32_t camera_frame_index = 0;
  float support_score = 0.0f;
};

struct SurfelTextureCell {
  cv::Vec3b bgr{0, 0, 0};
  float score = -1.0f;
  uint32_t source_camera_frame_index = 0;
  bool valid = false;
};

enum class SurfelInvalidReason {
  kNone = 0,
  kInsufficientPoints,
  kEigenDecompositionFailure,
};

struct SurfelGeometryStatistics {
  size_t total_voxels = 0;
  size_t valid_surfels = 0;
  size_t insufficient_points = 0;
  size_t eigen_decomposition_failures = 0;

  size_t InvalidSurfelCount() const {
    return insufficient_points + eigen_decomposition_failures;
  }
};

struct SurfelNode {
  uint32_t point_count = 0;
  Eigen::Vector3d point_sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d point_outer_sum = Eigen::Matrix3d::Zero();

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  float plane_rms = 0.0f;
  bool geometry_valid = false;
  SurfelInvalidReason invalid_reason = SurfelInvalidReason::kNone;

  std::vector<CameraCandidate> camera_candidates;
  std::vector<SurfelTextureCell> texture_cells;
};

class SurfelMap {
 public:
  using SurfelContainer =
      std::unordered_map<VoxelKey, SurfelNode, VoxelKeyHash>;

  explicit SurfelMap(const SurfelMapOptions& options);

  void InsertGeometryPoint(const Eigen::Vector3d& point_map);
  bool FinalizeGeometry();

  VoxelKey GetVoxelKey(const Eigen::Vector3d& point_map) const;
  bool AddCameraCandidate(const Eigen::Vector3d& support_point_map,
                          const CameraCandidate& candidate);

  SurfelNode* FindSurfel(const VoxelKey& key);
  const SurfelNode* FindSurfel(const VoxelKey& key) const;

  Eigen::Vector3d GetTextureCellCenter(const VoxelKey& key,
                                       const SurfelNode& surfel,
                                       int cell_index) const;

  bool UpdateTextureCell(const VoxelKey& key, int cell_index,
                         const cv::Vec3b& bgr, float score,
                         uint32_t camera_frame_index);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetColoredPointCloud() const;
  bool RenderBev(const GridFrame& frame, cv::Mat* rgb, cv::Mat* coverage) const;

  const SurfelContainer& surfels() const { return surfels_; }
  const SurfelMapOptions& options() const { return options_; }
  const SurfelGeometryStatistics& geometry_statistics() const {
    return geometry_statistics_;
  }

  size_t ValidSurfelCount() const;
  size_t ColoredCellCount() const;

 private:
  SurfelMapOptions options_;
  SurfelContainer surfels_;
  SurfelGeometryStatistics geometry_statistics_;
  bool geometry_finalized_ = false;
};

}  // namespace mapping
}  // namespace adlabel
