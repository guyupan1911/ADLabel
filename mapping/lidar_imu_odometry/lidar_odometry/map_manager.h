#pragma once

#include <Eigen/Geometry>

#include "mapping/common/pcl_types.h"
#include "mapping/registration/small_gicp_types.h"

namespace adlabel {
namespace mapping {

class MapManager {
 public:
  Eigen::Affine3d AlignScanToMap(const PointCloudXYZIRT& cloud,
                                 SmallGicpRegistrationResult& align_result);

 private:
  SmallGicpIncrementalVoxelMapPtr voxel_map_ = nullptr;
  Eigen::Affine3d T_world_lidar_ = Eigen::Affine3d::Identity();
};

}  // namespace mapping
}  // namespace adlabel
