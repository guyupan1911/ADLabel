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
  SmallGicpPointCloudPtr reference_cloud_ = nullptr;
  SmallGicpKdTreePtr reference_cloud_kdtree_ = nullptr;

  Eigen::Affine3d T_world_reference_ = Eigen::Affine3d::Identity();
};

}  // namespace mapping
}  // namespace adlabel
