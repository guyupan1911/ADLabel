#pragma once

#include <Eigen/Dense>

#include "mapping/common/pcl_types.h"

namespace adlabel {
namespace mapping {

struct FrameData {
  using Cloud = PointCloudXYZIRT;

  // Static points after range filtering.
  Cloud::Ptr raw_cloud;
  // Classified clouds are null when lidar labels are unavailable.
  Cloud::Ptr ground_cloud;
  Cloud::Ptr non_ground_cloud;

  Eigen::Affine3d pose_utm;
  Eigen::Affine3d pose_ecef;

  Eigen::Affine3d transform_from_sensor_to_imu;
};

}  // namespace mapping
}  // namespace adlabel
