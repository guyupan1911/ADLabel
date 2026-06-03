#pragma once

#include <vector>

#include <Eigen/Dense>
#include <pcl/pointcloud.h>

#include "pcl/pcl_types.h"

namespace adlabel {
namespace mapping {

struct FrameData {
  using Cloud = pcl::PointCloud<PointXYZIRT>;

  Cloud::Ptr raw_cloud;

  std::vector<int> ground_indices;
  std::vector<int> non_ground_indices;

  Eigen::Affine3d T_sensor_to_imu;
};

}
}