#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace adlabel {
namespace mapping {

struct EIGEN_ALIGN16 PointXYZIRT {
  PCL_ADD_POINT4D;   // float x, y, z + 4-byte padding = 16 bytes
  float intensity;   // 4 bytes
  uint16_t ring;     // 2 bytes
  uint8_t _pad[6];   // align timestamp to 8-byte boundary
  double timestamp;  // 8 bytes
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PointCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;

}  // namespace mapping
}  // namespace adlabel

// Must be in global namespace
POINT_CLOUD_REGISTER_POINT_STRUCT(
    adlabel::mapping::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))
