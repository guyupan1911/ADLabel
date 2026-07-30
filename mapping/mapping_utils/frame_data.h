#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "mapping/common/pcl_types.h"
#include "mapping/protos/camera_calibration.pb.h"

namespace adlabel {
namespace mapping {

struct FrameData {
  using Cloud = PointCloudXYZIRT;

  enum class SensorType {
    kUnknown = 0,
    kLidar = 1,
    kCamera = 2,
  };

  SensorType sensor_type = SensorType::kUnknown;

  // Static points after range filtering.
  Cloud::Ptr raw_cloud;
  // Classified clouds are null when lidar labels are unavailable.
  Cloud::Ptr ground_cloud;
  Cloud::Ptr non_ground_cloud;

  cv::Mat camera_image;
  CameraCalibration camera_calibration;

  Eigen::Affine3d pose_utm = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_ecef = Eigen::Affine3d::Identity();

  Eigen::Affine3d transform_from_sensor_to_imu = Eigen::Affine3d::Identity();
};

}  // namespace mapping
}  // namespace adlabel
