#include "mapping/mapping_utils/stitch_utils.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"
#include "mapping/protos/lidar_detection_3d.pb.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr double kDetectionBoxLengthExtensionM = 0.2;

struct LidarBox {
  Eigen::Affine3d T_lidar_to_box = Eigen::Affine3d::Identity();
  Eigen::Vector3d half_size = Eigen::Vector3d::Zero();
};

bool BuildLidarBox(const LidarDetectionObject& object,
                   const Eigen::Affine3d& T_imu_to_lidar,
                   LidarBox* lidar_box) {
  CHECK(lidar_box != nullptr);
  if (!object.has_box3d() || !object.box3d().has_center() ||
      !object.box3d().has_size()) {
    return false;
  }

  const auto& box = object.box3d();
  const auto& center = box.center();
  const auto& size = box.size();
  if (size.length() <= 0.0 || size.width() <= 0.0 || size.height() <= 0.0) {
    return false;
  }

  Eigen::Affine3d T_box_to_imu = Eigen::Affine3d::Identity();
  T_box_to_imu.translation() = Eigen::Vector3d(center.x(), center.y(), center.z());
  T_box_to_imu.linear() =
      Eigen::AngleAxisd(box.yaw(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  lidar_box->T_lidar_to_box = (T_imu_to_lidar * T_box_to_imu).inverse();
  lidar_box->half_size = Eigen::Vector3d(
      size.length() * 0.5 + kDetectionBoxLengthExtensionM,
      size.width() * 0.5,
      size.height() * 0.5);
  return true;
}

bool IsPointInsideBox(const PointXYZIRT& point, const LidarBox& box) {
  const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
  const Eigen::Vector3d point_box = box.T_lidar_to_box * point_lidar;
  return std::abs(point_box.x()) <= box.half_size.x() &&
         std::abs(point_box.y()) <= box.half_size.y() &&
         std::abs(point_box.z()) <= box.half_size.z();
}

bool IsPointInsideAnyBox(const PointXYZIRT& point,
                         const std::vector<LidarBox>& boxes) {
  for (const auto& box : boxes) {
    if (IsPointInsideBox(point, box)) {
      return true;
    }
  }
  return false;
}

FrameData::Cloud::Ptr RemovePointsInsideBoxes(
    const FrameData::Cloud::Ptr& raw_cloud,
    const std::vector<LidarBox>& boxes) {
  CHECK(raw_cloud != nullptr);

  FrameData::Cloud::Ptr filtered_cloud(new FrameData::Cloud);
  *filtered_cloud = *raw_cloud;
  filtered_cloud->points.clear();
  filtered_cloud->points.reserve(raw_cloud->points.size());

  for (const auto& point : raw_cloud->points) {
    if (!IsPointInsideAnyBox(point, boxes)) {
      filtered_cloud->points.push_back(point);
    }
  }

  filtered_cloud->width = static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = raw_cloud->is_dense;
  return filtered_cloud;
}

}  // namespace

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader) {
  CHECK(lidar_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";

  lidar_frame_data->T_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();

  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  lidar_frame_data->raw_cloud = raw_cloud;

  if (frame.has_lidar_object_detection_uri() &&
      !frame.lidar_object_detection_uri().empty()) {
    const std::vector<LidarDetectionObject> objects =
        local_data_reader->ReadMetaData<LidarDetectionObject>(
            frame.lidar_object_detection_uri());

    const Eigen::Affine3d T_imu_to_lidar =
        lidar_frame_data->T_sensor_to_imu.inverse();
    std::vector<LidarBox> lidar_boxes;
    lidar_boxes.reserve(objects.size());
    for (const auto& object : objects) {
      LidarBox lidar_box;
      if (BuildLidarBox(object, T_imu_to_lidar, &lidar_box)) {
        lidar_boxes.push_back(lidar_box);
      }
    }

    if (!lidar_boxes.empty()) {
      lidar_frame_data->raw_cloud = RemovePointsInsideBoxes(raw_cloud, lidar_boxes);
      LOG(INFO) << "filtered lidar cloud points: " << raw_cloud->points.size()
                << " -> " << lidar_frame_data->raw_cloud->points.size();
    }
  }

  return true;
}

}  // namespace mapping
}  // namespace adlabel
