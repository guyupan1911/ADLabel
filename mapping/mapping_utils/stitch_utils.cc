#include "mapping/mapping_utils/stitch_utils.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
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

bool DeskewPointCloud(const Frame& frame,
                      const SimplePose3DInterpolator& lio_pose_interpolator,
                      const Eigen::Affine3d& T_lidar_to_imu,
                      FrameData::Cloud::Ptr cloud) {
  CHECK(cloud != nullptr);

  if (!frame.has_lio_pose_3d()) {
    LOG(WARNING) << "frame does not have lio_pose_3d, skip deskewing";
    return false;
  }

  Pose3D lio_pose_ref;
  if (!lio_pose_interpolator.GetTimestampedPose(frame.timestamp_ns(),
                                                 &lio_pose_ref,
                                                 false)) {
    LOG(WARNING) << "failed to interpolate LIO pose at frame timestamp "
                 << frame.timestamp_ns() << ", skip deskewing";
    return false;
  }

  const Eigen::Affine3d T_imu_to_world_ref = lio_pose_ref.GetAffine3D();
  const Eigen::Affine3d T_imu_to_lidar = T_lidar_to_imu.inverse();

  size_t deskewed_count = 0;
  size_t invalid_timestamp_count = 0;
  size_t interpolation_failed_count = 0;

  for (auto& point : cloud->points) {
    if (!std::isfinite(point.timestamp) || point.timestamp < 0.0) {
      ++invalid_timestamp_count;
      continue;
    }

    const int64_t point_timestamp_ns =
        frame.timestamp_ns() + static_cast<int64_t>(point.timestamp * 1e9);

    Pose3D lio_pose_point;
    if (!lio_pose_interpolator.GetTimestampedPose(point_timestamp_ns,
                                                   &lio_pose_point,
                                                   false)) {
      ++interpolation_failed_count;
      continue;
    }

    const Eigen::Affine3d T_imu_to_world_point = lio_pose_point.GetAffine3D();

    const Eigen::Vector3d point_lidar_at_t(point.x, point.y, point.z);
    const Eigen::Vector3d point_deskewed =
        T_imu_to_lidar * T_imu_to_world_ref.inverse() *
        T_imu_to_world_point * T_lidar_to_imu * point_lidar_at_t;

    point.x = static_cast<float>(point_deskewed.x());
    point.y = static_cast<float>(point_deskewed.y());
    point.z = static_cast<float>(point_deskewed.z());

    ++deskewed_count;
  }

  LOG(INFO) << "point cloud deskewing completed: deskewed=" << deskewed_count
            << "/" << cloud->points.size()
            << ", invalid_timestamp=" << invalid_timestamp_count
            << ", interpolation_failed=" << interpolation_failed_count;

  return deskewed_count > 0;
}

}  // namespace

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader,
    const SimplePose3DInterpolator* lio_pose_interpolator) {
  CHECK(lidar_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";

  lidar_frame_data->T_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();

  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  if (lio_pose_interpolator != nullptr) {
    DeskewPointCloud(frame, *lio_pose_interpolator,
                     lidar_frame_data->T_sensor_to_imu, raw_cloud);
  }

  lidar_frame_data->raw_cloud = raw_cloud;

  // if (frame.has_lidar_object_detection_uri() &&
  //     !frame.lidar_object_detection_uri().empty()) {
  //   const std::vector<LidarDetectionObject> objects =
  //       local_data_reader->ReadMetaData<LidarDetectionObject>(
  //           frame.lidar_object_detection_uri());

  //   const Eigen::Affine3d T_imu_to_lidar =
  //       lidar_frame_data->T_sensor_to_imu.inverse();
  //   std::vector<LidarBox> lidar_boxes;
  //   lidar_boxes.reserve(objects.size());
  //   for (const auto& object : objects) {
  //     LidarBox lidar_box;
  //     if (BuildLidarBox(object, T_imu_to_lidar, &lidar_box)) {
  //       lidar_boxes.push_back(lidar_box);
  //     }
  //   }

  //   if (!lidar_boxes.empty()) {
  //     lidar_frame_data->raw_cloud = RemovePointsInsideBoxes(raw_cloud, lidar_boxes);
  //     LOG(INFO) << "filtered lidar cloud points: " << raw_cloud->points.size()
  //               << " -> " << lidar_frame_data->raw_cloud->points.size();
  //   }
  // }

  return true;
}

}  // namespace mapping
}  // namespace adlabel
