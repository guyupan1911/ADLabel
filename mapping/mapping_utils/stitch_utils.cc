#include "mapping/mapping_utils/stitch_utils.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <utility>
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

const Pose3DMessage* GetLocalizationPose(const Frame& frame, bool use_lio_pose) {
  if (use_lio_pose) {
    if (frame.has_refined_pose_3d()) {
      return &frame.refined_pose_3d();
    }
    return frame.has_lio_pose_3d() ? &frame.lio_pose_3d() : nullptr;
  }
  return frame.has_gnss_pose_3d() ? &frame.gnss_pose_3d() : nullptr;
}

const char* LocalizationPoseName(bool use_lio_pose) {
  return use_lio_pose ? "refined_pose_3d/lio_pose_3d" : "gnss_pose_3d";
}

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

constexpr std::uint32_t kLidarNnGroundLabel = 0;
constexpr double kEgoVehicleMinXM = -2.5;
constexpr double kEgoVehicleMaxXM = 3.5;
constexpr double kEgoVehicleMinYM = -1.3;
constexpr double kEgoVehicleMaxYM = 1.3;
constexpr double kEgoVehicleMinZM = -2.5;
constexpr double kEgoVehicleMaxZM = 1.0;

bool ReadUint32Labels(const std::string& uri, std::vector<std::uint32_t>* labels) {
  CHECK(labels != nullptr);
  labels->clear();

  const std::filesystem::path path(uri);
  if (!std::filesystem::exists(path)) {
    LOG(WARNING) << "lidar_nn_uri file does not exist: " << uri;
    return false;
  }

  const std::uintmax_t file_size = std::filesystem::file_size(path);
  if (file_size % sizeof(std::uint32_t) != 0) {
    LOG(WARNING) << "lidar_nn_uri file size is not uint32-aligned: "
                 << uri << ", size=" << file_size;
    return false;
  }

  labels->resize(file_size / sizeof(std::uint32_t));
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    LOG(WARNING) << "failed to open lidar_nn_uri file: " << uri;
    labels->clear();
    return false;
  }

  input.read(reinterpret_cast<char*>(labels->data()),
             static_cast<std::streamsize>(file_size));
  if (!input) {
    LOG(WARNING) << "failed to read lidar_nn_uri file: " << uri;
    labels->clear();
    return false;
  }
  return true;
}

FrameData::Cloud::Ptr KeepGroundPointsByLidarNnLabels(
    const FrameData::Cloud::Ptr& raw_cloud,
    const std::vector<std::uint32_t>& labels,
    FrameData* frame_data) {
  CHECK(raw_cloud != nullptr);
  CHECK(frame_data != nullptr);

  FrameData::Cloud::Ptr filtered_cloud(new FrameData::Cloud);
  *filtered_cloud = *raw_cloud;
  filtered_cloud->points.clear();
  filtered_cloud->points.reserve(raw_cloud->points.size());
  frame_data->ground_indices.clear();
  frame_data->non_ground_indices.clear();

  for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
    if (labels[i] == kLidarNnGroundLabel) {
      frame_data->ground_indices.push_back(static_cast<int>(i));
      filtered_cloud->points.push_back(raw_cloud->points[i]);
    } else {
      frame_data->non_ground_indices.push_back(static_cast<int>(i));
    }
  }

  filtered_cloud->width = static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = raw_cloud->is_dense;
  return filtered_cloud;
}

bool IsInsideEgoVehicleBox(const PointXYZIRT& point) {
  return point.x >= kEgoVehicleMinXM && point.x <= kEgoVehicleMaxXM &&
         point.y >= kEgoVehicleMinYM && point.y <= kEgoVehicleMaxYM &&
         point.z >= kEgoVehicleMinZM && point.z <= kEgoVehicleMaxZM;
}

FrameData::Cloud::Ptr RemoveEgoVehiclePoints(
    const FrameData::Cloud::Ptr& raw_cloud,
    std::vector<int>* point_indices) {
  CHECK(raw_cloud != nullptr);

  FrameData::Cloud::Ptr filtered_cloud(new FrameData::Cloud);
  *filtered_cloud = *raw_cloud;
  filtered_cloud->points.clear();
  filtered_cloud->points.reserve(raw_cloud->points.size());

  const bool update_indices = point_indices != nullptr && !point_indices->empty();
  std::vector<int> filtered_indices;
  if (update_indices) {
    CHECK_EQ(point_indices->size(), raw_cloud->points.size());
    filtered_indices.reserve(point_indices->size());
  }

  for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
    if (IsInsideEgoVehicleBox(raw_cloud->points[i])) {
      continue;
    }
    filtered_cloud->points.push_back(raw_cloud->points[i]);
    if (update_indices) {
      filtered_indices.push_back((*point_indices)[i]);
    }
  }

  if (update_indices) {
    *point_indices = std::move(filtered_indices);
  }
  filtered_cloud->width = static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = raw_cloud->is_dense;
  return filtered_cloud;
}

bool DeskewPointCloud(const Frame& frame,
                      const SimplePose3DInterpolator& localization_pose_interpolator,
                      const Eigen::Affine3d& T_lidar_to_imu,
                      FrameData::Cloud::Ptr cloud,
                      bool use_lio_pose) {
  CHECK(cloud != nullptr);

  if (GetLocalizationPose(frame, use_lio_pose) == nullptr) {
    LOG(WARNING) << "frame does not have " << LocalizationPoseName(use_lio_pose)
                 << ", skip deskewing";
    return false;
  }

  Pose3D localization_pose_ref;
  if (!localization_pose_interpolator.GetTimestampedPose(frame.timestamp_ns(),
                                                         &localization_pose_ref,
                                                         false)) {
    LOG(WARNING) << "failed to interpolate "
                 << LocalizationPoseName(use_lio_pose)
                 << " at frame timestamp "
                 << frame.timestamp_ns() << ", skip deskewing";
    return false;
  }

  const Eigen::Affine3d T_imu_to_world_ref = localization_pose_ref.GetAffine3D();
  const Eigen::Affine3d T_imu_to_lidar = T_lidar_to_imu.inverse();

  size_t deskewed_count = 0;
  size_t invalid_timestamp_count = 0;
  size_t interpolation_failed_count = 0;

  for (auto& point : cloud->points) {
    if (!std::isfinite(point.timestamp) || point.timestamp < 0.0) {
      ++invalid_timestamp_count;
      continue;
    }

    const int64_t point_timestamp_ns = static_cast<int64_t>(point.timestamp * 1e9);

    Pose3D localization_pose_point;
    if (!localization_pose_interpolator.GetTimestampedPose(point_timestamp_ns,
                                                           &localization_pose_point,
                                                           false)) {
      ++interpolation_failed_count;
      continue;
    }

    const Eigen::Affine3d T_imu_to_world_point = localization_pose_point.GetAffine3D();

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
    const SimplePose3DInterpolator* lio_pose_interpolator,
    bool use_lio_pose) {
  CHECK(lidar_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";

  lidar_frame_data->T_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();

  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  // if (lio_pose_interpolator != nullptr) {
  //   DeskewPointCloud(frame, *lio_pose_interpolator,
  //                    lidar_frame_data->T_sensor_to_imu, raw_cloud, use_lio_pose);
  // }

  lidar_frame_data->raw_cloud = raw_cloud;
  lidar_frame_data->ground_indices.clear();
  lidar_frame_data->non_ground_indices.clear();
  // LOG(INFO) << "loaded lidar cloud points: " << raw_cloud->points.size()
  //           << ", uri=" << frame.cloud_uri();

  if (frame.has_lidar_nn_uri() && !frame.lidar_nn_uri().empty()) {
    std::vector<std::uint32_t> lidar_nn_labels;
    if (ReadUint32Labels(frame.lidar_nn_uri(), &lidar_nn_labels)) {
      if (lidar_nn_labels.size() == raw_cloud->points.size()) {
        lidar_frame_data->raw_cloud =
            KeepGroundPointsByLidarNnLabels(raw_cloud, lidar_nn_labels,
                                            lidar_frame_data);
        // LOG(INFO) << "filtered lidar cloud by lidar_nn_uri ground labels: "
        //           << raw_cloud->points.size() << " -> "
        //           << lidar_frame_data->raw_cloud->points.size()
        //           << ", non_ground=" << lidar_frame_data->non_ground_indices.size()
        //           << ", uri=" << frame.lidar_nn_uri();
      } else {
        LOG(WARNING) << "lidar_nn_uri label count does not match cloud point count: "
                     << lidar_nn_labels.size() << " vs " << raw_cloud->points.size()
                     << ", skip ground filtering, uri=" << frame.lidar_nn_uri();
      }
    }
  }

  const std::size_t before_ego_filter_count =
      lidar_frame_data->raw_cloud->points.size();
  lidar_frame_data->raw_cloud = RemoveEgoVehiclePoints(
      lidar_frame_data->raw_cloud, &lidar_frame_data->ground_indices);
  // LOG(INFO) << "filtered lidar cloud by ego vehicle box in lidar frame: "
  //           << before_ego_filter_count << " -> "
  //           << lidar_frame_data->raw_cloud->points.size()
  //           << ", box_x=[" << kEgoVehicleMinXM << ", " << kEgoVehicleMaxXM
  //           << "], box_y=[" << kEgoVehicleMinYM << ", " << kEgoVehicleMaxYM
  //           << "], box_z=[" << kEgoVehicleMinZM << ", " << kEgoVehicleMaxZM << "]";

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
