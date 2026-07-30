#include "mapping/mapping_utils/stitch_utils.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr double kMinDistanceFromOriginM = 9.0;
constexpr double kMaxDistanceFromOriginM = 120.0;
constexpr double kMinDistanceFromOriginSquaredM2 =
    kMinDistanceFromOriginM * kMinDistanceFromOriginM;
constexpr double kMaxDistanceFromOriginSquaredM2 =
    kMaxDistanceFromOriginM * kMaxDistanceFromOriginM;

bool ReadInt32Labels(const std::shared_ptr<LocalDataReader>& local_data_reader,
                     const std::string& uri,
                     std::vector<std::int32_t>* labels) {
  CHECK(local_data_reader != nullptr);
  CHECK(labels != nullptr);
  labels->clear();

  std::vector<char> bytes;
  if (!local_data_reader->ReadBinaryFile(uri, &bytes)) {
    LOG(WARNING) << "failed to read lidar_nn_uri file: " << uri;
    return false;
  }
  if (bytes.size() % sizeof(std::int32_t) != 0) {
    LOG(WARNING) << "lidar_nn_uri file size is not int32-aligned: " << uri
                 << ", size=" << bytes.size();
    return false;
  }

  labels->resize(bytes.size() / sizeof(std::int32_t));
  if (!bytes.empty()) {
    std::memcpy(labels->data(), bytes.data(), bytes.size());
  }
  return true;
}

void FinalizeCloud(const FrameData::Cloud& input_cloud,
                   FrameData::Cloud* output_cloud) {
  CHECK(output_cloud != nullptr);
  output_cloud->width = static_cast<std::uint32_t>(output_cloud->points.size());
  output_cloud->height = 1;
  output_cloud->is_dense = input_cloud.is_dense;
}

}  // namespace

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader) {
  CHECK(lidar_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";
  *lidar_frame_data = FrameData();
  lidar_frame_data->sensor_type = FrameData::SensorType::kLidar;
  // if (!frame.has_refined_pose_3d()) {
  //   LOG(WARNING) << "frame has no refined_pose_3d: " << frame.fid();
  //   return false;
  // }

  lidar_frame_data->transform_from_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();
  if (frame.has_refined_pose_3d()) {
    lidar_frame_data->pose_ecef = Pose3D(frame.refined_pose_3d()).GetAffine3D();
  }
  if (frame.has_lio_pose_3d()) {
    lidar_frame_data->pose_utm = Pose3D(frame.lio_pose_3d()).GetAffine3D();
  }

  if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
    LOG(WARNING) << "frame has empty cloud_uri: " << frame.fid();
    return false;
  }
  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  std::vector<std::int32_t> lidar_nn_labels;
  bool has_valid_lidar_nn_labels = false;
  if (frame.has_lidar_nn_uri() && !frame.lidar_nn_uri().empty()) {
    if (ReadInt32Labels(local_data_reader, frame.lidar_nn_uri(),
                        &lidar_nn_labels)) {
      if (lidar_nn_labels.size() == raw_cloud->points.size()) {
        has_valid_lidar_nn_labels = true;
      } else {
        LOG(WARNING)
            << "lidar_nn_uri label count does not match cloud point count: "
            << lidar_nn_labels.size() << " vs " << raw_cloud->points.size()
            << ", skip lidar nn label filtering, uri=" << frame.lidar_nn_uri();
      }
    }
  }

  lidar_frame_data->raw_cloud.reset(new FrameData::Cloud);
  lidar_frame_data->raw_cloud->points.reserve(raw_cloud->points.size());
  if (has_valid_lidar_nn_labels) {
    lidar_frame_data->ground_cloud.reset(new FrameData::Cloud);
    lidar_frame_data->non_ground_cloud.reset(new FrameData::Cloud);
    lidar_frame_data->ground_cloud->points.reserve(raw_cloud->points.size() /
                                                   2);
    lidar_frame_data->non_ground_cloud->points.reserve(
        raw_cloud->points.size() / 2);
  }

  std::size_t invalid_label_count = 0;
  for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
    const auto& point = raw_cloud->points[i];
    std::int32_t label = 0;
    if (has_valid_lidar_nn_labels) {
      label = lidar_nn_labels[i];
      if (label >= 1) {
        continue;
      }
      if (label != -1 && label != 0) {
        ++invalid_label_count;
        continue;
      }
    }

    const double distance_squared =
        point.x * point.x + point.y * point.y + point.z * point.z;
    if (distance_squared < kMinDistanceFromOriginSquaredM2 ||
        distance_squared > kMaxDistanceFromOriginSquaredM2) {
      continue;
    }
    lidar_frame_data->raw_cloud->points.push_back(point);
    if (has_valid_lidar_nn_labels) {
      if (label == 0) {
        lidar_frame_data->ground_cloud->points.push_back(point);
      } else {
        lidar_frame_data->non_ground_cloud->points.push_back(point);
      }
    }
  }

  if (invalid_label_count > 0) {
    LOG(WARNING) << "ignored " << invalid_label_count
                 << " points with lidar labels below -1: "
                 << frame.lidar_nn_uri();
  }

  FinalizeCloud(*raw_cloud, lidar_frame_data->raw_cloud.get());
  if (has_valid_lidar_nn_labels) {
    FinalizeCloud(*raw_cloud, lidar_frame_data->ground_cloud.get());
    FinalizeCloud(*raw_cloud, lidar_frame_data->non_ground_cloud.get());
    CHECK_EQ(lidar_frame_data->raw_cloud->size(),
             lidar_frame_data->ground_cloud->size() +
                 lidar_frame_data->non_ground_cloud->size());
  }

  return true;
}

bool GenerateCameraFrameData(
    const Frame& frame, FrameData* camera_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader) {
  CHECK(camera_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";

  *camera_frame_data = FrameData();
  camera_frame_data->sensor_type = FrameData::SensorType::kCamera;
  camera_frame_data->transform_from_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();

  if (frame.has_lio_pose_3d()) {
    camera_frame_data->pose_utm = Pose3D(frame.lio_pose_3d()).GetAffine3D();
  }
  if (frame.has_refined_pose_3d()) {
    camera_frame_data->pose_ecef =
        Pose3D(frame.refined_pose_3d()).GetAffine3D();
  }

  if (!frame.has_camera_image_uri() || frame.camera_image_uri().empty()) {
    LOG(WARNING) << "frame has empty camera_image_uri: " << frame.fid();
    return false;
  }
  if (!frame.has_camera_calibration()) {
    LOG(WARNING) << "frame has no camera_calibration: " << frame.fid();
    return false;
  }
  if (!local_data_reader->ReadImage(frame.camera_image_uri(),
                                    &camera_frame_data->camera_image)) {
    return false;
  }

  camera_frame_data->camera_calibration = frame.camera_calibration();
  return true;
}

}  // namespace mapping
}  // namespace adlabel
