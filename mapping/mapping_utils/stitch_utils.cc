#include "mapping/mapping_utils/stitch_utils.h"

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr std::uint32_t kLidarNnGroundLabel = 0;
constexpr double kEgoVehicleMinXM = -2.5;
constexpr double kEgoVehicleMaxXM = 3.5;
constexpr double kEgoVehicleMinYM = -1.3;
constexpr double kEgoVehicleMaxYM = 1.3;
constexpr double kEgoVehicleMinZM = -2.5;
constexpr double kEgoVehicleMaxZM = 1.0;

bool ReadUint32Labels(const std::shared_ptr<LocalDataReader>& local_data_reader,
                      const std::string& uri,
                      std::vector<std::uint32_t>* labels) {
  CHECK(local_data_reader != nullptr);
  CHECK(labels != nullptr);
  labels->clear();

  std::vector<char> bytes;
  if (!local_data_reader->ReadBinaryFile(uri, &bytes)) {
    LOG(WARNING) << "failed to read lidar_nn_uri file: " << uri;
    return false;
  }
  if (bytes.size() % sizeof(std::uint32_t) != 0) {
    LOG(WARNING) << "lidar_nn_uri file size is not uint32-aligned: "
                 << uri << ", size=" << bytes.size();
    return false;
  }

  labels->resize(bytes.size() / sizeof(std::uint32_t));
  if (!bytes.empty()) {
    std::memcpy(labels->data(), bytes.data(), bytes.size());
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


}  // namespace

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader) {
  CHECK(lidar_frame_data != nullptr);
  CHECK(local_data_reader != nullptr);
  CHECK(frame.has_sensor_to_imu_extrinsic())
      << "frame has no sensor_to_imu_extrinsic";
  if (!frame.has_refined_pose_3d()) {
    LOG(WARNING) << "frame has no refined_pose_3d: " << frame.fid();
    return false;
  }

  lidar_frame_data->transform_from_sensor_to_imu =
      Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D();
  lidar_frame_data->pose_ecef =
      Pose3D(frame.refined_pose_3d()).GetAffine3D();

  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  lidar_frame_data->raw_cloud = raw_cloud;
  lidar_frame_data->ground_indices.clear();
  lidar_frame_data->non_ground_indices.clear();
  // LOG(INFO) << "loaded lidar cloud points: " << raw_cloud->points.size()
  //           << ", uri=" << frame.cloud_uri();

  if (frame.has_lidar_nn_uri() && !frame.lidar_nn_uri().empty()) {
    std::vector<std::uint32_t> lidar_nn_labels;
    if (ReadUint32Labels(local_data_reader, frame.lidar_nn_uri(), &lidar_nn_labels)) {
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

  lidar_frame_data->raw_cloud = RemoveEgoVehiclePoints(
      lidar_frame_data->raw_cloud, &lidar_frame_data->ground_indices);

  return true;
}

}  // namespace mapping
}  // namespace adlabel
