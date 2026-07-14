#include "mapping/mapping_utils/stitch_utils.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <pcl/filters/crop_box.h>

#include "mapping/common/pose3d.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr double kEgoVehicleMinXM = 1.0;
constexpr double kEgoVehicleMaxXM = 6.5;
constexpr double kEgoVehicleMinYM = -3.0;
constexpr double kEgoVehicleMaxYM = 3.0;
constexpr double kEgoVehicleMinZM = 0.5;
constexpr double kEgoVehicleMaxZM = 4.0;

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
    LOG(WARNING) << "lidar_nn_uri file size is not int32-aligned: "
                 << uri << ", size=" << bytes.size();
    return false;
  }

  labels->resize(bytes.size() / sizeof(std::int32_t));
  if (!bytes.empty()) {
    std::memcpy(labels->data(), bytes.data(), bytes.size());
  }
  return true;
}

FrameData::Cloud::Ptr FilterObjectInstancePointsByLidarNnLabels(
    const FrameData::Cloud::Ptr& raw_cloud,
    const std::vector<std::int32_t>& labels) {
  CHECK(raw_cloud != nullptr);
  CHECK_EQ(raw_cloud->points.size(), labels.size());

  FrameData::Cloud::Ptr filtered_cloud(new FrameData::Cloud);
  *filtered_cloud = *raw_cloud;
  filtered_cloud->points.clear();
  filtered_cloud->points.reserve(raw_cloud->points.size());

  for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
    if (labels[i] > 0) {
      continue;
    }
    filtered_cloud->points.push_back(raw_cloud->points[i]);
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

  if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
    LOG(WARNING) << "frame has empty cloud_uri: " << frame.fid();
    return false;
  }
  FrameData::Cloud::Ptr raw_cloud(new FrameData::Cloud);
  CHECK(local_data_reader->ReadPointCloud(frame.cloud_uri(), raw_cloud))
      << "Fail to load raw cloud at " << frame.cloud_uri();

  lidar_frame_data->raw_cloud = raw_cloud;

  if (frame.has_lidar_nn_uri() && !frame.lidar_nn_uri().empty()) {
    std::vector<std::int32_t> lidar_nn_labels;
    if (ReadInt32Labels(local_data_reader, frame.lidar_nn_uri(), &lidar_nn_labels)) {
      if (lidar_nn_labels.size() == raw_cloud->points.size()) {
        lidar_frame_data->raw_cloud =
            FilterObjectInstancePointsByLidarNnLabels(raw_cloud, lidar_nn_labels);
      } else {
        LOG(WARNING) << "lidar_nn_uri label count does not match cloud point count: "
                     << lidar_nn_labels.size() << " vs " << raw_cloud->points.size()
                     << ", skip lidar nn label filtering, uri=" << frame.lidar_nn_uri();
      }
    }
  }

  pcl::CropBox<PointXYZIRT> crop_box;
  crop_box.setInputCloud(lidar_frame_data->raw_cloud);
  crop_box.setMin(Eigen::Vector4f(kEgoVehicleMinXM, kEgoVehicleMinYM,
                                  kEgoVehicleMinZM, 1.0f));
  crop_box.setMax(Eigen::Vector4f(kEgoVehicleMaxXM, kEgoVehicleMaxYM,
                                  kEgoVehicleMaxZM, 1.0f));
  crop_box.setNegative(true);
  FrameData::Cloud::Ptr filtered_cloud(new FrameData::Cloud);
  crop_box.filter(*filtered_cloud);
  lidar_frame_data->raw_cloud = filtered_cloud;

  return true;
}

}  // namespace mapping
}  // namespace adlabel
