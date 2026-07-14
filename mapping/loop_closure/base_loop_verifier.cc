#include "mapping/loop_closure/base_loop_verifier.h"

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <GeographicLib/Geocentric.hpp>

#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"

namespace adlabel {
namespace mapping {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTopdownResolution = 0.1;
constexpr unsigned int kTopdownImageSize = 1600;
constexpr char kLocalMapLidarName[] = "local_map";

cv::Mat GenerateTopdownImageFromCloud(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) {
  CHECK(cloud != nullptr);
  CHECK(!cloud->points.empty()) << "cannot generate topdown image from empty cloud";

  GridFrame grid_frame;
  grid_frame.resolution = kTopdownResolution;
  grid_frame.cols = kTopdownImageSize;
  grid_frame.rows = kTopdownImageSize;
  grid_frame.top_left_corner = {
      -static_cast<double>(grid_frame.cols) * 0.5 * kTopdownResolution,
      static_cast<double>(grid_frame.rows) * 0.5 * kTopdownResolution,
  };

  LidarLosslessMapNode node;
  node.Init(grid_frame,
            LidarLosslessMapNode::IntensityMappingMode::kLogarithmic,
            LidarLosslessMapNode::IntensityAggregationMode::kMean);

  for (const auto& point : cloud->points) {
    const float clamped_intensity =
        std::max(0.0f, std::min(255.0f, point.intensity));
    node.SetValue(Eigen::Vector3d(point.x, point.y, point.z),
                  kLocalMapLidarName,
                  static_cast<unsigned char>(clamped_intensity));
  }

  cv::Mat image;
  node.GetIntensityImage(&image, 1);
  return image;
}

}  // namespace

void BaseLoopVerifier::RefineFramePairRelativePose(const FramePair& frame_pair) {
  from_local_map_ = StitchLocalMap(
      frame_pair.from_frame(), frame_pair.from_local_frames(),
      frame_pair.from_relative_poses());

  to_local_map_ = StitchLocalMap(
      frame_pair.to_frame(), frame_pair.to_local_frames(),
      frame_pair.to_relative_poses());
}

void BaseLoopVerifier::GenerateFramePairTopdownImages(
    cv::Mat* from_image, cv::Mat* to_image) {
  CHECK(from_image != nullptr);
  CHECK(to_image != nullptr);
  CHECK(from_local_map_ != nullptr) << "from_local_map_ has not been stitched";
  CHECK(to_local_map_ != nullptr) << "to_local_map_ has not been stitched";

  *from_image = GenerateTopdownImageFromCloud(from_local_map_);

  *to_image = GenerateTopdownImageFromCloud(to_local_map_);
}

pcl::PointCloud<PointXYZIRT>::Ptr BaseLoopVerifier::StitchLocalMap(
    const Frame& reference_frame,
    const google::protobuf::RepeatedPtrField<Frame>& frames,
    const google::protobuf::RepeatedPtrField<Pose3DMessage>& relative_poses) {
  CHECK(local_data_reader_ != nullptr);
  CHECK_EQ(frames.size(), relative_poses.size())
      << "local frame count must match relative pose count";
  LOG(INFO) << "frames size: " << frames.size();
  
  const Eigen::Affine3d T_ecef_reference =
      Pose3D(reference_frame.refined_pose_3d()).GetAffine3D();
  const Eigen::Vector3d reference_ecef = T_ecef_reference.translation();
  double reference_lat = 0.0;
  double reference_lon = 0.0;
  double reference_height = 0.0;
  GeographicLib::Geocentric::WGS84().Reverse(
      reference_ecef.x(), reference_ecef.y(), reference_ecef.z(),
      reference_lat, reference_lon, reference_height);
  const double reference_lat_rad = reference_lat * kPi / 180.0;
  const double reference_lon_rad = reference_lon * kPi / 180.0;
  const double sin_lat = std::sin(reference_lat_rad);
  const double cos_lat = std::cos(reference_lat_rad);
  const double sin_lon = std::sin(reference_lon_rad);
  const double cos_lon = std::cos(reference_lon_rad);
  Eigen::Matrix3d R_enu_ecef;
  R_enu_ecef << -sin_lon, cos_lon, 0.0,
      -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
      cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;

  pcl::PointCloud<PointXYZIRT>::Ptr local_map(new pcl::PointCloud<PointXYZIRT>);

  auto append_frame_cloud = [&](const Frame& frame,
                                const Eigen::Affine3d& T_reference_current) {
    FrameData frame_data;
    if (!GenerateLidarFrameData(frame, &frame_data, local_data_reader_)) {
      LOG(WARNING) << "failed to generate lidar frame data: " << frame.fid();
      return;
    }

    const Eigen::Affine3d T_ecef_lidar =
        T_ecef_reference * T_reference_current *
        frame_data.transform_from_sensor_to_imu;
    local_map->points.reserve(local_map->points.size() +
                              frame_data.raw_cloud->points.size());
    for (const auto& point : frame_data.raw_cloud->points) {
      PointXYZIRT transformed_point = point;
      const Eigen::Vector3d p_ecef =
          T_ecef_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      const Eigen::Vector3d p_enu = R_enu_ecef * (p_ecef - reference_ecef);
      transformed_point.x = static_cast<float>(p_enu.x());
      transformed_point.y = static_cast<float>(p_enu.y());
      transformed_point.z = static_cast<float>(p_enu.z());
      local_map->points.push_back(transformed_point);
    }
  };

  append_frame_cloud(reference_frame, Eigen::Affine3d::Identity());
  for (int i = 0; i < frames.size(); ++i) {
    append_frame_cloud(frames.Get(i), Pose3D(relative_poses.Get(i)).GetAffine3D());
  }

  local_map->width = static_cast<std::uint32_t>(local_map->points.size());
  local_map->height = 1;
  local_map->is_dense = false;
  return local_map;
}

}
}