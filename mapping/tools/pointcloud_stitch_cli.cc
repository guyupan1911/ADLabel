#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for LocalDataReader");
DEFINE_string(lidar_metadata, "",
              "comma-separated lidar metadata paths, relative to data_root or "
              "absolute");
DEFINE_string(output_root, "", "directory to save stitched point clouds");
DEFINE_double(distance, 0.0,
              "trajectory split distance in meters; 0 writes one cloud.pcd");
DEFINE_double(voxel_resolution, 0.3,
              "voxel-grid resolution in meters for both individual frames "
              "and the stitched map");
DEFINE_int32(map_downsample_interval, 50,
             "downsample the accumulated map after this many added frames");
DEFINE_bool(use_lio, false,
            "use lio_pose_3d/FrameData::pose_utm instead of "
            "refined_pose_3d/FrameData::pose_ecef");

namespace adlabel {
namespace mapping {
namespace {

std::string Trim(std::string value) {
  const auto is_space = [](unsigned char ch) { return std::isspace(ch); };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(),
                           [&](char ch) { return !is_space(ch); }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
                           [&](char ch) { return !is_space(ch); })
                  .base(),
              value.end());
  return value;
}

std::vector<std::filesystem::path> ParseMetadataPaths(
    const std::filesystem::path& data_root, const std::string& lidar_metadata) {
  std::vector<std::filesystem::path> paths;
  std::stringstream stream(lidar_metadata);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (item.empty()) {
      continue;
    }
    std::filesystem::path path(item);
    if (!path.is_absolute() && !std::filesystem::exists(path)) {
      path = data_root / path;
    }
    paths.push_back(std::move(path));
  }
  return paths;
}

Eigen::Matrix3d EcefToEnuRotation(double lat_deg, double lon_deg) {
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  Eigen::Matrix3d rotation;
  rotation << -sin_lon, cos_lon, 0.0, -sin_lat * cos_lon, -sin_lat * sin_lon,
      cos_lat, cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  return rotation;
}

Eigen::Affine3d BuildEcefToEnuTransform(const Eigen::Vector3d& origin_ecef) {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double height_m = 0.0;
  GeographicLib::Geocentric::WGS84().Reverse(origin_ecef.x(), origin_ecef.y(),
                                             origin_ecef.z(), lat_deg, lon_deg,
                                             height_m);

  const Eigen::Matrix3d rotation = EcefToEnuRotation(lat_deg, lon_deg);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = rotation;
  transform.translation() = -rotation * origin_ecef;
  LOG(INFO) << "ENU origin: latitude=" << lat_deg << ", longitude=" << lon_deg
            << ", height=" << height_m << ", ecef=" << origin_ecef.transpose();
  return transform;
}

double PoseDistance2D(const Eigen::Affine3d& left,
                      const Eigen::Affine3d& right) {
  const Eigen::Vector3d delta = right.translation() - left.translation();
  return std::hypot(delta.x(), delta.y());
}

std::string SegmentCloudFileName(size_t segment_index) {
  std::ostringstream stream;
  stream << "cloud_" << std::setw(3) << std::setfill('0') << segment_index
         << ".pcd";
  return stream.str();
}

PointCloudXYZIRT::Ptr VoxelGridDownsample(
    const PointCloudXYZIRT::ConstPtr& input_cloud, double resolution) {
  CHECK(input_cloud != nullptr);
  const float leaf_size = static_cast<float>(resolution);
  pcl::VoxelGrid<PointXYZIRT> voxel_grid;
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

  PointCloudXYZIRT::Ptr output_cloud(new PointCloudXYZIRT);
  voxel_grid.filter(*output_cloud);
  return output_cloud;
}

void DownsampleMap(PointCloudXYZIRT* map_cloud, double resolution) {
  CHECK(map_cloud != nullptr);
  if (map_cloud->empty()) {
    return;
  }

  PointCloudXYZIRT::Ptr input_cloud(new PointCloudXYZIRT);
  input_cloud->swap(*map_cloud);
  PointCloudXYZIRT::Ptr output_cloud =
      VoxelGridDownsample(input_cloud, resolution);
  map_cloud->swap(*output_cloud);
}

bool SaveStitchedCloud(const PointCloudXYZIRT::ConstPtr& stitched_cloud,
                       size_t loaded_frame_count,
                       const std::filesystem::path& output_path) {
  if (stitched_cloud == nullptr || stitched_cloud->empty()) {
    LOG(ERROR) << "cannot save empty stitched cloud: " << output_path.string();
    return false;
  }
  if (pcl::io::savePCDFileBinary(output_path.string(), *stitched_cloud) < 0) {
    LOG(ERROR) << "failed to save stitched cloud: " << output_path.string();
    return false;
  }
  LOG(INFO) << "stitched " << loaded_frame_count << " frames, saved "
            << stitched_cloud->size() << " points to " << output_path.string();
  return true;
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_output_root.empty()) << "--output_root is required";
  CHECK_GE(FLAGS_distance, 0.0) << "--distance must be non-negative";
  CHECK(std::isfinite(FLAGS_voxel_resolution) && FLAGS_voxel_resolution > 0.0)
      << "--voxel_resolution must be finite and positive";
  CHECK_GT(FLAGS_map_downsample_interval, 0)
      << "--map_downsample_interval must be positive";

  const std::filesystem::path data_root(FLAGS_data_root);
  CHECK(std::filesystem::is_directory(data_root))
      << "data_root does not exist: " << data_root.string();
  const std::vector<std::filesystem::path> metadata_paths =
      ParseMetadataPaths(data_root, FLAGS_lidar_metadata);
  CHECK(!metadata_paths.empty()) << "--lidar_metadata has no valid entries";

  std::vector<Frame> lidar_frames;
  for (const auto& metadata_path : metadata_paths) {
    CHECK(std::filesystem::is_regular_file(metadata_path))
        << "lidar_metadata does not exist: " << metadata_path.string();
    std::vector<Frame> frames = ReadMetaFile<Frame>(metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from "
                           << metadata_path.string();
    LOG(INFO) << "loaded " << frames.size() << " frames from "
              << metadata_path.string();
    lidar_frames.insert(lidar_frames.end(),
                        std::make_move_iterator(frames.begin()),
                        std::make_move_iterator(frames.end()));
  }
  std::sort(lidar_frames.begin(), lidar_frames.end(),
            [](const Frame& lhs, const Frame& rhs) {
              return lhs.timestamp_ns() < rhs.timestamp_ns();
            });
  CHECK(!lidar_frames.empty()) << "no lidar frames loaded";

  const std::filesystem::path output_root(FLAGS_output_root);
  std::error_code error;
  std::filesystem::create_directories(output_root, error);
  CHECK(!error) << "failed to create output_root: " << output_root.string()
                << ", error: " << error.message();

  auto data_reader = std::make_shared<LocalDataReader>(data_root.string());
  const bool split_by_distance = FLAGS_distance > 0.0;
  PointCloudXYZIRT::Ptr stitched_cloud(new PointCloudXYZIRT);
  size_t loaded_frame_count = 0;
  size_t total_loaded_frame_count = 0;
  size_t saved_segment_count = 0;
  double current_segment_distance = 0.0;
  bool map_needs_downsampling = false;

  Eigen::Affine3d world_to_local = Eigen::Affine3d::Identity();
  bool has_local_origin = false;
  Eigen::Affine3d last_local_imu_pose = Eigen::Affine3d::Identity();
  bool has_last_local_imu_pose = false;

  for (const Frame& frame : lidar_frames) {
    if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
      LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
      continue;
    }
    if (!frame.has_sensor_to_imu_extrinsic()) {
      LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: "
                   << frame.fid();
      continue;
    }
    if (FLAGS_use_lio && !frame.has_lio_pose_3d()) {
      LOG(WARNING) << "skip frame without lio_pose_3d: " << frame.fid();
      continue;
    }
    if (!FLAGS_use_lio && !frame.has_refined_pose_3d()) {
      LOG(WARNING) << "skip frame without refined_pose_3d: " << frame.fid();
      continue;
    }

    FrameData lidar_frame_data;
    if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
      LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
      continue;
    }
    if (lidar_frame_data.raw_cloud == nullptr ||
        lidar_frame_data.raw_cloud->empty()) {
      LOG(WARNING) << "skip frame without raw cloud: " << frame.fid();
      continue;
    }

    const Eigen::Affine3d& world_imu_pose =
        FLAGS_use_lio ? lidar_frame_data.pose_utm : lidar_frame_data.pose_ecef;
    if (!has_local_origin) {
      world_to_local =
          FLAGS_use_lio ? world_imu_pose.inverse()
                        : BuildEcefToEnuTransform(world_imu_pose.translation());
      has_local_origin = true;
      LOG(INFO) << "use frame " << frame.fid()
                << " as local origin, pose_source="
                << (FLAGS_use_lio ? "lio" : "ecef_to_enu");
    }

    const Eigen::Affine3d local_imu_pose = world_to_local * world_imu_pose;
    if (split_by_distance && has_last_local_imu_pose &&
        loaded_frame_count > 0) {
      const double frame_distance =
          PoseDistance2D(last_local_imu_pose, local_imu_pose);
      if (current_segment_distance + frame_distance > FLAGS_distance) {
        if (map_needs_downsampling) {
          const size_t points_before = stitched_cloud->size();
          DownsampleMap(stitched_cloud.get(), FLAGS_voxel_resolution);
          LOG(INFO) << "final map downsampling for segment "
                    << saved_segment_count << ": " << points_before << " -> "
                    << stitched_cloud->size()
                    << " points, resolution=" << FLAGS_voxel_resolution << " m";
          map_needs_downsampling = false;
        }
        const std::filesystem::path output_path =
            output_root / SegmentCloudFileName(saved_segment_count);
        if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count,
                               output_path)) {
          return 1;
        }
        ++saved_segment_count;
        stitched_cloud.reset(new PointCloudXYZIRT);
        loaded_frame_count = 0;
        current_segment_distance = 0.0;
      } else {
        current_segment_distance += frame_distance;
      }
    }

    const Eigen::Affine3d local_lidar_pose =
        local_imu_pose * lidar_frame_data.transform_from_sensor_to_imu;
    PointCloudXYZIRT::Ptr downsampled_frame =
        VoxelGridDownsample(lidar_frame_data.raw_cloud, FLAGS_voxel_resolution);
    PointCloudXYZIRT::Ptr transformed(new PointCloudXYZIRT);
    pcl::transformPointCloud(*downsampled_frame, *transformed,
                             local_lidar_pose.cast<float>());
    *stitched_cloud += *transformed;
    ++loaded_frame_count;
    ++total_loaded_frame_count;
    map_needs_downsampling = true;

    if (loaded_frame_count %
            static_cast<size_t>(FLAGS_map_downsample_interval) ==
        0) {
      const size_t points_before = stitched_cloud->size();
      DownsampleMap(stitched_cloud.get(), FLAGS_voxel_resolution);
      LOG(INFO) << "periodic map downsampling after " << loaded_frame_count
                << " frames in current segment: " << points_before << " -> "
                << stitched_cloud->size()
                << " points, resolution=" << FLAGS_voxel_resolution << " m";
      map_needs_downsampling = false;
    }
    last_local_imu_pose = local_imu_pose;
    has_last_local_imu_pose = true;

    if (total_loaded_frame_count % 50 == 0) {
      LOG(INFO) << "processed " << total_loaded_frame_count
                << " frames, stitched points in current output="
                << stitched_cloud->size();
    }
  }

  CHECK_GT(total_loaded_frame_count, 0u)
      << "no processable lidar frames were stitched";
  if (map_needs_downsampling) {
    const size_t points_before = stitched_cloud->size();
    DownsampleMap(stitched_cloud.get(), FLAGS_voxel_resolution);
    LOG(INFO) << "final map downsampling for segment " << saved_segment_count
              << ": " << points_before << " -> " << stitched_cloud->size()
              << " points, resolution=" << FLAGS_voxel_resolution << " m";
  }
  const std::filesystem::path final_output_path =
      split_by_distance
          ? output_root / SegmentCloudFileName(saved_segment_count)
          : output_root / "cloud.pcd";
  if (!SaveStitchedCloud(stitched_cloud, loaded_frame_count,
                         final_output_path)) {
    return 1;
  }
  if (split_by_distance) {
    ++saved_segment_count;
  }
  LOG(INFO) << "finished point cloud stitching: metadata_files="
            << metadata_paths.size() << ", frames=" << total_loaded_frame_count
            << ", outputs=" << (split_by_distance ? saved_segment_count : 1)
            << ", voxel_resolution=" << FLAGS_voxel_resolution
            << ", map_downsample_interval=" << FLAGS_map_downsample_interval
            << ", coordinate_frame="
            << (FLAGS_use_lio ? "first_lio_frame" : "ENU_at_first_frame");
  return 0;
}

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return adlabel::mapping::Run();
}
