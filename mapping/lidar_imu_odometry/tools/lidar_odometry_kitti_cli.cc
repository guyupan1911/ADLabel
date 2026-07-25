#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

#include "mapping/lidar_imu_odometry/lidar_odometry/map_manager.h"

DEFINE_string(sequence_dir, "", "Path to a KITTI odometry sequence directory.");
DEFINE_string(output_dir, "", "Directory for output artifacts.");
DEFINE_bool(stitch_map, false, "Stitch and save the point cloud map.");
DEFINE_double(map_leaf_size, 0.3, "Map voxel-grid leaf size in meters.");
DEFINE_int32(max_frames, 0,
             "Maximum number of frames to process; 0 means all frames.");

namespace {

constexpr char kTrajectoryFilename[] = "trajectory.txt";
constexpr char kMapFilename[] = "map.pcd";
constexpr std::size_t kMapDownsampleInterval = 50;

struct KittiPoint {
  float x;
  float y;
  float z;
  float intensity;
};

std::vector<std::filesystem::path> ListKittiFrames(
    const std::filesystem::path& sequence_dir) {
  const std::filesystem::path velodyne_dir = sequence_dir / "velodyne";
  CHECK(std::filesystem::is_directory(velodyne_dir))
      << "Velodyne directory does not exist: " << velodyne_dir;

  std::vector<std::filesystem::path> frame_paths;
  for (const auto& entry : std::filesystem::directory_iterator(velodyne_dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".bin") {
      frame_paths.push_back(entry.path());
    }
  }
  std::sort(frame_paths.begin(), frame_paths.end());
  CHECK(!frame_paths.empty()) << "No .bin frames found in: " << velodyne_dir;

  if (FLAGS_max_frames > 0 &&
      frame_paths.size() > static_cast<std::size_t>(FLAGS_max_frames)) {
    frame_paths.resize(FLAGS_max_frames);
  }
  return frame_paths;
}

adlabel::mapping::PointCloudXYZIRT ReadKittiCloud(
    const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  CHECK(input.is_open()) << "Failed to open KITTI point cloud: " << path;

  const std::streamsize file_size = input.tellg();
  CHECK_GT(file_size, 0) << "KITTI point cloud is empty: " << path;
  CHECK_EQ(file_size % static_cast<std::streamsize>(sizeof(KittiPoint)), 0)
      << "Invalid KITTI point cloud file size: " << path;

  const std::size_t num_points =
      static_cast<std::size_t>(file_size) / sizeof(KittiPoint);
  std::vector<KittiPoint> kitti_points(num_points);
  input.seekg(0, std::ios::beg);
  input.read(reinterpret_cast<char*>(kitti_points.data()), file_size);
  CHECK(input) << "Failed to read KITTI point cloud: " << path;

  adlabel::mapping::PointCloudXYZIRT cloud;
  cloud.reserve(num_points);
  for (const KittiPoint& input_point : kitti_points) {
    adlabel::mapping::PointXYZIRT point{};
    point.x = input_point.x;
    point.y = input_point.y;
    point.z = input_point.z;
    point.intensity = input_point.intensity;
    point.ring = 0;
    point.timestamp = 0.0;
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

void WriteKittiPose(const Eigen::Affine3d& pose, std::ostream* output) {
  CHECK(output != nullptr);
  const Eigen::Matrix4d matrix = pose.matrix();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (row != 0 || col != 0) {
        *output << ' ';
      }
      *output << matrix(row, col);
    }
  }
  *output << '\n';
}

void AddCloudToMap(const adlabel::mapping::PointCloudXYZIRT& cloud,
                   const Eigen::Affine3d& T_world_lidar,
                   adlabel::mapping::PointCloudXYZIRT* map_cloud) {
  CHECK(map_cloud != nullptr);
  map_cloud->reserve(map_cloud->size() + cloud.size());
  for (const auto& point : cloud) {
    const Eigen::Vector3d world_point =
        T_world_lidar * Eigen::Vector3d(static_cast<double>(point.x),
                                        static_cast<double>(point.y),
                                        static_cast<double>(point.z));

    adlabel::mapping::PointXYZIRT transformed_point = point;
    transformed_point.x = static_cast<float>(world_point.x());
    transformed_point.y = static_cast<float>(world_point.y());
    transformed_point.z = static_cast<float>(world_point.z());
    map_cloud->push_back(transformed_point);
  }
}

void DownsampleMap(double radius,
                   adlabel::mapping::PointCloudXYZIRT* map_cloud) {
  CHECK(map_cloud != nullptr);
  adlabel::mapping::PointCloudXYZIRT::Ptr input(
      new adlabel::mapping::PointCloudXYZIRT(std::move(*map_cloud)));
  pcl::UniformSampling<adlabel::mapping::PointXYZIRT> uniform_sampling;
  uniform_sampling.setInputCloud(input);
  uniform_sampling.setRadiusSearch(radius);
  uniform_sampling.filter(*map_cloud);
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_sequence_dir.empty()) << "--sequence_dir is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  if (FLAGS_stitch_map) {
    CHECK(std::isfinite(FLAGS_map_leaf_size) && FLAGS_map_leaf_size > 0.0)
        << "--map_leaf_size must be finite and positive";
  }
  CHECK_GE(FLAGS_max_frames, 0) << "--max_frames must not be negative";

  const std::vector<std::filesystem::path> frame_paths =
      ListKittiFrames(FLAGS_sequence_dir);
  const std::filesystem::path output_dir = FLAGS_output_dir;
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "Failed to create output directory " << output_dir << ": "
                << error.message();

  const std::filesystem::path trajectory_path =
      output_dir / kTrajectoryFilename;
  const std::filesystem::path map_path = output_dir / kMapFilename;
  std::ofstream output(trajectory_path);
  CHECK(output.is_open()) << "Failed to open trajectory output: "
                          << trajectory_path;
  output << std::fixed << std::setprecision(9);

  adlabel::mapping::MapManager map_manager;
  adlabel::mapping::PointCloudXYZIRT map_cloud;
  std::chrono::steady_clock::duration odometry_duration{};
  std::chrono::steady_clock::duration stitch_map_duration{};
  for (std::size_t index = 0; index < frame_paths.size(); ++index) {
    const adlabel::mapping::PointCloudXYZIRT cloud =
        ReadKittiCloud(frame_paths[index]);
    adlabel::mapping::SmallGicpRegistrationResult result;

    const auto odometry_start = std::chrono::steady_clock::now();
    const Eigen::Affine3d T_world_lidar =
        map_manager.AlignScanToMap(cloud, result);
    odometry_duration += std::chrono::steady_clock::now() - odometry_start;
    WriteKittiPose(T_world_lidar, &output);

    if (FLAGS_stitch_map) {
      const auto stitch_map_start = std::chrono::steady_clock::now();
      AddCloudToMap(cloud, T_world_lidar, &map_cloud);

      if ((index + 1) % kMapDownsampleInterval == 0 ||
          index + 1 == frame_paths.size()) {
        const std::size_t input_points = map_cloud.size();
        DownsampleMap(FLAGS_map_leaf_size, &map_cloud);
        LOG(INFO) << "Downsampled map after frame " << index + 1 << ": "
                  << input_points << " -> " << map_cloud.size() << " points";
      }
      stitch_map_duration +=
          std::chrono::steady_clock::now() - stitch_map_start;
    }

    if (index == 0) {
      LOG(INFO) << "Frame 0 initialized the odometry reference";
    } else if ((index + 1) % 100 == 0 || index + 1 == frame_paths.size()) {
      LOG(INFO) << "Processed frame " << index + 1 << "/" << frame_paths.size()
                << ": converged=" << result.converged
                << ", iterations=" << result.iterations
                << ", inlier_ratio=" << result.inlier_ratio;
    }
  }

  CHECK(output.good()) << "Failed while writing trajectory: "
                       << trajectory_path;
  LOG(INFO) << "Wrote " << frame_paths.size()
            << " poses to: " << trajectory_path;

  if (FLAGS_stitch_map) {
    const auto stitch_map_start = std::chrono::steady_clock::now();
    CHECK_EQ(pcl::io::savePCDFileBinary(map_path.string(), map_cloud), 0)
        << "Failed to save PCD map: " << map_path;
    stitch_map_duration += std::chrono::steady_clock::now() - stitch_map_start;
    LOG(INFO) << "Wrote " << map_cloud.size() << " map points at "
              << FLAGS_map_leaf_size << " m resolution to: " << map_path;
  }

  const double odometry_ms =
      std::chrono::duration<double, std::milli>(odometry_duration).count();
  LOG(INFO) << "LiDAR odometry took " << odometry_ms << " ms, "
            << odometry_ms / frame_paths.size() << " ms/frame";
  if (FLAGS_stitch_map) {
    const double stitch_map_ms =
        std::chrono::duration<double, std::milli>(stitch_map_duration).count();
    LOG(INFO) << "Map stitching took " << stitch_map_ms << " ms, "
              << stitch_map_ms / frame_paths.size() << " ms/frame";
  }
  return 0;
}
