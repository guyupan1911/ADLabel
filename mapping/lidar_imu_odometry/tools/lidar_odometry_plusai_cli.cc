#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_imu_odometry/lidar_odometry/map_manager.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "",
              "Root directory used to resolve point cloud URIs.");
DEFINE_string(lidar_metadata, "", "Path to lidar frame metadata.");
DEFINE_string(output_dir, "", "Directory for lidar odometry results.");
DEFINE_bool(stitch_map, false, "Stitch and save the point cloud map.");
DEFINE_int32(max_frames, 0,
             "Maximum number of valid frames to process; 0 means all frames.");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kTrajectoryFilename[] = "trajectory.txt";
constexpr char kTimestampsFilename[] = "timestamps.txt";
constexpr char kMapFilename[] = "map.pcd";
constexpr double kMapDownsampleResolution = 0.1;
constexpr std::size_t kMapDownsampleInterval = 50;

struct FramePose {
  Frame frame;
  Eigen::Affine3d T_world_lidar = Eigen::Affine3d::Identity();
};

void WritePose(const Eigen::Affine3d& pose, std::ostream* output) {
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

bool IsProcessable(const Frame& frame) {
  if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
    LOG(WARNING) << "Skip frame without cloud_uri: " << frame.fid();
    return false;
  }
  if (!frame.has_sensor_to_imu_extrinsic()) {
    LOG(WARNING) << "Skip frame without sensor_to_imu_extrinsic: "
                 << frame.fid();
    return false;
  }
  return true;
}

void AddCloudToMap(const PointCloudXYZIRT& cloud,
                   const Eigen::Affine3d& T_world_lidar,
                   PointCloudXYZIRT* map_cloud) {
  CHECK(map_cloud != nullptr);
  map_cloud->reserve(map_cloud->size() + cloud.size());
  for (const auto& point : cloud) {
    const Eigen::Vector3d world_point =
        T_world_lidar * Eigen::Vector3d(static_cast<double>(point.x),
                                        static_cast<double>(point.y),
                                        static_cast<double>(point.z));

    PointXYZIRT transformed_point = point;
    transformed_point.x = static_cast<float>(world_point.x());
    transformed_point.y = static_cast<float>(world_point.y());
    transformed_point.z = static_cast<float>(world_point.z());
    map_cloud->push_back(transformed_point);
  }
}

void DownsampleMap(PointCloudXYZIRT* map_cloud) {
  CHECK(map_cloud != nullptr);
  PointCloudXYZIRT::Ptr input(new PointCloudXYZIRT(std::move(*map_cloud)));
  pcl::UniformSampling<PointXYZIRT> uniform_sampling;
  uniform_sampling.setInputCloud(input);
  uniform_sampling.setRadiusSearch(kMapDownsampleResolution);
  uniform_sampling.filter(*map_cloud);
}

void SaveFramesMetadata(const std::filesystem::path& metadata_path,
                        const std::vector<Frame>& frames) {
  std::filesystem::path temporary_path = metadata_path;
  temporary_path += ".tmp";
  CHECK(WriteMetaFile(temporary_path.string(), frames))
      << "Failed to write temporary metadata: " << temporary_path;

  std::error_code error;
  std::filesystem::rename(temporary_path, metadata_path, error);
  CHECK(!error) << "Failed to replace metadata " << metadata_path << ": "
                << error.message();
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK_GE(FLAGS_max_frames, 0) << "--max_frames must not be negative";

  const std::filesystem::path data_root = FLAGS_data_root;
  const std::filesystem::path lidar_metadata = FLAGS_lidar_metadata;
  const std::filesystem::path output_dir = FLAGS_output_dir;
  CHECK(std::filesystem::is_directory(data_root))
      << "data_root does not exist: " << data_root;
  CHECK(std::filesystem::is_regular_file(lidar_metadata))
      << "lidar_metadata does not exist: " << lidar_metadata;

  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "Failed to create output directory " << output_dir << ": "
                << error.message();

  std::vector<Frame> frames = ReadMetaFile<Frame>(lidar_metadata.string());
  CHECK(!frames.empty()) << "No frames loaded from: " << lidar_metadata;
  std::sort(frames.begin(), frames.end(),
            [](const Frame& lhs, const Frame& rhs) {
              return lhs.timestamp_ns() < rhs.timestamp_ns();
            });

  const auto data_reader =
      std::make_shared<LocalDataReader>(data_root.string());
  const std::filesystem::path trajectory_path =
      output_dir / kTrajectoryFilename;
  const std::filesystem::path timestamps_path =
      output_dir / kTimestampsFilename;
  const std::filesystem::path map_path = output_dir / kMapFilename;
  std::ofstream trajectory_output(trajectory_path);
  CHECK(trajectory_output.is_open())
      << "Failed to open trajectory output: " << trajectory_path;
  std::ofstream timestamps_output(timestamps_path);
  CHECK(timestamps_output.is_open())
      << "Failed to open timestamp output: " << timestamps_path;
  trajectory_output << std::fixed << std::setprecision(9);

  MapManager map_manager;
  std::size_t processed_frames = 0;
  std::size_t skipped_frames = 0;
  std::vector<FramePose> frame_poses;
  frame_poses.reserve(frames.size());
  const auto odometry_start = std::chrono::steady_clock::now();
  for (Frame& frame : frames) {
    if (FLAGS_max_frames > 0 &&
        processed_frames >= static_cast<std::size_t>(FLAGS_max_frames)) {
      break;
    }
    if (!IsProcessable(frame)) {
      ++skipped_frames;
      continue;
    }

    FrameData frame_data;
    if (!GenerateLidarFrameData(frame, &frame_data, data_reader) ||
        frame_data.raw_cloud == nullptr || frame_data.raw_cloud->empty()) {
      LOG(WARNING) << "Skip frame that failed to load: " << frame.fid();
      ++skipped_frames;
      continue;
    }

    SmallGicpRegistrationResult result;
    const Eigen::Affine3d T_world_lidar =
        map_manager.AlignScanToMap(*frame_data.raw_cloud, result);
    const Eigen::Affine3d T_world_imu =
        T_world_lidar * frame_data.transform_from_sensor_to_imu.inverse();
    *frame.mutable_lio_pose_3d() = Pose3D(T_world_imu).GetPose3DMessage();

    WritePose(T_world_lidar, &trajectory_output);
    timestamps_output << frame.timestamp_ns() << '\n';
    frame_poses.push_back({frame, T_world_lidar});
    ++processed_frames;

    if (processed_frames == 1) {
      LOG(INFO) << "Initialized odometry with frame: " << frame.fid();
    } else if (processed_frames % 100 == 0) {
      LOG(INFO) << "Processed " << processed_frames
                << " frames: converged=" << result.converged
                << ", iterations=" << result.iterations
                << ", inlier_ratio=" << result.inlier_ratio;
    }
  }
  const double odometry_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    odometry_start)
          .count();

  CHECK_GT(processed_frames, static_cast<std::size_t>(0))
      << "No valid lidar frames were processed";
  CHECK(trajectory_output.good())
      << "Failed while writing trajectory: " << trajectory_path;
  CHECK(timestamps_output.good())
      << "Failed while writing timestamps: " << timestamps_path;

  LOG(INFO) << "Processed " << processed_frames << " frames, skipped "
            << skipped_frames;
  LOG(INFO) << "LiDAR odometry took " << odometry_seconds << " s";
  LOG(INFO) << "Wrote trajectory to: " << trajectory_path;
  LOG(INFO) << "Wrote timestamps to: " << timestamps_path;
  SaveFramesMetadata(lidar_metadata, frames);
  LOG(INFO) << "Updated lio_pose_3d for " << processed_frames
            << " frames in: " << lidar_metadata;

  if (FLAGS_stitch_map) {
    const auto stitch_map_start = std::chrono::steady_clock::now();
    PointCloudXYZIRT map_cloud;
    for (std::size_t index = 0; index < frame_poses.size(); ++index) {
      const FramePose& frame_pose = frame_poses[index];
      FrameData frame_data;
      CHECK(GenerateLidarFrameData(frame_pose.frame, &frame_data, data_reader))
          << "Failed to reload frame for map stitching: "
          << frame_pose.frame.fid();
      CHECK(frame_data.raw_cloud != nullptr && !frame_data.raw_cloud->empty())
          << "Empty point cloud while stitching frame: "
          << frame_pose.frame.fid();

      AddCloudToMap(*frame_data.raw_cloud, frame_pose.T_world_lidar,
                    &map_cloud);
      if ((index + 1) % kMapDownsampleInterval == 0) {
        const std::size_t input_points = map_cloud.size();
        DownsampleMap(&map_cloud);
        LOG(INFO) << "Downsampled map after " << index + 1
                  << " frames: " << input_points << " -> " << map_cloud.size()
                  << " points";
      }
    }

    const std::size_t input_points = map_cloud.size();
    DownsampleMap(&map_cloud);
    CHECK_EQ(pcl::io::savePCDFileBinary(map_path.string(), map_cloud), 0)
        << "Failed to save map: " << map_path;

    const double stitch_map_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      stitch_map_start)
            .count();
    LOG(INFO) << "Final map downsampling: " << input_points << " -> "
              << map_cloud.size() << " points";
    LOG(INFO) << "Map stitching took " << stitch_map_seconds << " s";
    LOG(INFO) << "Wrote map at " << kMapDownsampleResolution
              << " m resolution to: " << map_path;
  }
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
