#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/road_layering/road_layering.h"

DEFINE_string(data_root, "", "root directory for lidar data");
DEFINE_string(lidar_metadata, "", "path to lidar frame metadata file");
DEFINE_string(output_dir, "", "directory to save road layering outputs");
DEFINE_bool(output_pcd, false, "stitch and save ramp/non-ramp point clouds");

namespace adlabel {
namespace mapping {
namespace {

constexpr float kVoxelResolutionMeters = 0.3f;
constexpr size_t kProgressLogInterval = 100;

PointCloudXYZIRT::Ptr VoxelGridDownsample(
    const PointCloudXYZIRT::ConstPtr& input_cloud) {
  pcl::VoxelGrid<PointXYZIRT> voxel_grid;
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.setLeafSize(kVoxelResolutionMeters, kVoxelResolutionMeters,
                         kVoxelResolutionMeters);

  PointCloudXYZIRT::Ptr output_cloud(new PointCloudXYZIRT);
  voxel_grid.filter(*output_cloud);
  return output_cloud;
}

void WriteLayerMetadata(const RoadLayering& road_layering,
                        const std::filesystem::path& output_dir) {
  const auto& surfaces = road_layering.road_surfaces();
  CHECK(!surfaces.empty()) << "no road surfaces to write";

  std::unordered_map<std::string, const TripSegment*> segment_by_id;
  for (const auto& trip_and_segments : road_layering.trip_segments()) {
    for (const TripSegment& segment : trip_and_segments.second) {
      segment_by_id[segment.id] = &segment;
    }
  }

  std::unordered_map<std::string, std::size_t> surface_by_segment_id;
  std::vector<std::set<std::string>> frame_ids_by_surface(surfaces.size());
  for (std::size_t surface_index = 0; surface_index < surfaces.size();
       ++surface_index) {
    for (const std::string& segment_id : surfaces[surface_index].segment_ids) {
      const TripSegment& segment = *segment_by_id.at(segment_id);
      surface_by_segment_id[segment_id] = surface_index;
      frame_ids_by_surface[surface_index].insert(segment.frame_ids.begin(),
                                                 segment.frame_ids.end());
    }
  }

  for (const auto& trip_and_segments : road_layering.trip_segments()) {
    const std::vector<TripSegment>& segments = trip_and_segments.second;
    std::size_t index = 0;
    while (index < segments.size()) {
      if (!segments[index].is_ramp) {
        ++index;
        continue;
      }

      const std::size_t ramp_begin = index;
      while (index < segments.size() && segments[index].is_ramp) {
        ++index;
      }
      const std::size_t ramp_end = index;

      const std::size_t invalid_surface = surfaces.size();
      std::size_t previous_surface = invalid_surface;
      std::size_t next_surface = invalid_surface;
      if (ramp_begin > 0) {
        const auto iter =
            surface_by_segment_id.find(segments[ramp_begin - 1].id);
        if (iter != surface_by_segment_id.end()) {
          previous_surface = iter->second;
        }
      }
      if (ramp_end < segments.size()) {
        const auto iter = surface_by_segment_id.find(segments[ramp_end].id);
        if (iter != surface_by_segment_id.end()) {
          next_surface = iter->second;
        }
      }

      if (previous_surface == invalid_surface &&
          next_surface == invalid_surface) {
        LOG(WARNING) << "ramp has no connected surface in trip "
                     << trip_and_segments.first;
        continue;
      }

      for (std::size_t ramp_index = ramp_begin; ramp_index < ramp_end;
           ++ramp_index) {
        for (const std::string& frame_id : segments[ramp_index].frame_ids) {
          if (previous_surface != invalid_surface) {
            frame_ids_by_surface[previous_surface].insert(frame_id);
          }
          if (next_surface != invalid_surface) {
            frame_ids_by_surface[next_surface].insert(frame_id);
          }
        }
      }
    }
  }

  for (std::size_t layer_index = 0; layer_index < surfaces.size();
       ++layer_index) {
    std::vector<Frame> layer_frames;
    layer_frames.reserve(frame_ids_by_surface[layer_index].size());
    for (const std::string& frame_id : frame_ids_by_surface[layer_index]) {
      layer_frames.push_back(road_layering.GetFrame(frame_id));
    }

    const std::filesystem::path output_path =
        output_dir /
        ("lidar_metadata_layer_" + std::to_string(layer_index) + ".meta");
    CHECK(WriteMetaFile(output_path.string(), layer_frames))
        << "failed to write layer metadata: " << output_path.string();
    LOG(INFO) << "saved " << layer_frames.size() << " frames to "
              << output_path.string() << ", representative_height="
              << surfaces[layer_index].representative_height;
  }
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";

  const std::filesystem::path data_root(FLAGS_data_root);
  const std::filesystem::path lidar_metadata_path(FLAGS_lidar_metadata);
  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error: " << error.message();

  LOG(INFO) << "data_root=" << data_root.string()
            << ", lidar_metadata=" << lidar_metadata_path.string()
            << ", output_dir=" << output_dir.string()
            << ", output_pcd=" << FLAGS_output_pcd;

  const std::vector<Frame> frames =
      ReadMetaFile<Frame>(lidar_metadata_path.string());
  CHECK(!frames.empty()) << "no frames loaded from "
                         << lidar_metadata_path.string();

  RoadLayering road_layering;
  for (const Frame& frame : frames) {
    road_layering.AddFrame(frame);
  }

  road_layering.Run();
  WriteLayerMetadata(road_layering, output_dir);

  if (!FLAGS_output_pcd) {
    return 0;
  }

  auto data_reader = std::make_shared<LocalDataReader>(data_root.string());
  PointCloudXYZIRT::Ptr ramp_cloud(new PointCloudXYZIRT);
  PointCloudXYZIRT::Ptr non_ramp_cloud(new PointCloudXYZIRT);
  size_t ramp_frame_count = 0;
  size_t non_ramp_frame_count = 0;
  size_t total_frame_count = 0;
  for (const auto& trip_and_segments : road_layering.trip_segments()) {
    const auto& segments = trip_and_segments.second;
    for (const TripSegment& segment : segments) {
      total_frame_count += segment.frame_ids.size();
    }
  }
  size_t visited_frame_count = 0;

  for (const auto& [trip_id, segments] : road_layering.trip_segments()) {
    LOG(INFO) << "stitch trip " << trip_id << ", segments=" << segments.size();
    for (const TripSegment& segment : segments) {
      CHECK_EQ(segment.frame_ids.size(), segment.frame_poses.size())
          << "frame ids and poses size mismatch in segment " << segment.id;

      for (size_t i = 0; i < segment.frame_ids.size(); ++i) {
        ++visited_frame_count;
        if (visited_frame_count % kProgressLogInterval == 0 ||
            visited_frame_count == total_frame_count) {
          LOG(INFO) << "point cloud stitching progress: " << visited_frame_count
                    << "/" << total_frame_count
                    << ", ramp_points=" << ramp_cloud->size()
                    << ", non_ramp_points=" << non_ramp_cloud->size();
        }

        const Frame& frame = road_layering.GetFrame(segment.frame_ids[i]);
        if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
          LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
          continue;
        }
        if (!frame.has_sensor_to_imu_extrinsic()) {
          LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: "
                       << frame.fid();
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

        const Eigen::Affine3d T_enu_lidar =
            segment.frame_poses[i] *
            lidar_frame_data.transform_from_sensor_to_imu;
        PointCloudXYZIRT::Ptr downsampled_cloud =
            VoxelGridDownsample(lidar_frame_data.raw_cloud);
        PointCloudXYZIRT transformed_cloud;
        pcl::transformPointCloud(*downsampled_cloud, transformed_cloud,
                                 T_enu_lidar.cast<float>());

        if (segment.is_ramp) {
          *ramp_cloud += transformed_cloud;
          ++ramp_frame_count;
        } else {
          *non_ramp_cloud += transformed_cloud;
          ++non_ramp_frame_count;
        }
      }
    }
  }

  ramp_cloud = VoxelGridDownsample(ramp_cloud);
  non_ramp_cloud = VoxelGridDownsample(non_ramp_cloud);

  const std::filesystem::path ramp_output_path = output_dir / "ramp.pcd";
  const std::filesystem::path non_ramp_output_path =
      output_dir / "non_ramp.pcd";
  CHECK_EQ(pcl::io::savePCDFileBinary(ramp_output_path.string(), *ramp_cloud),
           0)
      << "failed to save ramp cloud: " << ramp_output_path.string();
  CHECK_EQ(pcl::io::savePCDFileBinary(non_ramp_output_path.string(),
                                      *non_ramp_cloud),
           0)
      << "failed to save non-ramp cloud: " << non_ramp_output_path.string();

  LOG(INFO) << "saved ramp cloud: " << ramp_output_path.string()
            << ", frames=" << ramp_frame_count
            << ", points=" << ramp_cloud->size();
  LOG(INFO) << "saved non-ramp cloud: " << non_ramp_output_path.string()
            << ", frames=" << non_ramp_frame_count
            << ", points=" << non_ramp_cloud->size();

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
