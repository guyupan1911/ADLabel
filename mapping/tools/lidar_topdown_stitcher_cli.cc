#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata, "", "lidar metadata path, relative to data_root or absolute");
DEFINE_string(data_root, "", "root directory for sensor data");
DEFINE_string(output_dir, "", "directory to save intensity.png");
DEFINE_double(resolution, 0.1, "topdown image resolution, meters per pixel");
DEFINE_double(margin_meters, 10.0, "extra margin around trajectory bounds");
DEFINE_uint64(min_samples, 1, "minimum samples per cell to render");
DEFINE_bool(use_log_intensity, true, "use logarithmic intensity mapping");

namespace adlabel {
namespace mapping {
namespace {

struct TrajectoryBounds {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    size_t valid_poses = 0;

    void Update(const Pose3DMessage& pose) {
        min_x = std::min(min_x, pose.x());
        max_x = std::max(max_x, pose.x());
        min_y = std::min(min_y, pose.y());
        max_y = std::max(max_y, pose.y());
        ++valid_poses;
    }

    bool IsValid() const { return valid_poses > 0; }
    double CenterX() const { return 0.5 * (min_x + max_x); }
    double CenterY() const { return 0.5 * (min_y + max_y); }
    double Width() const { return max_x - min_x; }
    double Height() const { return max_y - min_y; }
};

unsigned char ClampIntensity(float intensity) {
    const float clamped = std::max(0.0f, std::min(255.0f, intensity));
    return static_cast<unsigned char>(clamped);
}

TrajectoryBounds ComputeTrajectoryBounds(const std::vector<Frame>& frames) {
    TrajectoryBounds bounds;
    for (const auto& frame : frames) {
        if (frame.has_lio_pose_3d()) {
            bounds.Update(frame.lio_pose_3d());
        }
    }
    return bounds;
}

GridFrame MakeGridFrame(const TrajectoryBounds& bounds, double resolution, double margin_meters) {
    const double width_m = bounds.Width() + 2.0 * margin_meters;
    const double height_m = bounds.Height() + 2.0 * margin_meters;

    GridFrame frame;
    frame.resolution = resolution;
    frame.cols = static_cast<unsigned int>(std::ceil(width_m / resolution));
    frame.rows = static_cast<unsigned int>(std::ceil(height_m / resolution));
    frame.top_left_corner = {
            bounds.CenterX() - static_cast<double>(frame.cols) * 0.5 * resolution,
            bounds.CenterY() + static_cast<double>(frame.rows) * 0.5 * resolution,
    };
    return frame;
}

bool IsProcessableLidarFrame(const Frame& frame) {
    if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
        LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
        return false;
    }
    if (!frame.has_lio_pose_3d()) {
        LOG(WARNING) << "skip frame without lio_pose_3d: " << frame.fid();
        return false;
    }
    if (!frame.has_sensor_to_imu_extrinsic()) {
        LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: " << frame.fid();
        return false;
    }
    return true;
}

void AccumulateFrameToNode(const Frame& frame,
                           const FrameData& lidar_frame_data,
                           LidarLosslessMapNode* node) {
    CHECK(node != nullptr);
    CHECK(lidar_frame_data.raw_cloud != nullptr);

    const Eigen::Affine3d T_lidar_to_imu = lidar_frame_data.T_sensor_to_imu;
    const Eigen::Affine3d T_world_imu = Pose3D(frame.lio_pose_3d()).GetAffine3D();
    const Eigen::Affine3d T_world_lidar = T_world_imu * T_lidar_to_imu;

    for (const auto& point : lidar_frame_data.raw_cloud->points) {
        const Eigen::Vector3d p_world =
                T_world_lidar * Eigen::Vector3d(point.x, point.y, point.z);
        node->SetValue(p_world, frame.sensor_name(), ClampIntensity(point.intensity));
    }
}

int Run() {
    CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
    CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
    CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
    CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
    CHECK_GE(FLAGS_margin_meters, 0.0) << "--margin_meters must be non-negative";

    auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
    std::vector<Frame> frames = data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
    CHECK(!frames.empty()) << "no frames loaded from " << FLAGS_lidar_metadata;

    const TrajectoryBounds bounds = ComputeTrajectoryBounds(frames);
    CHECK(bounds.IsValid()) << "no valid lio_pose_3d found in frames";

    GridFrame grid_frame = MakeGridFrame(bounds, FLAGS_resolution, FLAGS_margin_meters);
    CHECK_GT(grid_frame.rows, 0u);
    CHECK_GT(grid_frame.cols, 0u);

    LOG(INFO) << "trajectory bounds x=[" << bounds.min_x << ", " << bounds.max_x
              << "] y=[" << bounds.min_y << ", " << bounds.max_y << "]"
              << " valid_poses=" << bounds.valid_poses << "/" << frames.size();
    LOG(INFO) << "topdown center=(" << bounds.CenterX() << ", " << bounds.CenterY()
              << ") image=" << grid_frame.cols << "x" << grid_frame.rows
              << " resolution=" << grid_frame.resolution
              << " top_left=(" << grid_frame.top_left_corner.x()
              << ", " << grid_frame.top_left_corner.y() << ")";

    const auto intensity_mapping = FLAGS_use_log_intensity
            ? LidarLosslessMapNode::IntensityMappingMode::kLogarithmic
            : LidarLosslessMapNode::IntensityMappingMode::kPassThrough;

    LidarLosslessMapNode node;
    node.Init(grid_frame, intensity_mapping);

    size_t processed_frames = 0;
    for (const auto& frame : frames) {
        if (!IsProcessableLidarFrame(frame)) {
            continue;
        }

        FrameData lidar_frame_data;
        if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
            LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
            continue;
        }

        AccumulateFrameToNode(frame, lidar_frame_data, &node);
        ++processed_frames;
        if (processed_frames % 50 == 0) {
            LOG(INFO) << "processed " << processed_frames << " lidar frames";
        }
    }

    CHECK_GT(processed_frames, 0u) << "no lidar frames were processed";

    cv::Mat intensity_image;
    node.GetIntensityImage(&intensity_image, static_cast<size_t>(FLAGS_min_samples));

    const std::filesystem::path output_dir(FLAGS_output_dir);
    std::error_code error;
    std::filesystem::create_directories(output_dir, error);
    CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                  << ", error: " << error.message();

    const std::filesystem::path output_path = output_dir / "intensity.png";
    CHECK(cv::imwrite(output_path.string(), intensity_image))
            << "failed to write image: " << output_path.string();

    LOG(INFO) << "saved intensity image to " << output_path.string()
              << ", processed_frames=" << processed_frames
              << ", occupancy=" << node.GetOccupancyRatio() * 100.0 << "%";
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
