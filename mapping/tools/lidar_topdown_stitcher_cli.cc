#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
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

DEFINE_string(data_root, "", "bag dump root directory");
DEFINE_string(output_dir, "", "directory to save topdown intensity image");
DEFINE_double(resolution, 0.05, "topdown image resolution, meters per pixel");
DEFINE_double(margin_meters, 50.0, "extra margin around trajectory bounds");
DEFINE_double(distance, 0.0,
              "trajectory split distance in meters, 0 disables splitting");
DEFINE_string(aggregation_mode, "mean",
              "intensity aggregation mode for stitching: mean or max");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kLidarMetadataPath[] = "metadata/lidar/lidar_plusai_unified.meta";

struct LocalFramePose {
    Frame frame;
    Eigen::Vector2d origin_xy{0.0, 0.0};
};

LidarLosslessMapNode::IntensityAggregationMode ParseAggregationMode(
        const std::string& mode) {
    if (mode == "mean") {
        return LidarLosslessMapNode::IntensityAggregationMode::kMean;
    }
    if (mode == "max") {
        return LidarLosslessMapNode::IntensityAggregationMode::kMax;
    }
    LOG(FATAL) << "unsupported --aggregation_mode=" << mode
               << ", expected mean or max";
    return LidarLosslessMapNode::IntensityAggregationMode::kMean;
}

int Run() {
    CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
    CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
    CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
    CHECK_GE(FLAGS_margin_meters, 0.0) << "--margin_meters must be non-negative";
    CHECK_GE(FLAGS_distance, 0.0) << "--distance must be non-negative";
    const LidarLosslessMapNode::IntensityAggregationMode aggregation_mode =
            ParseAggregationMode(FLAGS_aggregation_mode);

    const std::filesystem::path data_root(FLAGS_data_root);
    const std::filesystem::path output_dir(FLAGS_output_dir);
    std::error_code error;
    std::filesystem::create_directories(output_dir, error);
    CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                  << ", error: " << error.message();

    const std::filesystem::path lidar_metadata_path = data_root / kLidarMetadataPath;
    const std::filesystem::path data_reader_root = data_root.parent_path();
    auto data_reader = std::make_shared<LocalDataReader>(data_reader_root.string());
    LOG(INFO) << "data_root=" << data_root.string()
              << ", local_data_reader_root=" << data_reader_root.string();

    const std::vector<Frame> frames = ReadMetaFile<Frame>(lidar_metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from " << lidar_metadata_path.string();

    Eigen::Affine3d origin_pose_ecef = Eigen::Affine3d::Identity();
    std::vector<LocalFramePose> local_frames;
    for (const auto& frame : frames) {
        if (!frame.has_cloud_uri() || frame.cloud_uri().empty()) {
            LOG(WARNING) << "skip frame without cloud_uri: " << frame.fid();
            continue;
        }
        if (!frame.has_refined_pose_3d()) {
            LOG(WARNING) << "skip frame without refined_pose_3d: " << frame.fid();
            continue;
        }
        if (!frame.has_sensor_to_imu_extrinsic()) {
            LOG(WARNING) << "skip frame without sensor_to_imu_extrinsic: " << frame.fid();
            continue;
        }

        const Eigen::Affine3d pose_ecef = Pose3D(frame.refined_pose_3d()).GetAffine3D();
        if (local_frames.empty()) {
            origin_pose_ecef = pose_ecef;
        }

        LocalFramePose local_frame;
        local_frame.frame = frame;
        local_frame.origin_xy =
                (origin_pose_ecef.inverse() * pose_ecef).translation().head<2>();
        local_frames.push_back(local_frame);
    }
    CHECK(!local_frames.empty()) << "no processable lidar frames in "
                                 << lidar_metadata_path.string();

    std::vector<std::vector<size_t>> frame_segments;
    std::vector<size_t> current_segment;
    double current_distance_m = 0.0;
    for (size_t i = 0; i < local_frames.size(); ++i) {
        if (FLAGS_distance > 0.0 && !current_segment.empty()) {
            const Eigen::Vector2d& last_xy = local_frames[current_segment.back()].origin_xy;
            const double step_distance = (local_frames[i].origin_xy - last_xy).norm();
            if (current_distance_m + step_distance > FLAGS_distance) {
                frame_segments.push_back(current_segment);
                current_segment.clear();
                current_distance_m = 0.0;
            } else {
                current_distance_m += step_distance;
            }
        }
        current_segment.push_back(i);
    }
    if (!current_segment.empty()) {
        frame_segments.push_back(current_segment);
    }

    LOG(INFO) << "loaded " << frames.size() << " lidar frames, processable="
              << local_frames.size() << ", segments=" << frame_segments.size()
              << ", origin timestamp=" << local_frames.front().frame.timestamp_ns();

    const Eigen::Affine3d T_origin_ecef = origin_pose_ecef.inverse();
    for (size_t segment_index = 0; segment_index < frame_segments.size(); ++segment_index) {
        const auto& segment = frame_segments[segment_index];
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const size_t frame_index : segment) {
            const Eigen::Vector2d& xy = local_frames[frame_index].origin_xy;
            min_x = std::min(min_x, xy.x());
            max_x = std::max(max_x, xy.x());
            min_y = std::min(min_y, xy.y());
            max_y = std::max(max_y, xy.y());
        }

        GridFrame grid_frame;
        grid_frame.resolution = FLAGS_resolution;
        grid_frame.cols = std::max(
                1u,
                static_cast<unsigned int>(
                        std::ceil((max_x - min_x + 2.0 * FLAGS_margin_meters) /
                                  FLAGS_resolution)));
        grid_frame.rows = std::max(
                1u,
                static_cast<unsigned int>(
                        std::ceil((max_y - min_y + 2.0 * FLAGS_margin_meters) /
                                  FLAGS_resolution)));
        grid_frame.top_left_corner = {
                0.5 * (min_x + max_x) - static_cast<double>(grid_frame.cols) *
                                              0.5 * FLAGS_resolution,
                0.5 * (min_y + max_y) + static_cast<double>(grid_frame.rows) *
                                              0.5 * FLAGS_resolution,
        };

        LidarLosslessMapNode node;
        node.Init(grid_frame,
                  LidarLosslessMapNode::IntensityMappingMode::kLogarithmic,
                  aggregation_mode);

        size_t processed_frames = 0;
        for (const size_t frame_index : segment) {
            const Frame& frame = local_frames[frame_index].frame;
            FrameData lidar_frame_data;
            if (!GenerateLidarFrameData(frame, &lidar_frame_data, data_reader)) {
                LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
                continue;
            }

            const Eigen::Affine3d T_origin_lidar =
                    T_origin_ecef * lidar_frame_data.pose_ecef *
                    lidar_frame_data.transform_from_sensor_to_imu;
            for (const auto& point : lidar_frame_data.raw_cloud->points) {
                const Eigen::Vector3d p_origin =
                        T_origin_lidar * Eigen::Vector3d(point.x, point.y, point.z);
                const float clamped_intensity =
                        std::max(0.0f, std::min(255.0f, point.intensity));
                node.SetValue(p_origin,
                              frame.sensor_name(),
                              static_cast<unsigned char>(clamped_intensity));
            }

            ++processed_frames;
            if (processed_frames % 50 == 0) {
                LOG(INFO) << "processed " << processed_frames << " lidar frames";
            }
        }

        if (processed_frames == 0) {
            LOG(ERROR) << "no lidar frames were processed in segment " << segment_index;
            return 1;
        }

        std::filesystem::path output_path;
        if (FLAGS_distance > 0.0) {
            std::ostringstream file_name;
            file_name << "topdown_intensity_image_" << std::setw(3)
                      << std::setfill('0') << segment_index << ".png";
            output_path = output_dir / file_name.str();
        } else {
            output_path = output_dir / "topdown_intensity_image.png";
        }

        cv::Mat intensity_image;
        node.GetIntensityImage(&intensity_image, 1);
        if (!cv::imwrite(output_path.string(), intensity_image)) {
            LOG(ERROR) << "failed to write image: " << output_path.string();
            return 1;
        }

        LOG(INFO) << "saved topdown intensity image to " << output_path.string()
                  << ", frames=" << processed_frames
                  << ", image=" << grid_frame.cols << "x" << grid_frame.rows
                  << ", occupancy=" << node.GetOccupancyRatio() * 100.0 << "%";
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
