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
#include <GeographicLib/Geocentric.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for LocalDataReader");
DEFINE_string(lidar_metadata, "", "path to lidar frame metadata file");
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

struct LocalFramePose {
    Frame frame;
    Eigen::Vector2d origin_xy{0.0, 0.0};
};

Eigen::Matrix3d EcefToEnuRotation(double lat_deg, double lon_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double lat = lat_deg * kDegToRad;
    const double lon = lon_deg * kDegToRad;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    Eigen::Matrix3d rotation;
    rotation << -sin_lon, cos_lon, 0.0,
                -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
                 cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
    return rotation;
}

Eigen::Vector3d EcefToEnu(const Eigen::Vector3d& point_ecef,
                          const Eigen::Vector3d& origin_ecef,
                          const Eigen::Matrix3d& ecef_to_enu_rotation) {
    return ecef_to_enu_rotation * (point_ecef - origin_ecef);
}

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
    CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
    CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
    CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
    CHECK_GE(FLAGS_margin_meters, 0.0) << "--margin_meters must be non-negative";
    CHECK_GE(FLAGS_distance, 0.0) << "--distance must be non-negative";
    const LidarLosslessMapNode::IntensityAggregationMode aggregation_mode =
            ParseAggregationMode(FLAGS_aggregation_mode);

    const std::filesystem::path data_root(FLAGS_data_root);
    const std::filesystem::path lidar_metadata_path(FLAGS_lidar_metadata);
    const std::filesystem::path output_dir(FLAGS_output_dir);
    std::error_code error;
    std::filesystem::create_directories(output_dir, error);
    CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                  << ", error: " << error.message();

    auto data_reader = std::make_shared<LocalDataReader>(data_root.string());
    LOG(INFO) << "local_data_reader_root=" << data_root.string()
              << ", lidar_metadata=" << lidar_metadata_path.string();

    std::vector<Frame> frames = ReadMetaFile<Frame>(lidar_metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from " << lidar_metadata_path.string();
    std::sort(frames.begin(), frames.end(), [](const Frame& lhs, const Frame& rhs) {
        return lhs.timestamp_ns() < rhs.timestamp_ns();
    });

    Eigen::Vector3d enu_origin_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ecef_to_enu_rotation = Eigen::Matrix3d::Identity();
    bool has_enu_origin = false;
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
        const Eigen::Vector3d pose_translation_ecef = pose_ecef.translation();
        if (!has_enu_origin) {
            double lat = 0.0;
            double lon = 0.0;
            double height = 0.0;
            GeographicLib::Geocentric::WGS84().Reverse(
                    pose_translation_ecef.x(), pose_translation_ecef.y(),
                    pose_translation_ecef.z(), lat, lon, height);
            enu_origin_ecef = pose_translation_ecef;
            ecef_to_enu_rotation = EcefToEnuRotation(lat, lon);
            has_enu_origin = true;
        }

        const Eigen::Vector3d pose_enu =
                EcefToEnu(pose_translation_ecef, enu_origin_ecef,
                          ecef_to_enu_rotation);

        LocalFramePose local_frame;
        local_frame.frame = frame;
        local_frame.origin_xy = pose_enu.head<2>();
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

            const Eigen::Affine3d T_ecef_lidar =
                    lidar_frame_data.pose_ecef *
                    lidar_frame_data.transform_from_sensor_to_imu;
            for (const auto& point : lidar_frame_data.raw_cloud->points) {
                const Eigen::Vector3d p_ecef =
                        T_ecef_lidar * Eigen::Vector3d(point.x, point.y, point.z);
                const Eigen::Vector3d p_enu =
                        EcefToEnu(p_ecef, enu_origin_ecef,
                                  ecef_to_enu_rotation);
                const float clamped_intensity =
                        std::max(0.0f, std::min(255.0f, point.intensity));
                node.SetValue(p_enu,
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
