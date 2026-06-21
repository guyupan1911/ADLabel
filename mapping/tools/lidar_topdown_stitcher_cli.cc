#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include <opencv2/imgproc.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata, "", "lidar metadata path, relative to data_root or absolute");
DEFINE_string(data_root, "", "root directory for sensor data");
DEFINE_string(output_dir, "", "directory to save intensity.png");
DEFINE_double(resolution, 0.05, "topdown image resolution, meters per pixel");
DEFINE_double(margin_meters, 50.0, "extra margin around trajectory bounds");
DEFINE_uint64(min_samples, 1, "minimum samples per cell to render");
DEFINE_bool(use_log_intensity, true, "use logarithmic intensity mapping before image enhancement");
DEFINE_string(intensity_aggregation, "mean", "intensity aggregation in each cell: max or mean");
DEFINE_bool(apply_percentile_stretch, false, "apply percentile stretch to the intensity image");
DEFINE_double(percentile_low, 1.0, "low percentile used by intensity stretch");
DEFINE_double(percentile_high, 99.0, "high percentile used by intensity stretch");
DEFINE_bool(apply_clahe, true, "apply CLAHE after percentile stretch");
DEFINE_double(clahe_clip_limit, 2.0, "CLAHE clip limit");
DEFINE_int32(clahe_tile_size, 8, "CLAHE tile grid width and height");
DEFINE_double(distance, 0.0,
              "trajectory split distance in meters, 0 disables splitting");

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

bool IsProcessableLidarFrame(const Frame& frame);

SimplePose3DInterpolator BuildLioPoseInterpolator(const std::vector<Frame>& frames) {
    SimplePose3DInterpolator interpolator;
    size_t valid_pose_count = 0;

    for (const auto& frame : frames) {
        if (!frame.has_lio_pose_3d()) {
            continue;
        }
        interpolator.InsertTimestampedPose(
                frame.timestamp_ns(), Pose3D(frame.lio_pose_3d()));
        ++valid_pose_count;
    }

    LOG(INFO) << "built LIO pose interpolator with " << valid_pose_count
              << " poses from " << frames.size() << " lidar frames";
    return interpolator;
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

double FrameDistance2D(const Frame& left, const Frame& right) {
    if (!left.has_lio_pose_3d() || !right.has_lio_pose_3d()) {
        return 0.0;
    }
    const double dx = right.lio_pose_3d().x() - left.lio_pose_3d().x();
    const double dy = right.lio_pose_3d().y() - left.lio_pose_3d().y();
    return std::hypot(dx, dy);
}

std::vector<std::vector<Frame>> SplitFramesByDistance(
        const std::vector<Frame>& frames,
        double distance_m) {
    if (distance_m <= 0.0) {
        return {frames};
    }

    std::vector<std::vector<Frame>> segments;
    std::vector<Frame> current_segment;
    double current_distance_m = 0.0;

    for (const auto& frame : frames) {
        if (!IsProcessableLidarFrame(frame)) {
            continue;
        }

        if (!current_segment.empty()) {
            const double frame_distance = FrameDistance2D(current_segment.back(), frame);
            if (current_distance_m + frame_distance > distance_m) {
                segments.push_back(current_segment);
                current_segment.clear();
                current_distance_m = 0.0;
            } else {
                current_distance_m += frame_distance;
            }
        }

        current_segment.push_back(frame);
    }

    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }
    return segments;
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

std::string SegmentIntensityFileName(size_t segment_index) {
    std::ostringstream stream;
    stream << "intensity_" << std::setw(3) << std::setfill('0')
           << segment_index << ".png";
    return stream.str();
}

LidarLosslessMapNode::IntensityAggregationMode ParseIntensityAggregationMode(
        const std::string& mode) {
    if (mode == "max") {
        return LidarLosslessMapNode::IntensityAggregationMode::kMax;
    }
    if (mode == "mean") {
        return LidarLosslessMapNode::IntensityAggregationMode::kMean;
    }
    LOG(FATAL) << "unsupported --intensity_aggregation=" << mode
               << ", expected max or mean";
    return LidarLosslessMapNode::IntensityAggregationMode::kMax;
}

double GetPercentile(std::vector<unsigned char> values, double percentile) {
    CHECK(!values.empty());
    std::sort(values.begin(), values.end());

    const double normalized = std::max(0.0, std::min(100.0, percentile)) / 100.0;
    const double index = normalized * static_cast<double>(values.size() - 1);
    const size_t low_index = static_cast<size_t>(std::floor(index));
    const size_t high_index = static_cast<size_t>(std::ceil(index));
    const double alpha = index - static_cast<double>(low_index);
    return static_cast<double>(values[low_index]) * (1.0 - alpha) +
           static_cast<double>(values[high_index]) * alpha;
}

cv::Mat BuildValidIntensityMask(const LidarLosslessMapNode& node, size_t min_samples) {
    const GridFrame& frame = node.GetFrame();
    cv::Mat mask = cv::Mat::zeros(
            static_cast<int>(frame.rows), static_cast<int>(frame.cols), CV_8UC1);
    node.GetMatrix().ForEachOccupied(
            [&](unsigned int row, unsigned int col, const LosslessMapCell& cell) {
                if (cell.GetCount() < min_samples) return;
                mask.at<unsigned char>(static_cast<int>(row), static_cast<int>(col)) = 255;
            });
    return mask;
}

cv::Mat PercentileStretchIntensity(const LidarLosslessMapNode& node,
                                   const cv::Mat& image,
                                   size_t min_samples,
                                   double low_percentile,
                                   double high_percentile) {
    CHECK_EQ(image.type(), CV_8UC1);

    std::vector<unsigned char> values;
    node.GetMatrix().ForEachOccupied(
            [&](unsigned int, unsigned int, const LosslessMapCell& cell) {
                if (cell.GetCount() < min_samples) return;
                values.push_back(cell.GetValue());
            });

    if (values.empty()) {
        LOG(WARNING) << "skip percentile stretch because no valid intensity cells exist";
        return image.clone();
    }

    const double low_value = GetPercentile(values, low_percentile);
    const double high_value = GetPercentile(values, high_percentile);
    if (high_value <= low_value) {
        LOG(WARNING) << "skip percentile stretch because percentile range is invalid: "
                     << low_value << " to " << high_value;
        return image.clone();
    }

    cv::Mat stretched = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    node.GetMatrix().ForEachOccupied(
            [&](unsigned int row, unsigned int col, const LosslessMapCell& cell) {
                if (cell.GetCount() < min_samples) return;
                const double value = static_cast<double>(cell.GetValue());
                const double normalized = (value - low_value) / (high_value - low_value);
                const double scaled = std::max(0.0, std::min(255.0, normalized * 255.0));
                stretched.at<unsigned char>(static_cast<int>(row), static_cast<int>(col)) =
                        static_cast<unsigned char>(std::lround(scaled));
            });

    LOG(INFO) << "applied percentile stretch: p" << low_percentile << "=" << low_value
              << ", p" << high_percentile << "=" << high_value
              << ", valid_cells=" << values.size();
    return stretched;
}

cv::Mat ApplyClahe(const cv::Mat& image, const cv::Mat& valid_mask) {
    CHECK_EQ(image.type(), CV_8UC1);
    CHECK_EQ(valid_mask.type(), CV_8UC1);
    CHECK_EQ(image.rows, valid_mask.rows);
    CHECK_EQ(image.cols, valid_mask.cols);

    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
            FLAGS_clahe_clip_limit, cv::Size(FLAGS_clahe_tile_size, FLAGS_clahe_tile_size));
    clahe->apply(image, enhanced);
    enhanced.setTo(0, valid_mask == 0);
    return enhanced;
}

cv::Mat EnhanceIntensityImage(const LidarLosslessMapNode& node,
                              const cv::Mat& image,
                              size_t min_samples) {
    cv::Mat enhanced = image.clone();
    if (FLAGS_apply_percentile_stretch) {
        enhanced = PercentileStretchIntensity(node,
                                              enhanced,
                                              min_samples,
                                              FLAGS_percentile_low,
                                              FLAGS_percentile_high);
    }
    if (FLAGS_apply_clahe) {
        const cv::Mat valid_mask = BuildValidIntensityMask(node, min_samples);
        enhanced = ApplyClahe(enhanced, valid_mask);
        LOG(INFO) << "applied CLAHE: clip_limit=" << FLAGS_clahe_clip_limit
                  << ", tile_size=" << FLAGS_clahe_tile_size;
    }
    return enhanced;
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

bool RenderIntensityImage(const std::vector<Frame>& frames,
                          const std::shared_ptr<LocalDataReader>& data_reader,
                          const SimplePose3DInterpolator& lio_pose_interpolator,
                          const std::filesystem::path& output_path) {
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
    const auto intensity_aggregation =
            ParseIntensityAggregationMode(FLAGS_intensity_aggregation);

    LidarLosslessMapNode node;
    node.Init(grid_frame, intensity_mapping, intensity_aggregation);

    size_t processed_frames = 0;
    for (const auto& frame : frames) {
        if (!IsProcessableLidarFrame(frame)) {
            continue;
        }

        FrameData lidar_frame_data;
        if (!GenerateLidarFrameData(
                    frame, &lidar_frame_data, data_reader, &lio_pose_interpolator, true)) {
            LOG(ERROR) << "failed to generate lidar frame data: " << frame.fid();
            continue;
        }

        AccumulateFrameToNode(frame, lidar_frame_data, &node);
        ++processed_frames;
        if (processed_frames % 50 == 0) {
            LOG(INFO) << "processed " << processed_frames << " lidar frames";
        }
    }

    if (processed_frames == 0) {
        LOG(ERROR) << "no lidar frames were processed for " << output_path.string();
        return false;
    }

    cv::Mat intensity_image;
    node.GetIntensityImage(&intensity_image, static_cast<size_t>(FLAGS_min_samples));
    intensity_image = EnhanceIntensityImage(node,
                                            intensity_image,
                                            static_cast<size_t>(FLAGS_min_samples));

    if (!cv::imwrite(output_path.string(), intensity_image)) {
        LOG(ERROR) << "failed to write image: " << output_path.string();
        return false;
    }

    LOG(INFO) << "saved intensity image to " << output_path.string()
              << ", processed_frames=" << processed_frames
              << ", occupancy=" << node.GetOccupancyRatio() * 100.0 << "%";
    return true;
}

int Run() {
    CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
    CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
    CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
    CHECK_GT(FLAGS_resolution, 0.0) << "--resolution must be positive";
    CHECK_GE(FLAGS_margin_meters, 0.0) << "--margin_meters must be non-negative";
    CHECK_GE(FLAGS_percentile_low, 0.0) << "--percentile_low must be in [0, 100]";
    CHECK_LE(FLAGS_percentile_low, 100.0) << "--percentile_low must be in [0, 100]";
    CHECK_GE(FLAGS_percentile_high, 0.0) << "--percentile_high must be in [0, 100]";
    CHECK_LE(FLAGS_percentile_high, 100.0) << "--percentile_high must be in [0, 100]";
    CHECK_LT(FLAGS_percentile_low, FLAGS_percentile_high)
            << "--percentile_low must be less than --percentile_high";
    CHECK_GT(FLAGS_clahe_clip_limit, 0.0) << "--clahe_clip_limit must be positive";
    CHECK_GT(FLAGS_clahe_tile_size, 0) << "--clahe_tile_size must be positive";
    CHECK_GE(FLAGS_distance, 0.0) << "--distance must be non-negative";

    auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
    std::vector<Frame> frames = data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
    CHECK(!frames.empty()) << "no frames loaded from " << FLAGS_lidar_metadata;
    const SimplePose3DInterpolator lio_pose_interpolator =
            BuildLioPoseInterpolator(frames);

    const std::filesystem::path output_dir(FLAGS_output_dir);
    std::error_code error;
    std::filesystem::create_directories(output_dir, error);
    CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                  << ", error: " << error.message();

    const bool split_by_distance = FLAGS_distance > 0.0;
    const std::vector<std::vector<Frame>> frame_segments =
            SplitFramesByDistance(frames, FLAGS_distance);
    CHECK(!frame_segments.empty()) << "no processable frame segments";

    for (size_t index = 0; index < frame_segments.size(); ++index) {
        const std::filesystem::path output_path =
                split_by_distance
                        ? output_dir / SegmentIntensityFileName(index)
                        : output_dir / "intensity.png";
        LOG(INFO) << "render lidar topdown segment " << index
                  << "/" << frame_segments.size()
                  << ", frames=" << frame_segments[index].size()
                  << ", output=" << output_path.string();
        if (!RenderIntensityImage(
                    frame_segments[index], data_reader, lio_pose_interpolator, output_path)) {
            return 1;
        }
    }

    if (split_by_distance) {
        LOG(INFO) << "saved " << frame_segments.size()
                  << " intensity image segments with split distance "
                  << FLAGS_distance << " meters";
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
