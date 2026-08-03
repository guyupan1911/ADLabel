#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "mapping/camera/camera.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for sensor data");
DEFINE_string(lidar_metadata, "",
              "lidar metadata path, relative to data_root or absolute");
DEFINE_string(camera_metadata, "",
              "camera metadata path, relative to data_root or absolute");
DEFINE_string(output_dir, "", "directory to save projected camera images");
DEFINE_double(intensity_min, 60.0, "exclusive minimum point intensity");
DEFINE_double(intensity_max, 100.0, "exclusive maximum point intensity");
DEFINE_uint64(max_frames, 0,
              "maximum number of initial lidar frames; 0 means all");
DEFINE_bool(use_lio, false,
            "use lidar pose_utm instead of pose_ecef for mapping and camera "
            "pose interpolation");

namespace adlabel {
namespace mapping {
namespace {

struct MapPoint {
  Eigen::Vector3f position;
  float intensity = 0.0f;
};

bool IsFinitePoint(const PointXYZIRT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

cv::Scalar IntensityColor(float intensity) {
  const double t =
      std::max(0.0, std::min(1.0, static_cast<double>(intensity) / 255.0));
  const auto channel = [t](double center) {
    return 255.0 *
           std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * t - center)));
  };
  return cv::Scalar(channel(1.0), channel(2.0), channel(3.0));
}

void ConvertToBgr(cv::Mat* image) {
  CHECK(image != nullptr);
  if (image->channels() == 1) {
    if (image->depth() == CV_8U) {
      cv::cvtColor(*image, *image, cv::COLOR_GRAY2BGR);
    } else {
      cv::Mat normalized;
      cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(normalized, *image, cv::COLOR_GRAY2BGR);
    }
    return;
  }
  if (image->channels() == 4) {
    cv::cvtColor(*image, *image, cv::COLOR_BGRA2BGR);
  }
  CHECK_EQ(image->channels(), 3) << "unsupported image channel count";
  if (image->depth() != CV_8U) {
    cv::Mat normalized;
    cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    *image = normalized;
  }
}

std::vector<MapPoint> BuildMapCloud(
    const std::vector<Frame>& lidar_frames,
    const std::shared_ptr<LocalDataReader>& data_reader,
    SimplePose3DInterpolator* pose_interpolator, Eigen::Affine3d* T_map_world,
    std::vector<int64_t>* lidar_pose_timestamps) {
  CHECK(data_reader != nullptr);
  CHECK(pose_interpolator != nullptr);
  CHECK(T_map_world != nullptr);
  CHECK(lidar_pose_timestamps != nullptr);

  std::vector<MapPoint> map_cloud;
  size_t processed_frames = 0;
  size_t ground_points = 0;
  lidar_pose_timestamps->clear();
  for (const Frame& frame : lidar_frames) {
    const bool has_selected_pose =
        FLAGS_use_lio ? frame.has_lio_pose_3d() : frame.has_refined_pose_3d();
    if (!frame.has_timestamp_ns() || !has_selected_pose ||
        !frame.has_cloud_uri() || frame.cloud_uri().empty() ||
        !frame.has_lidar_nn_uri() || frame.lidar_nn_uri().empty() ||
        !frame.has_sensor_to_imu_extrinsic()) {
      continue;
    }

    FrameData frame_data;
    if (!GenerateLidarFrameData(frame, &frame_data, data_reader) ||
        frame_data.ground_cloud == nullptr) {
      LOG(WARNING) << "skip lidar frame without valid lidar_nn labels: "
                   << frame.fid();
      continue;
    }

    const Eigen::Affine3d& T_world_imu =
        FLAGS_use_lio ? frame_data.pose_utm : frame_data.pose_ecef;
    pose_interpolator->InsertTimestampedPose(frame.timestamp_ns(),
                                             Pose3D(T_world_imu));
    if (lidar_pose_timestamps->empty()) {
      *T_map_world = T_world_imu.inverse();
    }
    lidar_pose_timestamps->push_back(frame.timestamp_ns());

    // pose is T_world_imu and the extrinsic is T_imu_lidar.
    const Eigen::Affine3d T_map_lidar =
        *T_map_world * T_world_imu * frame_data.transform_from_sensor_to_imu;
    ground_points += frame_data.ground_cloud->size();
    for (const PointXYZIRT& point : *frame_data.ground_cloud) {
      if (!IsFinitePoint(point) || point.intensity <= FLAGS_intensity_min ||
          point.intensity >= FLAGS_intensity_max) {
        continue;
      }
      const Eigen::Vector3d position_map =
          T_map_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      map_cloud.push_back(
          {position_map.cast<float>(), static_cast<float>(point.intensity)});
    }

    ++processed_frames;
    if (processed_frames % 50 == 0) {
      LOG(INFO) << "stitched " << processed_frames
                << " lidar frames, retained_points=" << map_cloud.size();
    }
  }

  LOG(INFO) << "built map cloud from " << processed_frames
            << " lidar frames, ground_points=" << ground_points
            << ", intensity_range=(" << FLAGS_intensity_min << ", "
            << FLAGS_intensity_max << ")"
            << " retained_points=" << map_cloud.size();
  LOG(INFO) << "initialized " << (FLAGS_use_lio ? "pose_utm" : "pose_ecef")
            << " trajectory from FrameData with "
            << lidar_pose_timestamps->size() << " poses";
  return map_cloud;
}

int64_t FindNearestTimestamp(const std::vector<int64_t>& timestamps,
                             int64_t timestamp_ns) {
  CHECK(!timestamps.empty());
  const auto upper =
      std::lower_bound(timestamps.begin(), timestamps.end(), timestamp_ns);
  if (upper == timestamps.begin()) {
    return *upper;
  }
  if (upper == timestamps.end()) {
    return timestamps.back();
  }
  const int64_t before = *std::prev(upper);
  const int64_t after = *upper;
  return timestamp_ns - before <= after - timestamp_ns ? before : after;
}

void DrawTimestampInfo(int64_t camera_timestamp_ns, int64_t lidar_timestamp_ns,
                       cv::Mat* image) {
  CHECK(image != nullptr);
  const double delta_ms =
      static_cast<double>(camera_timestamp_ns - lidar_timestamp_ns) / 1e6;
  std::vector<std::string> lines;
  lines.push_back("camera_ns: " + std::to_string(camera_timestamp_ns));
  lines.push_back("nearest_lidar_ns: " + std::to_string(lidar_timestamp_ns));
  std::ostringstream delta_stream;
  delta_stream << "camera-lidar: " << std::showpos << std::fixed
               << std::setprecision(3) << delta_ms << " ms";
  lines.push_back(delta_stream.str());

  constexpr int kFontFace = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kFontScale = 0.6;
  constexpr int kThickness = 1;
  constexpr int kMargin = 8;
  constexpr int kLineGap = 6;
  int baseline = 0;
  int text_width = 0;
  int text_height = 0;
  for (const std::string& line : lines) {
    const cv::Size size =
        cv::getTextSize(line, kFontFace, kFontScale, kThickness, &baseline);
    text_width = std::max(text_width, size.width);
    text_height += size.height + kLineGap;
  }
  cv::rectangle(*image, cv::Point(0, 0),
                cv::Point(text_width + 2 * kMargin, text_height + 2 * kMargin),
                cv::Scalar(0, 0, 0), cv::FILLED);
  int y = kMargin;
  for (const std::string& line : lines) {
    const cv::Size size =
        cv::getTextSize(line, kFontFace, kFontScale, kThickness, &baseline);
    y += size.height;
    cv::putText(*image, line, cv::Point(kMargin, y), kFontFace, kFontScale,
                cv::Scalar(255, 255, 255), kThickness, cv::LINE_AA);
    y += kLineGap;
  }
}

bool InterpolateCameraImuPose(const Frame& camera_frame,
                              const SimplePose3DInterpolator& pose_interpolator,
                              Eigen::Affine3d* T_world_imu) {
  CHECK(T_world_imu != nullptr);
  if (!camera_frame.has_timestamp_ns()) {
    return false;
  }

  Pose3D interpolated_pose;
  if (!pose_interpolator.GetTimestampedPose(camera_frame.timestamp_ns(),
                                            &interpolated_pose, false)) {
    LOG(WARNING) << "failed to interpolate IMU pose for camera frame: "
                 << camera_frame.fid();
    return false;
  }
  *T_world_imu = interpolated_pose.GetAffine3D();
  return true;
}

size_t ProjectMapCloud(const std::vector<MapPoint>& map_cloud,
                       const Eigen::Affine3d& T_camera_map,
                       const BaseCamera& camera, cv::Mat* image) {
  CHECK(image != nullptr);
  size_t projected_count = 0;
  for (const MapPoint& map_point : map_cloud) {
    const Eigen::Vector3d point_camera =
        T_camera_map * map_point.position.cast<double>();
    Eigen::Vector2d pixel;
    if (!camera.Project(point_camera, &pixel) ||
        !pixel.array().isFinite().all()) {
      continue;
    }
    const int u = static_cast<int>(std::lround(pixel.x()));
    const int v = static_cast<int>(std::lround(pixel.y()));
    if (u < 0 || u >= image->cols || v < 0 || v >= image->rows) {
      continue;
    }
    cv::circle(*image, cv::Point(u, v), 1, IntensityColor(map_point.intensity),
               -1, cv::LINE_AA);
    ++projected_count;
  }
  return projected_count;
}

std::string SanitizeFileComponent(std::string value) {
  for (char& character : value) {
    const bool valid = (character >= 'a' && character <= 'z') ||
                       (character >= 'A' && character <= 'Z') ||
                       (character >= '0' && character <= '9') ||
                       character == '-' || character == '_';
    if (!valid) {
      character = '_';
    }
  }
  return value.empty() ? "camera" : value;
}

std::string MakeOutputFilename(const Frame& frame, size_t index) {
  std::ostringstream stream;
  stream << "projection_" << std::setfill('0') << std::setw(6) << index << "_"
         << SanitizeFileComponent(frame.sensor_name()) << "_"
         << frame.timestamp_ns() << ".png";
  return stream.str();
}

int Run() {
  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_camera_metadata.empty()) << "--camera_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";
  CHECK(std::isfinite(FLAGS_intensity_min) &&
        std::isfinite(FLAGS_intensity_max) &&
        FLAGS_intensity_min < FLAGS_intensity_max)
      << "--intensity_min and --intensity_max must be finite and min < max";

  const auto data_reader = std::make_shared<LocalDataReader>(FLAGS_data_root);
  std::vector<Frame> lidar_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_lidar_metadata);
  std::vector<Frame> camera_frames =
      data_reader->ReadMetaData<Frame>(FLAGS_camera_metadata);
  CHECK(!lidar_frames.empty()) << "no lidar frames loaded";
  CHECK(!camera_frames.empty()) << "no camera frames loaded";
  std::sort(lidar_frames.begin(), lidar_frames.end(),
            [](const Frame& lhs, const Frame& rhs) {
              return lhs.timestamp_ns() < rhs.timestamp_ns();
            });
  if (FLAGS_max_frames > 0 && lidar_frames.size() > FLAGS_max_frames) {
    lidar_frames.resize(static_cast<size_t>(FLAGS_max_frames));
  }
  std::sort(camera_frames.begin(), camera_frames.end(),
            [](const Frame& lhs, const Frame& rhs) {
              return lhs.timestamp_ns() < rhs.timestamp_ns();
            });

  SimplePose3DInterpolator pose_interpolator;
  Eigen::Affine3d T_map_world = Eigen::Affine3d::Identity();
  std::vector<int64_t> lidar_pose_timestamps;
  const std::vector<MapPoint> map_cloud =
      BuildMapCloud(lidar_frames, data_reader, &pose_interpolator, &T_map_world,
                    &lidar_pose_timestamps);
  CHECK_GE(lidar_pose_timestamps.size(), static_cast<size_t>(2))
      << "at least two valid lidar FrameData IMU poses are required";
  CHECK(!map_cloud.empty()) << "map cloud has no valid ground points";
  const Eigen::Affine3d T_world_map = T_map_world.inverse();

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error=" << error.message();

  size_t processed_frames = 0;
  size_t saved_images = 0;
  for (size_t index = 0; index < camera_frames.size(); ++index) {
    const Frame& frame = camera_frames[index];
    if (!frame.has_camera_image_uri() || frame.camera_image_uri().empty() ||
        !frame.has_sensor_to_imu_extrinsic() ||
        !frame.has_camera_calibration()) {
      continue;
    }

    Eigen::Affine3d T_world_imu;
    if (!InterpolateCameraImuPose(frame, pose_interpolator, &T_world_imu)) {
      continue;
    }

    cv::Mat image;
    if (!data_reader->ReadImage(frame.camera_image_uri(), &image)) {
      continue;
    }
    ConvertToBgr(&image);
    const std::unique_ptr<BaseCamera> camera =
        CreateCamera(frame.camera_calibration());
    if (camera->Width() != static_cast<size_t>(image.cols) ||
        camera->Height() != static_cast<size_t>(image.rows)) {
      LOG(WARNING) << "camera calibration size " << camera->Width() << "x"
                   << camera->Height() << " differs from image size "
                   << image.cols << "x" << image.rows << ": " << frame.fid();
    }

    // T_camera_map = T_camera_imu * T_imu_world * T_world_map.
    const Eigen::Affine3d T_camera_map =
        Pose3D(frame.sensor_to_imu_extrinsic()).GetAffine3D().inverse() *
        T_world_imu.inverse() * T_world_map;
    const size_t projected_count =
        ProjectMapCloud(map_cloud, T_camera_map, *camera, &image);
    const int64_t nearest_lidar_timestamp_ns =
        FindNearestTimestamp(lidar_pose_timestamps, frame.timestamp_ns());
    DrawTimestampInfo(frame.timestamp_ns(), nearest_lidar_timestamp_ns, &image);

    const std::filesystem::path output_path =
        output_dir / MakeOutputFilename(frame, index);
    if (!cv::imwrite(output_path.string(), image)) {
      LOG(ERROR) << "failed to save projected image: " << output_path.string();
      continue;
    }

    ++processed_frames;
    ++saved_images;
    LOG(INFO) << "saved " << output_path.string()
              << ", projected_pixels=" << projected_count;
  }

  LOG(INFO) << "map projection finished, processed_camera_frames="
            << processed_frames << ", saved_images=" << saved_images;
  return saved_images > 0 ? 0 : 1;
}

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return adlabel::mapping::Run();
}
