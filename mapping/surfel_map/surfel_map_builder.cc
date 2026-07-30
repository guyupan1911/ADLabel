#include "mapping/surfel_map/surfel_map_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "mapping/camera/camera.h"
#include "mapping/common/pcl_types.h"
#include "mapping/common/pose3d.h"
#include "mapping/mapping_utils/frame_data.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/mapping_utils/timestamp_aligner.h"

namespace adlabel {
namespace mapping {
namespace {

bool IsFinitePoint(const PointXYZIRT& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

void ConvertToBgr(cv::Mat* image) {
  CHECK(image != nullptr);
  if (image->channels() == 1) {
    cv::cvtColor(*image, *image, cv::COLOR_GRAY2BGR);
  } else if (image->channels() == 4) {
    cv::cvtColor(*image, *image, cv::COLOR_BGRA2BGR);
  }
  CHECK_EQ(image->channels(), 3);

  if (image->depth() != CV_8U) {
    cv::Mat normalized;
    cv::normalize(*image, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    *image = normalized;
  }
}

cv::Vec3b BilinearSample(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  const double x =
      std::max(0.0, std::min(pixel.x(), static_cast<double>(image.cols - 1)));
  const double y =
      std::max(0.0, std::min(pixel.y(), static_cast<double>(image.rows - 1)));
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, image.cols - 1);
  const int y1 = std::min(y0 + 1, image.rows - 1);
  const double dx = x - x0;
  const double dy = y - y0;

  const cv::Vec3d c00 = image.at<cv::Vec3b>(y0, x0);
  const cv::Vec3d c10 = image.at<cv::Vec3b>(y0, x1);
  const cv::Vec3d c01 = image.at<cv::Vec3b>(y1, x0);
  const cv::Vec3d c11 = image.at<cv::Vec3b>(y1, x1);
  const cv::Vec3d color = (1.0 - dx) * (1.0 - dy) * c00 +
                          dx * (1.0 - dy) * c10 + (1.0 - dx) * dy * c01 +
                          dx * dy * c11;

  return {
      cv::saturate_cast<uint8_t>(color[0]),
      cv::saturate_cast<uint8_t>(color[1]),
      cv::saturate_cast<uint8_t>(color[2]),
  };
}

bool IsProcessableLidarFrame(const Frame& frame) {
  return frame.has_timestamp_ns() && frame.has_refined_pose_3d() &&
         frame.has_sensor_to_imu_extrinsic() && frame.has_cloud_uri() &&
         !frame.cloud_uri().empty();
}

bool IsProcessableCameraFrame(const Frame& frame) {
  return frame.has_timestamp_ns() && frame.has_sensor_to_imu_extrinsic() &&
         frame.has_camera_calibration() && frame.has_camera_image_uri() &&
         !frame.camera_image_uri().empty();
}

}  // namespace

SurfelMapBuilder::SurfelMapBuilder(const SurfelMapBuilderOptions& options,
                                   std::shared_ptr<LocalDataReader> data_reader,
                                   std::vector<Frame> lidar_frames,
                                   std::vector<Frame> camera_frames)
    : options_(options),
      data_reader_(std::move(data_reader)),
      lidar_frames_(std::move(lidar_frames)),
      camera_frames_(std::move(camera_frames)),
      surfel_map_(options.surfel_map_options) {
  CHECK(data_reader_ != nullptr);
  CHECK_GE(options_.maximum_time_difference_ns, 0);

  const auto by_timestamp = [](const Frame& left, const Frame& right) {
    return left.timestamp_ns() < right.timestamp_ns();
  };
  std::sort(lidar_frames_.begin(), lidar_frames_.end(), by_timestamp);
  std::sort(camera_frames_.begin(), camera_frames_.end(), by_timestamp);
}

bool SurfelMapBuilder::Build() {
  if (!InitializeTrajectory()) {
    return false;
  }
  if (!BuildGeometry()) {
    return false;
  }
  if (!BuildTexture()) {
    return false;
  }
  return true;
}

bool SurfelMapBuilder::InitializeTrajectory() {
  const Frame* first_valid_frame = nullptr;
  size_t pose_count = 0;
  for (const Frame& frame : lidar_frames_) {
    if (!frame.has_timestamp_ns() || !frame.has_refined_pose_3d()) {
      continue;
    }

    const Pose3D pose_ecef(frame.refined_pose_3d());
    pose_ecef_interpolator_.InsertTimestampedPose(frame.timestamp_ns(),
                                                  pose_ecef);
    if (first_valid_frame == nullptr) {
      first_valid_frame = &frame;
    }
    ++pose_count;
  }

  if (pose_count < 2 || first_valid_frame == nullptr) {
    LOG(ERROR) << "at least two refined LiDAR poses are required";
    return false;
  }

  T_map_ecef_ =
      Pose3D(first_valid_frame->refined_pose_3d()).GetAffine3D().inverse();
  LOG(INFO) << "initialized ECEF trajectory with " << pose_count << " poses";
  return true;
}

bool SurfelMapBuilder::BuildGeometry() {
  size_t processed_frames = 0;
  size_t inserted_points = 0;

  for (const Frame& frame : lidar_frames_) {
    if (options_.maximum_frames > 0 &&
        processed_frames >= options_.maximum_frames) {
      break;
    }
    if (!IsProcessableLidarFrame(frame)) {
      continue;
    }

    FrameData lidar_data;
    if (!GenerateLidarFrameData(frame, &lidar_data, data_reader_)) {
      continue;
    }
    if (lidar_data.ground_cloud == nullptr) {
      LOG(WARNING) << "skip LiDAR frame without ground labels: " << frame.fid();
      continue;
    }

    const Eigen::Affine3d T_map_lidar = T_map_ecef_ * lidar_data.pose_ecef *
                                        lidar_data.transform_from_sensor_to_imu;
    for (const PointXYZIRT& point : *lidar_data.ground_cloud) {
      if (!IsFinitePoint(point)) {
        continue;
      }
      surfel_map_.InsertGeometryPoint(
          T_map_lidar * Eigen::Vector3d(point.x, point.y, point.z));
      ++inserted_points;
    }
    ++processed_frames;
  }

  LOG(INFO) << "geometry input: frames=" << processed_frames
            << ", ground_points=" << inserted_points;
  if (processed_frames == 0 || inserted_points == 0) {
    return false;
  }
  return surfel_map_.FinalizeGeometry();
}

bool SurfelMapBuilder::BuildTexture() {
  TimestampAligner aligner(options_.maximum_time_difference_ns);
  aligner.AddSensorFrameRefs("lidar", lidar_frames_);
  aligner.AddSensorFrameRefs("camera", camera_frames_);
  const std::vector<AlignedFrames> aligned_frames =
      aligner.Align("lidar", true);
  if (aligned_frames.empty()) {
    LOG(ERROR) << "no aligned LiDAR/Camera frames";
    return false;
  }

  size_t processed_pairs = 0;
  for (const AlignedFrames& aligned : aligned_frames) {
    if (options_.maximum_frames > 0 &&
        processed_pairs >= options_.maximum_frames) {
      break;
    }

    const auto camera_ref = aligned.aligned_frames.find("camera");
    if (camera_ref == aligned.aligned_frames.end()) {
      continue;
    }
    const size_t lidar_index = aligned.reference_frame.index;
    const size_t camera_index = camera_ref->second.index;
    if (lidar_index >= lidar_frames_.size() ||
        camera_index >= camera_frames_.size() ||
        camera_index > std::numeric_limits<uint32_t>::max()) {
      continue;
    }

    const Frame& lidar_frame = lidar_frames_[lidar_index];
    const Frame& camera_frame = camera_frames_[camera_index];
    if (!IsProcessableLidarFrame(lidar_frame) ||
        !IsProcessableCameraFrame(camera_frame)) {
      continue;
    }

    FrameData lidar_data;
    FrameData camera_data;
    if (!GenerateLidarFrameData(lidar_frame, &lidar_data, data_reader_) ||
        !GenerateCameraFrameData(camera_frame, &camera_data, data_reader_) ||
        lidar_data.ground_cloud == nullptr) {
      continue;
    }

    if (!InterpolateCameraPose(camera_frame.timestamp_ns(),
                               &camera_data.pose_ecef)) {
      continue;
    }
    ConvertToBgr(&camera_data.camera_image);
    const std::unique_ptr<BaseCamera> camera =
        CreateCamera(camera_data.camera_calibration);

    const Eigen::Affine3d T_camera_lidar =
        camera_data.transform_from_sensor_to_imu.inverse() *
        camera_data.pose_ecef.inverse() * lidar_data.pose_ecef *
        lidar_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d T_map_lidar = T_map_ecef_ * lidar_data.pose_ecef *
                                        lidar_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d T_map_camera =
        T_map_ecef_ * camera_data.pose_ecef *
        camera_data.transform_from_sensor_to_imu;
    const Eigen::Affine3d T_camera_map = T_map_camera.inverse();
    const Eigen::Vector3d camera_position_map = T_map_camera.translation();

    std::unordered_set<VoxelKey, VoxelKeyHash> supported_surfels;
    for (const PointXYZIRT& point : *lidar_data.ground_cloud) {
      if (!IsFinitePoint(point)) {
        continue;
      }

      const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
      const Eigen::Vector3d point_camera = T_camera_lidar * point_lidar;
      Eigen::Vector2d pixel;
      if (!camera->Project(point_camera, &pixel)) {
        continue;
      }

      const Eigen::Vector3d point_map = T_map_lidar * point_lidar;
      const VoxelKey key = surfel_map_.GetVoxelKey(point_map);
      const SurfelNode* surfel = surfel_map_.FindSurfel(key);
      if (surfel == nullptr || !surfel->geometry_valid) {
        continue;
      }

      const Eigen::Vector3d camera_to_point = camera_position_map - point_map;
      const double distance = camera_to_point.norm();
      const float support_score =
          static_cast<float>(1.0 / std::max(distance, 1.0));
      surfel_map_.AddCameraCandidate(
          point_map,
          CameraCandidate{static_cast<uint32_t>(camera_index), support_score});
      supported_surfels.insert(key);
    }

    for (const VoxelKey& key : supported_surfels) {
      const SurfelNode* surfel = surfel_map_.FindSurfel(key);
      if (surfel == nullptr || !surfel->geometry_valid) {
        continue;
      }

      for (int cell_index = 0;
           cell_index < static_cast<int>(surfel->texture_cells.size());
           ++cell_index) {
        const Eigen::Vector3d cell_center =
            surfel_map_.GetTextureCellCenter(key, *surfel, cell_index);
        if (!cell_center.array().isFinite().all()) {
          continue;
        }
        const Eigen::Vector3d camera_to_cell =
            camera_position_map - cell_center;
        const double distance = camera_to_cell.norm();

        const Eigen::Vector3d point_camera = T_camera_map * cell_center;
        Eigen::Vector2d pixel;
        if (!camera->Project(point_camera, &pixel) || pixel.x() < 0.0 ||
            pixel.x() >= camera_data.camera_image.cols || pixel.y() < 0.0 ||
            pixel.y() >= camera_data.camera_image.rows) {
          continue;
        }

        const float score = static_cast<float>(1.0 / std::max(distance, 1.0));
        surfel_map_.UpdateTextureCell(
            key, cell_index, BilinearSample(camera_data.camera_image, pixel),
            score, static_cast<uint32_t>(camera_index));
      }
    }

    ++processed_pairs;
    if (processed_pairs % 50 == 0) {
      LOG(INFO) << "textured " << processed_pairs << " aligned frame pairs";
    }
  }

  LOG(INFO) << "texture result: aligned_pairs=" << processed_pairs
            << ", colored_cells=" << surfel_map_.ColoredCellCount();
  return processed_pairs > 0 && surfel_map_.ColoredCellCount() > 0;
}

bool SurfelMapBuilder::InterpolateCameraPose(int64_t timestamp_ns,
                                             Eigen::Affine3d* pose_ecef) const {
  CHECK(pose_ecef != nullptr);
  Pose3D interpolated_pose;
  if (!pose_ecef_interpolator_.GetTimestampedPose(timestamp_ns,
                                                  &interpolated_pose, false)) {
    LOG(WARNING) << "failed to interpolate Camera ECEF pose at "
                 << timestamp_ns;
    return false;
  }
  *pose_ecef = interpolated_pose.GetAffine3D();
  return true;
}

}  // namespace mapping
}  // namespace adlabel
