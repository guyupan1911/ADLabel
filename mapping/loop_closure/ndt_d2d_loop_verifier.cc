#include "mapping/loop_closure/ndt_d2d_loop_verifier.h"

#include <algorithm>
#include <chrono>
#include <cstdint>

#include <Eigen/Geometry>
#include <glog/logging.h>

#include "mapping/common/pose3d.h"
#include "mapping/lidar_topdown/lidar_lossless_map_node.h"
#include "mapping/loop_closure/ndt_d2d.h"
#include "mapping/mapping_utils/stitch_utils.h"

namespace adlabel {
namespace mapping {

namespace {

constexpr double kTopdownResolution = 0.1;
constexpr unsigned int kTopdownImageSize = 1600;
constexpr char kLocalMapLidarName[] = "local_map";

pcl::PointCloud<PointXYZIRT>::Ptr TransformFromVehicleFrameToImageFrame(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) {
  CHECK(cloud != nullptr);
  pcl::PointCloud<PointXYZIRT>::Ptr image_cloud(
      new pcl::PointCloud<PointXYZIRT>);
  image_cloud->points.reserve(cloud->points.size());
  for (const auto& point : cloud->points) {
    PointXYZIRT image_point = point;
    image_point.x = -point.y;
    image_point.y = point.x;
    image_cloud->points.push_back(image_point);
  }
  image_cloud->width = static_cast<std::uint32_t>(image_cloud->points.size());
  image_cloud->height = 1;
  image_cloud->is_dense = cloud->is_dense;
  return image_cloud;
}

cv::Mat RasterizeTopdownImageFromCloud(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) {
  CHECK(cloud != nullptr);
  CHECK(!cloud->points.empty())
      << "cannot generate topdown image from empty cloud";

  GridFrame grid_frame;
  grid_frame.resolution = kTopdownResolution;
  grid_frame.cols = kTopdownImageSize;
  grid_frame.rows = kTopdownImageSize;
  grid_frame.top_left_corner = {
      -static_cast<double>(grid_frame.cols) * 0.5 * kTopdownResolution,
      static_cast<double>(grid_frame.rows) * 0.5 * kTopdownResolution,
  };

  LidarLosslessMapNode node;
  node.Init(grid_frame,
            LidarLosslessMapNode::IntensityMappingMode::kLogarithmic,
            LidarLosslessMapNode::IntensityAggregationMode::kMean);

  for (const auto& point : cloud->points) {
    const float clamped_intensity =
        std::max(0.0f, std::min(255.0f, point.intensity));
    node.SetValue(Eigen::Vector3d(point.x, point.y, point.z),
                  kLocalMapLidarName,
                  static_cast<unsigned char>(clamped_intensity));
  }

  cv::Mat image;
  node.GetIntensityImage(&image, 1);
  return image;
}

pcl::PointCloud<PointXYZIRT>::Ptr TransformCloud(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud,
    const Eigen::Affine3d& transform) {
  CHECK(cloud != nullptr);
  pcl::PointCloud<PointXYZIRT>::Ptr transformed_cloud(
      new pcl::PointCloud<PointXYZIRT>);
  transformed_cloud->points.reserve(cloud->points.size());
  for (const auto& point : cloud->points) {
    PointXYZIRT transformed_point = point;
    const Eigen::Vector3d p_transformed =
        transform * Eigen::Vector3d(point.x, point.y, point.z);
    transformed_point.x = static_cast<float>(p_transformed.x());
    transformed_point.y = static_cast<float>(p_transformed.y());
    transformed_point.z = static_cast<float>(p_transformed.z());
    transformed_cloud->points.push_back(transformed_point);
  }
  transformed_cloud->width =
      static_cast<std::uint32_t>(transformed_cloud->points.size());
  transformed_cloud->height = 1;
  transformed_cloud->is_dense = cloud->is_dense;
  return transformed_cloud;
}

pcl::PointCloud<PointXYZIRT>::Ptr MergeClouds(
    const pcl::PointCloud<PointXYZIRT>::Ptr& first,
    const pcl::PointCloud<PointXYZIRT>::Ptr& second) {
  CHECK(first != nullptr);
  CHECK(second != nullptr);
  pcl::PointCloud<PointXYZIRT>::Ptr merged(new pcl::PointCloud<PointXYZIRT>);
  merged->points.reserve(first->points.size() + second->points.size());
  merged->points.insert(merged->points.end(), first->points.begin(),
                        first->points.end());
  merged->points.insert(merged->points.end(), second->points.begin(),
                        second->points.end());
  merged->width = static_cast<std::uint32_t>(merged->points.size());
  merged->height = 1;
  merged->is_dense = first->is_dense && second->is_dense;
  return merged;
}

void GenerateTopdownImageFromCloud(
    const pcl::PointCloud<PointXYZIRT>::Ptr& from_cloud,
    const pcl::PointCloud<PointXYZIRT>::Ptr& to_cloud,
    const Eigen::Affine3d& init_pose_relative, cv::Mat* from_image,
    cv::Mat* to_image, cv::Mat* merged_image,
    pcl::PointCloud<PointXYZIRT>::Ptr* merged_cloud) {
  CHECK(from_image != nullptr);
  CHECK(to_image != nullptr);
  CHECK(merged_image != nullptr);
  CHECK(merged_cloud != nullptr);

  pcl::PointCloud<PointXYZIRT>::Ptr transformed_from_cloud =
      TransformCloud(from_cloud, init_pose_relative);
  *merged_cloud = MergeClouds(transformed_from_cloud, to_cloud);

  *from_image = RasterizeTopdownImageFromCloud(
      TransformFromVehicleFrameToImageFrame(from_cloud));
  *to_image = RasterizeTopdownImageFromCloud(
      TransformFromVehicleFrameToImageFrame(to_cloud));
  *merged_image = RasterizeTopdownImageFromCloud(
      TransformFromVehicleFrameToImageFrame(*merged_cloud));
}

}  // namespace

Eigen::Affine3d NdtD2dLoopVerifier::RefineFramePairRelativePose(
    const FramePair& frame_pair, LoopVerifierResult* loop_verifier_result,
    const bool debug) {
  CHECK(loop_verifier_result != nullptr);

  from_local_map_ =
      StitchLocalMap(frame_pair.from_frame(), frame_pair.from_local_frames(),
                     frame_pair.from_relative_poses());

  to_local_map_ =
      StitchLocalMap(frame_pair.to_frame(), frame_pair.to_local_frames(),
                     frame_pair.to_relative_poses());

  init_relative_pose_ =
      Pose3D(frame_pair.to_frame().refined_pose_3d()).GetAffine3D().inverse() *
      Pose3D(frame_pair.from_frame().refined_pose_3d()).GetAffine3D();

  cv::Mat source_topdown_image;
  cv::Mat target_topdown_image;
  cv::Mat merge_before_refine_image;
  pcl::PointCloud<PointXYZIRT>::Ptr merged_before_refine_cloud;
  if (debug) {
    GenerateTopdownImageFromCloud(
        from_local_map_, to_local_map_, init_relative_pose_,
        &source_topdown_image, &target_topdown_image,
        &merge_before_refine_image, &merged_before_refine_cloud);
  }

  NdtD2D ndt_d2d(ndt_d2d_config_);
  ndt_d2d.SetInputTarget(ToPclCloud(to_local_map_));
  ndt_d2d.SetInputSource(ToPclCloud(from_local_map_));

  Eigen::Affine3d refined_pose = init_relative_pose_;
  double inlier_ratio = 0.0;
  int num_iterations = 0;
  const bool success = ndt_d2d.Align(init_relative_pose_, &inlier_ratio,
                                     &num_iterations, &refined_pose);
  if (!success) {
    LOG(WARNING) << "NDT-D2D failed, use initial relative pose";
    refined_pose = init_relative_pose_;
  } else {
    LOG(INFO) << "NDT-D2D finished, inlier_ratio=" << inlier_ratio
              << ", iterations=" << num_iterations;
  }

  Eigen::Matrix3d orientation_covariance;
  Eigen::Matrix3d position_covariance;
  ndt_d2d.CompuateCovariance(&orientation_covariance, &position_covariance);

  LOG(ERROR) << "orientation_covariance: \n" << orientation_covariance;
  LOG(ERROR) << "position_covariance: \n" << position_covariance;

  cv::Mat merge_after_refine_image;
  pcl::PointCloud<PointXYZIRT>::Ptr merged_after_refine_cloud;
  if (debug) {
    const auto generate_after_image_start = std::chrono::steady_clock::now();
    GenerateTopdownImageFromCloud(from_local_map_, to_local_map_, refined_pose,
                                  &source_topdown_image, &target_topdown_image,
                                  &merge_after_refine_image,
                                  &merged_after_refine_cloud);
    const auto generate_after_image_end = std::chrono::steady_clock::now();
    LOG(INFO) << "GenerateFramePairTopdownImages after refine cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     generate_after_image_end - generate_after_image_start)
                     .count()
              << " ms";
  }

  loop_verifier_result->refined_pose = refined_pose;
  loop_verifier_result->inlier_ratio = inlier_ratio;
  loop_verifier_result->num_iterations = num_iterations;
  loop_verifier_result->orientation_covariance = orientation_covariance;
  loop_verifier_result->position_covariance = position_covariance;
  loop_verifier_result->source_topdown_image = source_topdown_image;
  loop_verifier_result->target_topdown_image = target_topdown_image;
  loop_verifier_result->merge_before_refine_image = merge_before_refine_image;
  loop_verifier_result->merge_after_refine_image = merge_after_refine_image;
  loop_verifier_result->merged_before_refine_cloud = merged_before_refine_cloud;
  loop_verifier_result->merged_after_refine_cloud = merged_after_refine_cloud;
  return refined_pose;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NdtD2dLoopVerifier::ToPclCloud(
    const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) const {
  CHECK(cloud != nullptr);

  constexpr double kMaxDistanceFromOriginM = 100.0;
  constexpr double kMaxDistanceFromOriginSquaredM2 =
      kMaxDistanceFromOriginM * kMaxDistanceFromOriginM;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->points.reserve(cloud->points.size());
  for (const auto& point : cloud->points) {
    const double distance_squared =
        point.x * point.x + point.y * point.y + point.z * point.z;
    if (distance_squared > kMaxDistanceFromOriginSquaredM2) {
      continue;
    }
    pcl_cloud->points.emplace_back(point.x, point.y, point.z);
  }
  pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->points.size());
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = cloud->is_dense;
  return pcl_cloud;
}

void NdtD2dLoopVerifier::GenerateFramePairTopdownImages(
    const Eigen::Affine3d& init_pose_relative, cv::Mat* from_image,
    cv::Mat* to_image, cv::Mat* merged_image) {
  CHECK(from_local_map_ != nullptr) << "from_local_map_ has not been stitched";
  CHECK(to_local_map_ != nullptr) << "to_local_map_ has not been stitched";

  pcl::PointCloud<PointXYZIRT>::Ptr merged_cloud;
  GenerateTopdownImageFromCloud(from_local_map_, to_local_map_,
                                init_pose_relative, from_image, to_image,
                                merged_image, &merged_cloud);
}

pcl::PointCloud<PointXYZIRT>::Ptr NdtD2dLoopVerifier::StitchLocalMap(
    const Frame& reference_frame,
    const google::protobuf::RepeatedPtrField<Frame>& frames,
    const google::protobuf::RepeatedPtrField<Pose3DMessage>& relative_poses) {
  CHECK(local_data_reader_ != nullptr);
  CHECK_EQ(frames.size(), relative_poses.size())
      << "local frame count must match relative pose count";
  LOG(INFO) << "frames size: " << frames.size();

  pcl::PointCloud<PointXYZIRT>::Ptr local_map(new pcl::PointCloud<PointXYZIRT>);

  auto append_frame_cloud = [&](const Frame& frame,
                                const Eigen::Affine3d& T_reference_current) {
    FrameData frame_data;
    if (!GenerateLidarFrameData(frame, &frame_data, local_data_reader_)) {
      LOG(WARNING) << "failed to generate lidar frame data: " << frame.fid();
      return;
    }

    const Eigen::Affine3d T_reference_lidar =
        T_reference_current * frame_data.transform_from_sensor_to_imu;
    local_map->points.reserve(local_map->points.size() +
                              frame_data.raw_cloud->points.size());
    for (const auto& point : frame_data.raw_cloud->points) {
      PointXYZIRT transformed_point = point;
      const Eigen::Vector3d p_reference =
          T_reference_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      transformed_point.x = static_cast<float>(p_reference.x());
      transformed_point.y = static_cast<float>(p_reference.y());
      transformed_point.z = static_cast<float>(p_reference.z());
      local_map->points.push_back(transformed_point);
    }
  };

  append_frame_cloud(reference_frame, Eigen::Affine3d::Identity());
  for (int i = 0; i < frames.size(); ++i) {
    append_frame_cloud(frames.Get(i),
                       Pose3D(relative_poses.Get(i)).GetAffine3D());
  }

  local_map->width = static_cast<std::uint32_t>(local_map->points.size());
  local_map->height = 1;
  local_map->is_dense = false;
  return local_map;
}

}  // namespace mapping
}  // namespace adlabel