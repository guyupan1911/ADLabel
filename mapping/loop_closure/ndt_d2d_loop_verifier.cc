#include "mapping/loop_closure/ndt_d2d_loop_verifier.h"

#include <cstdint>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>

#include "mapping/common/pose3d.h"
#include "mapping/loop_closure/ndt_d2d.h"
#include "mapping/mapping_utils/stitch_utils.h"
#include "mapping/registration/registration_visualization.h"

namespace adlabel {
namespace mapping {
namespace {

constexpr float kNdtInputVoxelSizeM = 0.2f;

pcl::PointCloud<pcl::PointXYZ>::Ptr DownsampleNdtInput(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
  CHECK(cloud != nullptr);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(kNdtInputVoxelSizeM, kNdtInputVoxelSizeM,
                         kNdtInputVoxelSizeM);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*downsampled_cloud);
  LOG(INFO) << "NDT input voxel downsampling: " << cloud->size() << " -> "
            << downsampled_cloud->size()
            << " points, leaf_size=" << kNdtInputVoxelSizeM << " m";
  return downsampled_cloud;
}

}  // namespace

Eigen::Affine3d NdtD2dLoopVerifier::RefineFramePairRelativePose(
    const FramePair& frame_pair, LoopVerifierResult* loop_verifier_result,
    const bool debug) {
  CHECK(loop_verifier_result != nullptr);

  const pcl::PointCloud<PointXYZIRT>::Ptr from_local_map =
      StitchLocalMap(frame_pair.from_frame(), frame_pair.from_local_frames(),
                     frame_pair.from_relative_poses());

  const pcl::PointCloud<PointXYZIRT>::Ptr to_local_map =
      StitchLocalMap(frame_pair.to_frame(), frame_pair.to_local_frames(),
                     frame_pair.to_relative_poses());

  const Eigen::Affine3d init_relative_pose =
      Pose3D(frame_pair.to_frame().refined_pose_3d()).GetAffine3D().inverse() *
      Pose3D(frame_pair.from_frame().refined_pose_3d()).GetAffine3D();

  // const pcl::PointCloud<pcl::PointXYZ>::Ptr from_ndt_cloud =
  //     DownsampleNdtInput(ToPclCloud(from_local_map));
  // const pcl::PointCloud<pcl::PointXYZ>::Ptr to_ndt_cloud =
  //     DownsampleNdtInput(ToPclCloud(to_local_map));

  const pcl::PointCloud<pcl::PointXYZ>::Ptr from_ndt_cloud =
      ToPclCloud(from_local_map);
  const pcl::PointCloud<pcl::PointXYZ>::Ptr to_ndt_cloud =
      ToPclCloud(to_local_map);

  NdtD2D ndt_d2d(ndt_d2d_config_);
  ndt_d2d.SetInputTarget(to_ndt_cloud);
  ndt_d2d.SetInputSource(from_ndt_cloud);

  Eigen::Affine3d refined_pose = init_relative_pose;
  double inlier_ratio = 0.0;
  int num_iterations = 0;
  const bool success = ndt_d2d.Align(init_relative_pose, &inlier_ratio,
                                     &num_iterations, &refined_pose);
  if (!success) {
    LOG(WARNING) << "NDT-D2D failed, use initial relative pose";
    refined_pose = init_relative_pose;
  } else {
    LOG(INFO) << "NDT-D2D finished, inlier_ratio=" << inlier_ratio
              << ", iterations=" << num_iterations;
  }

  Eigen::Matrix3d orientation_covariance;
  Eigen::Matrix3d position_covariance;
  ndt_d2d.CompuateCovariance(&orientation_covariance, &position_covariance);

  // LOG(ERROR) << "orientation_covariance: \n" << orientation_covariance;
  // LOG(ERROR) << "position_covariance: \n" << position_covariance;

  if (debug) {
    Eigen::Isometry3d initial_target_source = Eigen::Isometry3d::Identity();
    initial_target_source.matrix() = init_relative_pose.matrix();
    Eigen::Isometry3d optimized_target_source = Eigen::Isometry3d::Identity();
    optimized_target_source.matrix() = refined_pose.matrix();

    RegistrationVisualizationOptions options;
    options.downsample_size = 0.0;
    loop_verifier_result->topdown_images = RenderRegistrationTopdownImages(
        *to_local_map, *from_local_map, initial_target_source,
        optimized_target_source, success,
        static_cast<std::size_t>(num_iterations), inlier_ratio, options);
  }

  loop_verifier_result->refined_pose = refined_pose;
  loop_verifier_result->inlier_ratio = inlier_ratio;
  loop_verifier_result->num_iterations = num_iterations;
  loop_verifier_result->orientation_covariance = orientation_covariance;
  loop_verifier_result->position_covariance = position_covariance;
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
