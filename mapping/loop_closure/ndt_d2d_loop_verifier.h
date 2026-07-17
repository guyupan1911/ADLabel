#pragma once

#include <memory>
#include <utility>

#include <google/protobuf/repeated_ptr_field.h>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/loop_closure/proto/ndt_d2d_config.pb.h"
#include "mapping/protos/frame_pair.pb.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/pose.pb.h"

namespace adlabel {
namespace mapping {

struct LoopVerifierResult {
  // Relative pose that transforms points from from_frame local map into
  // to_frame local map.
  Eigen::Affine3d refined_pose = Eigen::Affine3d::Identity();
  double inlier_ratio = 0.0;
  int num_iterations = 0;

  Eigen::Matrix3d orientation_covariance;
  Eigen::Matrix3d position_covariance;

  cv::Mat source_topdown_image;
  cv::Mat target_topdown_image;
  cv::Mat merge_before_refine_image;
  cv::Mat merge_after_refine_image;
  pcl::PointCloud<PointXYZIRT>::Ptr merged_before_refine_cloud;
  pcl::PointCloud<PointXYZIRT>::Ptr merged_after_refine_cloud;
};

class NdtD2dLoopVerifier {
 public:
  NdtD2dLoopVerifier(std::shared_ptr<LocalDataReader> local_data_reader,
                       NdtD2DConfig ndt_d2d_config)
      : local_data_reader_(std::move(local_data_reader)),
        ndt_d2d_config_(std::move(ndt_d2d_config)) {}

  Eigen::Affine3d RefineFramePairRelativePose(
      const FramePair& frame_pair, LoopVerifierResult* loop_verifier_result);

  void GenerateFramePairTopdownImages(
      const Eigen::Affine3d& init_pose_relative, cv::Mat* from_image,
      cv::Mat* to_image, cv::Mat* merged_image);

  pcl::PointCloud<PointXYZIRT>::Ptr GetFromLocalMap() const {
    return from_local_map_;
  }

  pcl::PointCloud<PointXYZIRT>::Ptr GetToLocalMap() const {
    return to_local_map_;
  }

  const Eigen::Affine3d& GetInitRelativePose() const {
    return init_relative_pose_;
  }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ToPclCloud(
      const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) const;

  pcl::PointCloud<PointXYZIRT>::Ptr StitchLocalMap(
      const Frame& reference_frame,
      const google::protobuf::RepeatedPtrField<Frame>& frames,
      const google::protobuf::RepeatedPtrField<Pose3DMessage>& relative_poses);

  std::shared_ptr<LocalDataReader> local_data_reader_;
  NdtD2DConfig ndt_d2d_config_;
  pcl::PointCloud<PointXYZIRT>::Ptr from_local_map_;
  pcl::PointCloud<PointXYZIRT>::Ptr to_local_map_;
  Eigen::Affine3d init_relative_pose_ = Eigen::Affine3d::Identity();
};

}
}