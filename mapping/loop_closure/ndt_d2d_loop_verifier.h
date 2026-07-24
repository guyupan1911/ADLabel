#pragma once

#include <memory>
#include <utility>

#include <Eigen/Geometry>
#include <google/protobuf/repeated_ptr_field.h>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/loop_closure/proto/ndt_d2d_config.pb.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/frame_pair.pb.h"
#include "mapping/protos/pose.pb.h"
#include "mapping/registration/registration_visualization.h"

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

  RegistrationTopdownImages topdown_images;
};

class NdtD2dLoopVerifier {
 public:
  NdtD2dLoopVerifier(std::shared_ptr<LocalDataReader> local_data_reader,
                     NdtD2DConfig ndt_d2d_config)
      : local_data_reader_(std::move(local_data_reader)),
        ndt_d2d_config_(std::move(ndt_d2d_config)) {}

  Eigen::Affine3d RefineFramePairRelativePose(
      const FramePair& frame_pair, LoopVerifierResult* loop_verifier_result,
      bool debug = false);

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ToPclCloud(
      const pcl::PointCloud<PointXYZIRT>::Ptr& cloud) const;

  pcl::PointCloud<PointXYZIRT>::Ptr StitchLocalMap(
      const Frame& reference_frame,
      const google::protobuf::RepeatedPtrField<Frame>& frames,
      const google::protobuf::RepeatedPtrField<Pose3DMessage>& relative_poses);

  std::shared_ptr<LocalDataReader> local_data_reader_;
  NdtD2DConfig ndt_d2d_config_;
};

}  // namespace mapping
}  // namespace adlabel
