#pragma once

#include <memory>
#include <utility>

#include <google/protobuf/repeated_ptr_field.h>
#include <opencv2/core.hpp>

#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/protos/frame_pair.pb.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/pose.pb.h"

namespace adlabel {
namespace mapping {

class BaseLoopVerifier {
 public:
  explicit BaseLoopVerifier(std::shared_ptr<LocalDataReader> local_data_reader)
    : local_data_reader_(std::move(local_data_reader)) {}

  void RefineFramePairRelativePose(const FramePair& frame_pair);

  void GenerateFramePairTopdownImages(cv::Mat* from_image, cv::Mat* to_image);

  pcl::PointCloud<PointXYZIRT>::Ptr GetFromLocalMap() const {
    return from_local_map_;
  }

  pcl::PointCloud<PointXYZIRT>::Ptr GetToLocalMap() const {
    return to_local_map_;
  }

 private:
  pcl::PointCloud<PointXYZIRT>::Ptr StitchLocalMap(
    const Frame& reference_frame,
    const google::protobuf::RepeatedPtrField<Frame>& frames,
    const google::protobuf::RepeatedPtrField<Pose3DMessage>& relative_poses);

  std::shared_ptr<LocalDataReader> local_data_reader_;
  pcl::PointCloud<PointXYZIRT>::Ptr from_local_map_;
  pcl::PointCloud<PointXYZIRT>::Ptr to_local_map_;
};

}
}