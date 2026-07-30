#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "mapping/common/local_data_reader.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/surfel_map/surfel_map.h"

namespace adlabel {
namespace mapping {

struct SurfelMapBuilderOptions {
  SurfelMapOptions surfel_map_options;
  int64_t maximum_time_difference_ns = 10 * 1000 * 1000;
  size_t maximum_frames = 0;
};

class SurfelMapBuilder {
 public:
  SurfelMapBuilder(const SurfelMapBuilderOptions& options,
                   std::shared_ptr<LocalDataReader> data_reader,
                   std::vector<Frame> lidar_frames,
                   std::vector<Frame> camera_frames);

  bool Build();

  const SurfelMap& GetSurfelMap() const { return surfel_map_; }
  const Eigen::Affine3d& GetMapFromEcef() const { return T_map_ecef_; }

 private:
  bool InitializeTrajectory();
  bool BuildGeometry();
  bool BuildTexture();
  bool InterpolateCameraPose(int64_t timestamp_ns,
                             Eigen::Affine3d* pose_ecef) const;

  SurfelMapBuilderOptions options_;
  std::shared_ptr<LocalDataReader> data_reader_;
  std::vector<Frame> lidar_frames_;
  std::vector<Frame> camera_frames_;

  SimplePose3DInterpolator pose_ecef_interpolator_;
  Eigen::Affine3d T_map_ecef_ = Eigen::Affine3d::Identity();
  SurfelMap surfel_map_;
};

}  // namespace mapping
}  // namespace adlabel
