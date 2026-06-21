#pragma once

#include <memory>

#include "mapping/common/local_data_reader.h"
#include "mapping/mapping_utils/frame_data.h"
#include "mapping/mapping_utils/simple_pose3d_interpolator.h"
#include "mapping/protos/frame.pb.h"

namespace adlabel {
namespace mapping {

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader,
    const SimplePose3DInterpolator* lio_pose_interpolator = nullptr,
    bool use_lio_pose = false);

}  // namespace mapping
}  // namespace adlabel
