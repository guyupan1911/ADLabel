#pragma once

#include <memory>

#include "mapping/common/local_data_reader.h"
#include "mapping/mapping_utils/frame_data.h"
#include "mapping/protos/frame.pb.h"

namespace adlabel {
namespace mapping {

bool GenerateLidarFrameData(
    const Frame& frame, FrameData* lidar_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader);

bool GenerateCameraFrameData(
    const Frame& frame, FrameData* camera_frame_data,
    const std::shared_ptr<LocalDataReader>& local_data_reader);

}  // namespace mapping
}  // namespace adlabel
