#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "mapping/protos/frame.pb.h"

namespace adlabel {
namespace mapping {

struct FrameRef {
    int64_t timestamp_ns = 0;
    size_t index = 0;
    // Matched frame timestamp minus reference frame timestamp.
    int64_t time_diff_ns = 0;
};

struct AlignedFrames {
    FrameRef reference_frame;
    std::unordered_map<std::string, FrameRef> aligned_frames;
};

using SensorFrameRefs = std::unordered_map<std::string, std::vector<FrameRef>>;

class TimestampAligner {
  public:
    explicit TimestampAligner(int64_t max_time_diff_ns);

    void AddSensorFrameRefs(const std::string& sensor_name,
                            const std::vector<Frame>& frames);

    std::vector<AlignedFrames> Align(const std::string& main_sensor_name,
                                     bool require_all_sensors = false) const;

  private:
    bool FindNearest(const std::vector<FrameRef>& target_frame_refs,
                     int64_t reference_timestamp_ns,
                     FrameRef* matched_frame_ref) const;

    SensorFrameRefs sensor_frame_refs_;
    int64_t max_time_diff_ns_ = 0;
};

}  // namespace mapping
}  // namespace adlabel
