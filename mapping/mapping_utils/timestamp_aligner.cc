#include "mapping/mapping_utils/timestamp_aligner.h"

#include <algorithm>
#include <cstdlib>
#include <iterator>

#include <glog/logging.h>

namespace adlabel {
namespace mapping {

TimestampAligner::TimestampAligner(int64_t max_time_diff_ns)
    : max_time_diff_ns_(max_time_diff_ns) {
    CHECK_GE(max_time_diff_ns_, 0) << "max_time_diff_ns must be non-negative";
}

void TimestampAligner::AddSensorFrameRefs(const std::string& sensor_name,
                                          const std::vector<Frame>& frames) {
    auto& frame_refs = sensor_frame_refs_[sensor_name];
    frame_refs.clear();
    frame_refs.reserve(frames.size());

    size_t skipped_without_timestamp = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (!frames[i].has_timestamp_ns()) {
            ++skipped_without_timestamp;
            LOG(WARNING) << "skip " << sensor_name
                         << " frame without timestamp_ns: " << frames[i].fid();
            continue;
        }
        frame_refs.push_back(FrameRef{frames[i].timestamp_ns(), i, 0});
    }

    std::sort(frame_refs.begin(), frame_refs.end(), [](const FrameRef& a,
                                                       const FrameRef& b) {
        return a.timestamp_ns < b.timestamp_ns;
    });

    LOG(INFO) << "add sensor: " << sensor_name
              << ", frames=" << frame_refs.size()
              << ", skipped_without_timestamp=" << skipped_without_timestamp;
}

std::vector<AlignedFrames> TimestampAligner::Align(
        const std::string& main_sensor_name, bool require_all_sensors) const {
    const auto main_sensor_it = sensor_frame_refs_.find(main_sensor_name);
    if (main_sensor_it == sensor_frame_refs_.end()) {
        LOG(ERROR) << "failed to find main sensor: " << main_sensor_name;
        return {};
    }

    const size_t target_sensor_count =
            sensor_frame_refs_.empty() ? 0 : sensor_frame_refs_.size() - 1;

    std::vector<AlignedFrames> aligned_results;
    aligned_results.reserve(main_sensor_it->second.size());

    for (const auto& main_frame_ref : main_sensor_it->second) {
        AlignedFrames aligned;
        aligned.reference_frame = main_frame_ref;

        for (auto it = sensor_frame_refs_.begin(); it != sensor_frame_refs_.end(); ++it) {
            if (it->first == main_sensor_name) {
                continue;
            }

            FrameRef matched_target_frame_ref;
            if (!FindNearest(it->second,
                             main_frame_ref.timestamp_ns,
                             &matched_target_frame_ref)) {
                continue;
            }
            aligned.aligned_frames[it->first] = matched_target_frame_ref;
        }

        if (require_all_sensors &&
            aligned.aligned_frames.size() != target_sensor_count) {
            continue;
        }
        if (!aligned.aligned_frames.empty()) {
            aligned_results.push_back(aligned);
        }
    }

    LOG(INFO) << "aligned frame groups: " << aligned_results.size()
              << ", main_sensor=" << main_sensor_name
              << ", require_all_sensors=" << require_all_sensors;
    return aligned_results;
}

bool TimestampAligner::FindNearest(const std::vector<FrameRef>& target_frame_refs,
                                   int64_t reference_timestamp_ns,
                                   FrameRef* matched_frame_ref) const {
    CHECK(matched_frame_ref != nullptr);
    if (target_frame_refs.empty()) {
        return false;
    }

    auto it = std::lower_bound(
            target_frame_refs.begin(), target_frame_refs.end(), reference_timestamp_ns,
            [](const FrameRef& ref, int64_t target_timestamp_ns) {
                return ref.timestamp_ns < target_timestamp_ns;
            });

    const FrameRef* best = nullptr;
    if (it != target_frame_refs.end()) {
        best = &(*it);
    }

    if (it != target_frame_refs.begin()) {
        const FrameRef* previous = &(*std::prev(it));
        if (best == nullptr ||
            std::llabs(previous->timestamp_ns - reference_timestamp_ns) <
                    std::llabs(best->timestamp_ns - reference_timestamp_ns)) {
            best = previous;
        }
    }

    if (best == nullptr) {
        return false;
    }

    const int64_t time_diff_ns = best->timestamp_ns - reference_timestamp_ns;
    if (std::llabs(time_diff_ns) > max_time_diff_ns_) {
        LOG(INFO) << "time_diff=" << static_cast<double>(time_diff_ns) / 1e6
                  << " ms is larger than threshold "
                  << static_cast<double>(max_time_diff_ns_) / 1e6
                  << " ms, drop frame pair";
        return false;
    }

    *matched_frame_ref = *best;
    matched_frame_ref->time_diff_ns = time_diff_ns;
    return true;
}

}  // namespace mapping
}  // namespace adlabel
