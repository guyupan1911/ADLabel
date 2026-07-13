#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "mapping/common/file.h"
#include "mapping/loop_closure/matching_pair_searcher.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_roots, "", "comma-separated bag dump root directories");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kLidarMetadataPath[] = "metadata/lidar/lidar_plusai_unified.meta";

int Run() {
  CHECK(!FLAGS_data_roots.empty()) << "--data_roots is required";

  std::vector<std::filesystem::path> data_roots;
  std::stringstream stream(FLAGS_data_roots);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      data_roots.emplace_back(item);
    }
  }
  CHECK(!data_roots.empty()) << "--data_roots has no valid entries";

  MatchingPairSearcher searcher;

  for (const auto& data_root : data_roots) {
    const std::filesystem::path metadata_path = data_root / kLidarMetadataPath;
    const std::vector<Frame> frames = ReadMetaFile<Frame>(metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from " << metadata_path.string();

    for (const auto& frame : frames) {
      if (!frame.has_trip_id() || frame.trip_id().empty()) {
        LOG(WARNING) << "skip frame without trip_id: " << frame.fid();
        continue;
      }
      if (!frame.has_fid() || frame.fid().empty()) {
        LOG(WARNING) << "skip frame without fid, timestamp_ns=" << frame.timestamp_ns();
        continue;
      }
      searcher.AddFrame(frame);
    }
  }

  return 0;
}

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return adlabel::mapping::Run();
}
