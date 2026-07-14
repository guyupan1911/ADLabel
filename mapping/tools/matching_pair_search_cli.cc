#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "mapping/common/file.h"
#include "mapping/loop_closure/matching_pair_searcher.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/frame_pair.pb.h"

DEFINE_string(data_roots, "", "comma-separated bag dump root directories");
DEFINE_string(output_dir, "", "directory to save serialized FramePair proto bin files");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kLidarMetadataPath[] = "metadata/lidar/lidar_plusai_unified.meta";

int Run() {
  CHECK(!FLAGS_data_roots.empty()) << "--data_roots is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";

  std::vector<std::filesystem::path> data_roots;
  std::stringstream stream(FLAGS_data_roots);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      data_roots.emplace_back(item);
    }
  }
  CHECK(!data_roots.empty()) << "--data_roots has no valid entries";

  const std::filesystem::path output_dir(FLAGS_output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  CHECK(!error) << "failed to create output_dir: " << output_dir.string()
                << ", error: " << error.message();

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

  std::vector<FramePair> frame_pairs;
  searcher.FindFramePairs(&frame_pairs);

  for (const auto& frame_pair : frame_pairs) {
    CHECK(frame_pair.has_from_frame()) << "FramePair missing from_frame";
    CHECK(frame_pair.has_to_frame()) << "FramePair missing to_frame";
    const std::string file_name =
        frame_pair.from_frame().fid() + "__" + frame_pair.to_frame().fid() + ".bin";
    const std::filesystem::path output_path = output_dir / file_name;

    std::ofstream ofs(output_path, std::ios::binary | std::ios::trunc);
    CHECK(ofs.is_open()) << "failed to open output file: " << output_path.string();
    CHECK(frame_pair.SerializeToOstream(&ofs))
        << "failed to serialize FramePair to: " << output_path.string();
  }
  LOG(INFO) << "saved frame_pairs size: " << frame_pairs.size()
            << " to " << output_dir.string();

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
