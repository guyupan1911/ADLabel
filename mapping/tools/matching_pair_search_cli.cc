#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>

#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/loop_closure/base_loop_verifier.h"
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

  const std::filesystem::path data_reader_root = data_roots.front().parent_path();
  for (const auto& data_root : data_roots) {
    if (data_root.parent_path() != data_reader_root) {
      LOG(WARNING) << "data_root has different parent from first data_root: "
                   << data_root.string() << ", first parent="
                   << data_reader_root.string();
    }
  }
  auto data_reader = std::make_shared<LocalDataReader>(data_reader_root.string());
  BaseLoopVerifier loop_verifier(data_reader);

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
    const std::string frame_pair_id =
        frame_pair.from_frame().fid() + "__" + frame_pair.to_frame().fid();
    const std::string file_name = frame_pair_id + ".bin";
    const std::filesystem::path output_path = output_dir / file_name;

    std::ofstream ofs(output_path, std::ios::binary | std::ios::trunc);
    CHECK(ofs.is_open()) << "failed to open output file: " << output_path.string();
    CHECK(frame_pair.SerializeToOstream(&ofs))
        << "failed to serialize FramePair to: " << output_path.string();

    const std::filesystem::path frame_pair_dir = output_dir / frame_pair_id;
    std::filesystem::create_directories(frame_pair_dir, error);
    CHECK(!error) << "failed to create frame_pair_dir: "
                  << frame_pair_dir.string() << ", error: " << error.message();

    loop_verifier.RefineFramePairRelativePose(frame_pair);
    cv::Mat from_topdown_image;
    cv::Mat to_topdown_image;
    loop_verifier.GenerateFramePairTopdownImages(&from_topdown_image,
                                                 &to_topdown_image);

    const std::filesystem::path from_image_path =
        frame_pair_dir / "from_topdown.png";
    const std::filesystem::path to_image_path = frame_pair_dir / "to_topdown.png";
    CHECK(cv::imwrite(from_image_path.string(), from_topdown_image))
        << "failed to write image: " << from_image_path.string();
    CHECK(cv::imwrite(to_image_path.string(), to_topdown_image))
        << "failed to write image: " << to_image_path.string();

    const std::filesystem::path from_local_map_path =
        frame_pair_dir / "from_local_map.pcd";
    const std::filesystem::path to_local_map_path =
        frame_pair_dir / "to_local_map.pcd";
    CHECK(pcl::io::savePCDFileBinary(from_local_map_path.string(),
                                     *loop_verifier.GetFromLocalMap()) == 0)
        << "failed to write point cloud: " << from_local_map_path.string();
    CHECK(pcl::io::savePCDFileBinary(to_local_map_path.string(),
                                     *loop_verifier.GetToLocalMap()) == 0)
        << "failed to write point cloud: " << to_local_map_path.string();
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
