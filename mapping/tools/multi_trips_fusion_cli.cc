#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>

#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pose3d.h"
#include "mapping/loop_closure/ndt_d2d_loop_verifier.h"
#include "mapping/loop_closure/proto/ndt_d2d_config.pb.h"
#include "mapping/loop_closure/matching_pair_searcher.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/frame_pair.pb.h"

DEFINE_string(data_roots, "", "comma-separated bag dump root directories");
DEFINE_string(output_dir, "", "directory to save multi-trip fusion outputs");
DEFINE_string(ndt_d2d_config, "mapping/loop_closure/config/ndt_d2d_config.pb.txt",
              "path to NdtD2DConfig text proto");

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
  NdtD2DConfig ndt_d2d_config;
  std::ifstream config_file(FLAGS_ndt_d2d_config);
  CHECK(config_file.is_open())
      << "failed to open --ndt_d2d_config: " << FLAGS_ndt_d2d_config;
  std::stringstream config_buffer;
  config_buffer << config_file.rdbuf();
  CHECK(google::protobuf::TextFormat::ParseFromString(config_buffer.str(),
                                                       &ndt_d2d_config))
      << "failed to parse --ndt_d2d_config: " << FLAGS_ndt_d2d_config;
  CHECK_GT(ndt_d2d_config.resolutions_size(), 0)
      << "--ndt_d2d_config must contain at least one resolution";

  auto data_reader = std::make_shared<LocalDataReader>(data_reader_root.string());
  NdtD2dLoopVerifier loop_verifier(data_reader, ndt_d2d_config);

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
    const std::filesystem::path frame_pair_dir = output_dir / frame_pair_id;
    std::filesystem::create_directories(frame_pair_dir, error);
    CHECK(!error) << "failed to create frame_pair_dir: "
                  << frame_pair_dir.string() << ", error: " << error.message();

    LoopVerifierResult loop_verifier_result;
    loop_verifier.RefineFramePairRelativePose(frame_pair, &loop_verifier_result);

    cv::Mat merged_topdown_compare_image;
    cv::hconcat(loop_verifier_result.merge_before_refine_image,
                loop_verifier_result.merge_after_refine_image,
                merged_topdown_compare_image);

    const std::filesystem::path from_image_path =
        frame_pair_dir / "from_topdown.png";
    const std::filesystem::path to_image_path = frame_pair_dir / "to_topdown.png";
    const std::filesystem::path merged_before_image_path =
        frame_pair_dir / "merged_topdown_before.png";
    const std::filesystem::path merged_after_image_path =
        frame_pair_dir / "merged_topdown_after.png";
    const std::filesystem::path merged_compare_image_path =
        frame_pair_dir / "merged_topdown_compare.png";
    CHECK(cv::imwrite(from_image_path.string(), loop_verifier_result.source_topdown_image))
        << "failed to write image: " << from_image_path.string();
    CHECK(cv::imwrite(to_image_path.string(), loop_verifier_result.target_topdown_image))
        << "failed to write image: " << to_image_path.string();
    CHECK(cv::imwrite(merged_before_image_path.string(),
                      loop_verifier_result.merge_before_refine_image))
        << "failed to write image: " << merged_before_image_path.string();
    CHECK(cv::imwrite(merged_after_image_path.string(), loop_verifier_result.merge_after_refine_image))
        << "failed to write image: " << merged_after_image_path.string();
    CHECK(cv::imwrite(merged_compare_image_path.string(),
                      merged_topdown_compare_image))
        << "failed to write image: " << merged_compare_image_path.string();

    const auto save_point_cloud = [](const std::filesystem::path& path,
                                     const auto& cloud) {
      CHECK(cloud != nullptr) << "cannot write null point cloud: "
                              << path.string();
      CHECK(pcl::io::savePCDFileBinary(path.string(), *cloud) == 0)
          << "failed to write point cloud: " << path.string();
    };

    const std::filesystem::path from_local_map_path =
        frame_pair_dir / "from_local_map.pcd";
    const std::filesystem::path to_local_map_path =
        frame_pair_dir / "to_local_map.pcd";
    const std::filesystem::path merged_before_local_map_path =
        frame_pair_dir / "merged_local_map_before_refine.pcd";
    const std::filesystem::path merged_after_local_map_path =
        frame_pair_dir / "merged_local_map_after_refine.pcd";
    const std::filesystem::path merged_local_map_path =
        frame_pair_dir / "merged_local_map.pcd";
    save_point_cloud(from_local_map_path, loop_verifier.GetFromLocalMap());
    save_point_cloud(to_local_map_path, loop_verifier.GetToLocalMap());
    save_point_cloud(merged_before_local_map_path,
                     loop_verifier_result.merged_before_refine_cloud);
    save_point_cloud(merged_after_local_map_path,
                     loop_verifier_result.merged_after_refine_cloud);
    save_point_cloud(merged_local_map_path,
                     loop_verifier_result.merged_after_refine_cloud);
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
