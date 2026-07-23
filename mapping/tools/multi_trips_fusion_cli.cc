#include <cstddef>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
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
#include "mapping/loop_closure/matching_pair_searcher.h"
#include "mapping/loop_closure/ndt_d2d_loop_verifier.h"
#include "mapping/loop_closure/pose_graph_optimizer.h"
#include "mapping/loop_closure/proto/ndt_d2d_config.pb.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/frame_pair.pb.h"

DEFINE_string(data_roots, "", "comma-separated bag dump root directories");
DEFINE_string(output_dir, "", "directory to save multi-trip fusion outputs");
DEFINE_string(ndt_d2d_config,
              "mapping/loop_closure/config/ndt_d2d_config.pb.txt",
              "path to NdtD2DConfig text proto");
DEFINE_bool(debug, false,
            "generate and save loop verifier visualization artifacts");

namespace adlabel {
namespace mapping {
namespace {

constexpr char kLidarMetadataPath[] =
    "metadata/lidar/lidar_plusai_unified.meta";

void WriteOptimizedFramesByTrip(const std::filesystem::path& output_dir,
                                const std::vector<Frame>& optimized_frames) {
  const std::filesystem::path by_trip_dir =
      output_dir / "pose_graph_optimized_frames_by_trip";
  std::error_code error;
  std::filesystem::create_directories(by_trip_dir, error);
  CHECK(!error) << "failed to create optimized frame by-trip dir: "
                << by_trip_dir.string() << ", error: " << error.message();

  std::map<std::string, std::vector<Frame>> frames_by_trip;
  for (const Frame& frame : optimized_frames) {
    CHECK(frame.has_trip_id() && !frame.trip_id().empty())
        << "optimized frame missing trip_id: " << frame.fid();
    frames_by_trip[frame.trip_id()].push_back(frame);
  }

  for (const auto& trip_id_and_frames : frames_by_trip) {
    const std::filesystem::path trip_frames_path =
        by_trip_dir / (trip_id_and_frames.first + ".meta");
    CHECK(WriteMetaFile(trip_frames_path.string(), trip_id_and_frames.second))
        << "failed to write optimized frames for trip "
        << trip_id_and_frames.first << ": " << trip_frames_path.string();
    LOG(INFO) << "saved optimized frames for trip " << trip_id_and_frames.first
              << " to " << trip_frames_path.string()
              << ", frames=" << trip_id_and_frames.second.size();
  }
}

void WritePoseGraphResidualSummary(const std::filesystem::path& path,
                                   const PoseGraphOptimizationResult& result) {
  std::ofstream ofs(path);
  CHECK(ofs.is_open()) << "failed to open pose graph residual summary: "
                       << path.string();
  ofs << "node_count " << result.node_count << "\n";
  ofs << "prior_factor_count " << result.prior_factor_count << "\n";
  ofs << "between_factor_count " << result.between_factor_count << "\n";
  ofs << "loop_closure_factor_count " << result.loop_closure_factor_count
      << "\n";
  ofs << "initial_graph_error " << result.initial_graph_error << "\n";
  ofs << "optimized_graph_error " << result.optimized_graph_error << "\n";
  ofs << "initial_mean_prior_translation_residual "
      << result.initial_residuals.mean_prior_translation_residual << "\n";
  ofs << "optimized_mean_prior_translation_residual "
      << result.optimized_residuals.mean_prior_translation_residual << "\n";
  ofs << "initial_mean_translation_residual "
      << result.initial_residuals.mean_translation_residual << "\n";
  ofs << "optimized_mean_translation_residual "
      << result.optimized_residuals.mean_translation_residual << "\n";
  ofs << "initial_mean_rotation_residual "
      << result.initial_residuals.mean_rotation_residual << "\n";
  ofs << "optimized_mean_rotation_residual "
      << result.optimized_residuals.mean_rotation_residual << "\n";
  ofs << "initial_mean_loop_closure_residual "
      << result.initial_residuals.mean_loop_closure_residual << "\n";
  ofs << "optimized_mean_loop_closure_residual "
      << result.optimized_residuals.mean_loop_closure_residual << "\n";
  CHECK(ofs.good()) << "failed to write pose graph residual summary: "
                    << path.string();
}

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

  const std::filesystem::path data_reader_root =
      data_roots.front().parent_path();
  for (const auto& data_root : data_roots) {
    if (data_root.parent_path() != data_reader_root) {
      LOG(WARNING) << "data_root has different parent from first data_root: "
                   << data_root.string()
                   << ", first parent=" << data_reader_root.string();
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

  auto data_reader =
      std::make_shared<LocalDataReader>(data_reader_root.string());
  NdtD2dLoopVerifier loop_verifier(data_reader, ndt_d2d_config);

  MatchingPairSearcher searcher;
  PoseGraphOptimizer pose_graph_optimizer;
  std::unordered_map<std::string, Frame> frame_by_id;

  for (const auto& data_root : data_roots) {
    const std::filesystem::path metadata_path = data_root / kLidarMetadataPath;
    const std::vector<Frame> frames =
        ReadMetaFile<Frame>(metadata_path.string());
    CHECK(!frames.empty()) << "no frames loaded from "
                           << metadata_path.string();
    for (const auto& frame : frames) {
      if (!frame.has_trip_id() || frame.trip_id().empty()) {
        LOG(WARNING) << "skip frame without trip_id: " << frame.fid();
        continue;
      }
      if (!frame.has_fid() || frame.fid().empty()) {
        LOG(WARNING) << "skip frame without fid, timestamp_ns="
                     << frame.timestamp_ns();
        continue;
      }
      searcher.AddFrame(frame);
      frame_by_id[frame.fid()] = frame;
    }
  }

  std::vector<FramePair> frame_pairs;
  searcher.FindFramePairs(&frame_pairs);

  for (const auto& frame_pair : frame_pairs) {
    CHECK(frame_pair.has_from_frame()) << "FramePair missing from_frame";
    CHECK(frame_pair.has_to_frame()) << "FramePair missing to_frame";
    LoopVerifierResult loop_verifier_result;
    loop_verifier.RefineFramePairRelativePose(frame_pair, &loop_verifier_result,
                                              FLAGS_debug);

    auto from_frame_iter = frame_by_id.find(frame_pair.from_frame().fid());
    CHECK(from_frame_iter != frame_by_id.end())
        << "unknown from frame id: " << frame_pair.from_frame().fid();
    MatchedFrame* matched_frame = from_frame_iter->second.add_matched_frames();
    matched_frame->set_frame_id(frame_pair.to_frame().fid());
    matched_frame->mutable_relative_pose()->CopyFrom(
        Pose3D(loop_verifier_result.refined_pose).GetPose3DMessage());
    matched_frame->set_inlier_ratio(loop_verifier_result.inlier_ratio);

    if (FLAGS_debug) {
      const std::string frame_pair_id =
          frame_pair.from_frame().fid() + "__" + frame_pair.to_frame().fid();
      const std::filesystem::path frame_pair_dir = output_dir / frame_pair_id;
      std::filesystem::create_directories(frame_pair_dir, error);
      CHECK(!error) << "failed to create frame_pair_dir: "
                    << frame_pair_dir.string()
                    << ", error: " << error.message();

      cv::Mat merged_topdown_compare_image;
      cv::hconcat(loop_verifier_result.merge_before_refine_image,
                  loop_verifier_result.merge_after_refine_image,
                  merged_topdown_compare_image);

      const std::filesystem::path from_image_path =
          frame_pair_dir / "from_topdown.png";
      const std::filesystem::path to_image_path =
          frame_pair_dir / "to_topdown.png";
      const std::filesystem::path merged_before_image_path =
          frame_pair_dir / "merged_topdown_before.png";
      const std::filesystem::path merged_after_image_path =
          frame_pair_dir / "merged_topdown_after.png";
      const std::filesystem::path merged_compare_image_path =
          frame_pair_dir / "merged_topdown_compare.png";
      CHECK(cv::imwrite(from_image_path.string(),
                        loop_verifier_result.source_topdown_image))
          << "failed to write image: " << from_image_path.string();
      CHECK(cv::imwrite(to_image_path.string(),
                        loop_verifier_result.target_topdown_image))
          << "failed to write image: " << to_image_path.string();
      CHECK(cv::imwrite(merged_before_image_path.string(),
                        loop_verifier_result.merge_before_refine_image))
          << "failed to write image: " << merged_before_image_path.string();
      CHECK(cv::imwrite(merged_after_image_path.string(),
                        loop_verifier_result.merge_after_refine_image))
          << "failed to write image: " << merged_after_image_path.string();
      CHECK(cv::imwrite(merged_compare_image_path.string(),
                        merged_topdown_compare_image))
          << "failed to write image: " << merged_compare_image_path.string();

      const auto save_point_cloud = [](const std::filesystem::path& path,
                                       const auto& cloud) {
        CHECK(cloud != nullptr)
            << "cannot write null point cloud: " << path.string();
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
  }

  for (const auto& frame_id_and_frame : frame_by_id) {
    pose_graph_optimizer.AddFrame(frame_id_and_frame.second);
  }

  PoseGraphOptimizationResult pose_graph_result;
  CHECK(pose_graph_optimizer.Optimize(&pose_graph_result))
      << "failed to optimize pose graph";

  const std::filesystem::path optimized_frames_path =
      output_dir / "pose_graph_optimized_frames.meta";
  CHECK(WriteMetaFile(optimized_frames_path.string(),
                      pose_graph_result.optimized_frames))
      << "failed to write optimized frames: " << optimized_frames_path.string();
  WriteOptimizedFramesByTrip(output_dir, pose_graph_result.optimized_frames);
  WritePoseGraphResidualSummary(output_dir / "pose_graph_residuals.txt",
                                pose_graph_result);

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
