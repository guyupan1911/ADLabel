#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mapping/common/file.h"
#include "mapping/common/local_data_reader.h"
#include "mapping/common/pcl_types.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(data_root, "", "root directory for LocalDataReader");
DEFINE_string(lidar_metadata, "", "path to lidar frame metadata file");
DEFINE_string(output_dir, "", "directory to save colored point clouds");

namespace adlabel {
namespace mapping {
namespace {

constexpr std::uint8_t kNegativeOneRed = 255;
constexpr std::uint8_t kNegativeOneGreen = 0;
constexpr std::uint8_t kNegativeOneBlue = 0;
constexpr std::uint8_t kZeroRed = 0;
constexpr std::uint8_t kZeroGreen = 255;
constexpr std::uint8_t kZeroBlue = 0;
constexpr std::uint8_t kOtherRed = 0;
constexpr std::uint8_t kOtherGreen = 0;
constexpr std::uint8_t kOtherBlue = 255;

bool ReadInt32Labels(const LocalDataReader& data_reader, const std::string& uri,
                     std::vector<std::int32_t>* labels) {
  CHECK(labels != nullptr);
  labels->clear();

  std::vector<char> bytes;
  if (!data_reader.ReadBinaryFile(uri, &bytes)) {
    LOG(ERROR) << "failed to read lidar_nn file: " << uri;
    return false;
  }
  if (bytes.size() % sizeof(std::int32_t) != 0) {
    LOG(ERROR) << "lidar_nn file size is not int32-aligned: " << uri
               << ", size=" << bytes.size();
    return false;
  }

  labels->resize(bytes.size() / sizeof(std::int32_t));
  if (!bytes.empty()) {
    std::memcpy(labels->data(), bytes.data(), bytes.size());
  }
  return true;
}

void SetLabelColor(std::int32_t label, pcl::PointXYZRGB* point) {
  CHECK(point != nullptr);
  if (label == -1) {
    point->r = kNegativeOneRed;
    point->g = kNegativeOneGreen;
    point->b = kNegativeOneBlue;
    return;
  }
  if (label == 0) {
    point->r = kZeroRed;
    point->g = kZeroGreen;
    point->b = kZeroBlue;
    return;
  }
  point->r = kOtherRed;
  point->g = kOtherGreen;
  point->b = kOtherBlue;
}

std::filesystem::path ResolveMetadataPath(
    const std::filesystem::path& data_root,
    const std::filesystem::path& lidar_metadata) {
  if (lidar_metadata.is_absolute() || std::filesystem::exists(lidar_metadata)) {
    return lidar_metadata;
  }
  return data_root / lidar_metadata;
}

}  // namespace

bool ColorizeLidarNn(const std::string& data_root,
                     const std::string& lidar_metadata,
                     const std::string& output_dir) {
  const std::filesystem::path data_root_path(data_root);
  const std::filesystem::path metadata_path =
      ResolveMetadataPath(data_root_path, lidar_metadata);
  const std::filesystem::path output_path(output_dir);

  CHECK(std::filesystem::is_directory(data_root_path))
      << "data_root does not exist: " << data_root_path.string();
  CHECK(std::filesystem::is_regular_file(metadata_path))
      << "lidar_metadata does not exist: " << metadata_path.string();

  std::error_code error;
  std::filesystem::create_directories(output_path, error);
  CHECK(!error) << "failed to create output_dir: " << output_path.string()
                << ", error: " << error.message();

  LocalDataReader data_reader(data_root_path.string());
  const std::vector<Frame> frames = ReadMetaFile<Frame>(metadata_path.string());
  CHECK(!frames.empty()) << "no frames loaded from " << metadata_path.string();

  std::size_t saved_frames = 0;
  std::size_t skipped_frames = 0;
  std::size_t negative_one_points = 0;
  std::size_t zero_points = 0;
  std::size_t other_points = 0;

  for (const Frame& frame : frames) {
    if (!frame.has_cloud_uri() || frame.cloud_uri().empty() ||
        !frame.has_lidar_nn_uri() || frame.lidar_nn_uri().empty()) {
      LOG(WARNING) << "skip frame without cloud_uri or lidar_nn_uri: "
                   << frame.fid();
      ++skipped_frames;
      continue;
    }

    PointCloudXYZIRT::Ptr raw_cloud(new PointCloudXYZIRT);
    if (!data_reader.ReadPointCloud(frame.cloud_uri(), raw_cloud)) {
      LOG(ERROR) << "failed to read point cloud: " << frame.cloud_uri();
      ++skipped_frames;
      continue;
    }

    std::vector<std::int32_t> labels;
    if (!ReadInt32Labels(data_reader, frame.lidar_nn_uri(), &labels)) {
      ++skipped_frames;
      continue;
    }
    CHECK_EQ(labels.size(), raw_cloud->points.size())
        << "lidar_nn label count does not match point count, frame="
        << frame.fid() << ", lidar_nn_uri=" << frame.lidar_nn_uri()
        << ", cloud_uri=" << frame.cloud_uri();

    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    colored_cloud.points.resize(raw_cloud->points.size());
    colored_cloud.width = raw_cloud->width;
    colored_cloud.height = raw_cloud->height;
    colored_cloud.is_dense = raw_cloud->is_dense;

    for (std::size_t i = 0; i < raw_cloud->points.size(); ++i) {
      const auto& source = raw_cloud->points[i];
      auto& target = colored_cloud.points[i];
      target.x = source.x;
      target.y = source.y;
      target.z = source.z;
      SetLabelColor(labels[i], &target);
      if (labels[i] == -1) {
        ++negative_one_points;
      } else if (labels[i] == 0) {
        ++zero_points;
      } else {
        ++other_points;
      }
    }

    const std::filesystem::path frame_output =
        output_path / (std::to_string(frame.timestamp_ns()) + ".pcd");
    if (pcl::io::savePCDFileBinary(frame_output.string(), colored_cloud) != 0) {
      LOG(ERROR) << "failed to write colored point cloud: "
                 << frame_output.string();
      ++skipped_frames;
      continue;
    }
    ++saved_frames;
  }

  LOG(INFO) << "saved colored point clouds to " << output_path.string()
            << ", frames=" << saved_frames << ", skipped=" << skipped_frames
            << ", label_-1=" << negative_one_points
            << ", label_0=" << zero_points << ", other_labels=" << other_points;
  return saved_frames > 0;
}

}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  CHECK(!FLAGS_data_root.empty()) << "--data_root is required";
  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";

  return adlabel::mapping::ColorizeLidarNn(
             FLAGS_data_root, FLAGS_lidar_metadata, FLAGS_output_dir)
             ? 0
             : 1;
}
