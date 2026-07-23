#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/scoped_timer.h"
#include "mapping/registration/registration_visualization.h"
#include "mapping/registration/small_gicp_utils.h"

DEFINE_string(cloud_source, "", "Path to the source PCD file.");
DEFINE_string(cloud_target, "", "Path to the target PCD file.");
DEFINE_double(leaf_size, 0.25, "Voxel-grid leaf size in meters.");
DEFINE_int32(num_neighbors, 20,
             "Number of neighbors for covariance estimation.");
DEFINE_string(output_dir, "", "Directory for topdown registration images.");

namespace {

struct PreparedCloud {
  adlabel::mapping::PointCloudXYZIRT original_cloud;
  small_gicp::PointCloud::Ptr cloud;
  adlabel::mapping::SmallGicpKdTree::Ptr kdtree;
};

PreparedCloud PrepareCloud(const std::string& name, const std::string& path) {
  adlabel::mapping::PointCloudXYZIRT pcl_cloud;
  {
    adlabel::mapping::ScopedTimer timer(name + " PCD I/O");
    CHECK_EQ(
        pcl::io::loadPCDFile<adlabel::mapping::PointXYZIRT>(path, pcl_cloud), 0)
        << "Failed to load PCD file: " << path;
  }
  CHECK(!pcl_cloud.empty()) << "PCD file contains no points: " << path;

  small_gicp::PointCloud::Ptr raw_cloud;
  {
    adlabel::mapping::ScopedTimer timer(name + " point cloud conversion");
    raw_cloud = adlabel::mapping::ToSmallGicpPointCloud(pcl_cloud);
  }
  CHECK(raw_cloud);
  CHECK(!raw_cloud->empty())
      << "PCD file contains no finite XYZ points: " << path;

  PreparedCloud prepared;
  {
    adlabel::mapping::ScopedTimer timer(name + " voxel downsampling");
    prepared.cloud =
        adlabel::mapping::VoxelGridDownsample(*raw_cloud, FLAGS_leaf_size);
  }
  CHECK(prepared.cloud);
  CHECK(!prepared.cloud->empty())
      << "Downsampling produced an empty point cloud: " << path;

  {
    adlabel::mapping::ScopedTimer timer(name + " KD-tree construction");
    prepared.kdtree = adlabel::mapping::BuildKdTree(prepared.cloud);
  }
  CHECK(prepared.kdtree);

  {
    adlabel::mapping::ScopedTimer timer(name + " covariance estimation");
    adlabel::mapping::EstimateCovariances(*prepared.cloud, *prepared.kdtree,
                                          FLAGS_num_neighbors);
  }

  LOG(INFO) << name << " points: input=" << pcl_cloud.size()
            << ", finite=" << raw_cloud->size()
            << ", downsampled=" << prepared.cloud->size();
  prepared.original_cloud = std::move(pcl_cloud);
  return prepared;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_cloud_source.empty()) << "--cloud_source is required";
  CHECK(!FLAGS_cloud_target.empty()) << "--cloud_target is required";
  CHECK(std::isfinite(FLAGS_leaf_size) && FLAGS_leaf_size > 0.0)
      << "--leaf_size must be finite and positive";
  CHECK_GT(FLAGS_num_neighbors, 0) << "--num_neighbors must be positive";
  CHECK(!FLAGS_output_dir.empty()) << "--output_dir is required";

  const PreparedCloud source = PrepareCloud("source", FLAGS_cloud_source);
  const PreparedCloud target = PrepareCloud("target", FLAGS_cloud_target);

  const Eigen::Isometry3d initial_target_source = Eigen::Isometry3d::Identity();
  adlabel::mapping::SmallGicpRegistrationResult result;
  {
    adlabel::mapping::ScopedTimer timer("GICP registration");
    result = adlabel::mapping::AlignGicp(*target.cloud, *source.cloud,
                                         *target.kdtree, initial_target_source);
  }

  LOG(INFO) << "GICP result: converged=" << result.registration.converged
            << ", iterations=" << result.registration.iterations
            << ", inlier_ratio=" << result.inlier_ratio
            << ", error=" << result.registration.error;
  LOG(INFO) << "T_target_source:\n"
            << result.registration.T_target_source.matrix();

  adlabel::mapping::RegistrationTopdownImages images;
  {
    adlabel::mapping::ScopedTimer timer("topdown visualization");
    images = adlabel::mapping::RenderRegistrationTopdownImages(
        target.original_cloud, source.original_cloud, initial_target_source,
        result);
  }

  CHECK(
      adlabel::mapping::SaveRegistrationTopdownImages(images, FLAGS_output_dir))
      << "Failed to save registration topdown images to " << FLAGS_output_dir;
  LOG(INFO) << "Saved registration topdown images to " << FLAGS_output_dir;
  return 0;
}
