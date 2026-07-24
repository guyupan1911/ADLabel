#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "mapping/common/scoped_timer.h"
#include "mapping/registration/small_gicp_adapter.h"

DEFINE_string(pcd_path, "", "Path to the input PCD file.");
DEFINE_double(leaf_size, 0.25, "Voxel-grid leaf size in meters.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pcd_path.empty()) << "--pcd_path is required";

  adlabel::mapping::PointCloudXYZIRT input;
  {
    adlabel::mapping::ScopedTimer timer("PCD I/O");
    pcl::io::loadPCDFile<adlabel::mapping::PointXYZIRT>(FLAGS_pcd_path, input);
  }

  const auto raw_cloud = adlabel::mapping::ToSmallGicpPointCloud(input);

  adlabel::mapping::SmallGicpPointCloudPtr downsampled_cloud;
  {
    adlabel::mapping::ScopedTimer timer("small_gicp voxel downsampling");
    downsampled_cloud =
        adlabel::mapping::VoxelGridDownsample(*raw_cloud, FLAGS_leaf_size);
  }

  const auto pcl_input = input.makeShared();
  adlabel::mapping::PointCloudXYZIRT pcl_downsampled_cloud;
  {
    adlabel::mapping::ScopedTimer timer("PCL voxel downsampling");
    pcl::VoxelGrid<adlabel::mapping::PointXYZIRT> voxel_grid;
    const float leaf_size = static_cast<float>(FLAGS_leaf_size);
    voxel_grid.setInputCloud(pcl_input);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(pcl_downsampled_cloud);
  }

  LOG(INFO) << "Input points: " << raw_cloud->size();
  LOG(INFO) << "small_gicp downsampled points: " << downsampled_cloud->size();
  LOG(INFO) << "PCL downsampled points: " << pcl_downsampled_cloud.size();

  return 0;
}
