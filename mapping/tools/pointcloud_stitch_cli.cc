#include <vector>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "mapping/common/file.h"
#include "mapping/common/pcl_types.h"
#include "mapping/protos/frame.pb.h"

DEFINE_string(lidar_metadata,
  "/workspace/data/20260503T173334_pdb-l4e-c0002_016_40to60/metadata/lidar/"
  "em4_front_lidar.meta", "path to lidar metadata");
DEFINE_string(data_root, "/workspace/data", "root directory for sensor data");

using namespace adlabel::mapping;

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  auto lidar_frames = ReadMetaFile<Frame>(FLAGS_lidar_metadata);
  LOG(INFO) << "lidar_frames size: " << lidar_frames.size();

  for (const auto& frame : lidar_frames) {
    const std::string pcd_path = FLAGS_data_root + "/" + frame.cloud_uri();

    PointCloudXYZIRT::Ptr cloud(new PointCloudXYZIRT);
    if (pcl::io::loadPCDFile<PointXYZIRT>(pcd_path, *cloud) < 0) {
      LOG(ERROR) << "failed to load pcd: " << pcd_path;
      continue;
    }
    LOG(INFO) << "loaded " << cloud->size() << " points from " << pcd_path;

    // example: passthrough filter to remove ground points below -2m
    pcl::PassThrough<PointXYZIRT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 50.0);
    PointCloudXYZIRT::Ptr filtered(new PointCloudXYZIRT);
    pass.filter(*filtered);

    // example: voxel grid downsample at 10cm
    pcl::VoxelGrid<PointXYZIRT> vg;
    vg.setInputCloud(filtered);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    PointCloudXYZIRT::Ptr downsampled(new PointCloudXYZIRT);
    vg.filter(*downsampled);

    LOG(INFO) << "after filter: " << filtered->size()
              << " after voxel: " << downsampled->size();
  }

  return 0;
}