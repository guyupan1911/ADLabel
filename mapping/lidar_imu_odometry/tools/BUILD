load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "lidar_odometry_kitti_cli",
    srcs = ["lidar_odometry_kitti_cli.cc"],
    deps = [
        "//mapping/lidar_imu_odometry/lidar_odometry:map_manager",
        "@system_libs//:eigen",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)
