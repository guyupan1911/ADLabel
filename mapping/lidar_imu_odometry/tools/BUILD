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

cc_binary(
    name = "lidar_odometry_plusai_cli",
    srcs = ["lidar_odometry_plusai_cli.cc"],
    deps = [
        "//mapping/common:file",
        "//mapping/common:local_data_reader",
        "//mapping/common:pose3d",
        "//mapping/lidar_imu_odometry/lidar_odometry:map_manager",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/protos:frame_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)
