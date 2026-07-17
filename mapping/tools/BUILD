package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_binary")


cc_binary(
    name = "pointcloud_stitch_cli",
    srcs= [
      "pointcloud_stitch_cli.cc",
    ],
    deps = [
      "//mapping/common:local_data_reader",
      "//mapping/common:pcl_types",
      "//mapping/common:pose3d",
      "//mapping/mapping_utils:stitch_utils",
      "//mapping/protos:frame_cc_proto",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:pcl",
    ],
)

cc_binary(
    name = "lidar_topdown_stitcher_cli",
    srcs = [
      "lidar_topdown_stitcher_cli.cc",
    ],
    deps = [
      "//mapping/common:local_data_reader",
      "//mapping/common:pose3d",
      "//mapping/lidar_topdown:lidar_topdown",
      "//mapping/mapping_utils:stitch_utils",
      "//mapping/protos:frame_cc_proto",
      "@system_libs//:eigen",
      "@system_libs//:geographiclib",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:opencv",
    ],
)

cc_binary(
    name = "lidar_topdown_stitcher_cli_multi_trips_cli",
    srcs = [
      "lidar_topdown_stitcher_cli_multi_trips_cli.cc",
    ],
    deps = [
      "//mapping/common:local_data_reader",
      "//mapping/common:pose3d",
      "//mapping/lidar_topdown:lidar_topdown",
      "//mapping/mapping_utils:stitch_utils",
      "//mapping/protos:frame_cc_proto",
      "@system_libs//:eigen",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:opencv",
    ],
)

cc_binary(
    name = "lidar_camera_fusion_cli",
    srcs = [
      "lidar_camera_fusion_cli.cc",
    ],
    deps = [
      "//mapping/camera:camera",
      "//mapping/common:local_data_reader",
      "//mapping/common:pcl_types",
      "//mapping/common:pose3d",
      "//mapping/mapping_utils:simple_pose3d_interpolator",
      "//mapping/mapping_utils:timestamp_aligner",
      "//mapping/protos:frame_cc_proto",
      "@system_libs//:eigen",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:opencv",
    ],
)

cc_binary(
    name = "multi_trips_fusion_cli",
    srcs = [
      "multi_trips_fusion_cli.cc",
    ],
    deps = [
      "//mapping/common:file",
      "//mapping/common:local_data_reader",
      "//mapping/loop_closure:ndt_d2d_loop_verifier",
      "//mapping/loop_closure:matching_pair_searcher",
      "//mapping/loop_closure:pose_graph_optimizer",
      "//mapping/loop_closure/proto:ndt_d2d_config_cc_proto",
      "//mapping/protos:frame_cc_proto",
      "//mapping/protos:frame_pair_cc_proto",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:opencv",
      "@system_libs//:pcl",
    ],
)

cc_binary(
    name = "pose_optimize_cli",
    srcs = [
      "pose_optimize_cli.cc",
    ],
    deps = [
      "//mapping/common:file",
      "//mapping/protos:frame_cc_proto",
      "//mapping/protos:localization_cc_proto",
      "@local_libs//:gtsam",
      "@system_libs//:eigen",
      "@system_libs//:gflags",
      "@system_libs//:glog",
    ],
)
