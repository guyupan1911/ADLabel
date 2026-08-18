package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "lidar_camera_fusion_cli",
    srcs = ["lidar_camera_fusion_cli.cc"],
    deps = [
        "//mapping/camera:camera",
        "//mapping/common:file",
        "//mapping/common:local_data_reader",
        "//mapping/common:pcl_types",
        "//mapping/common:pose3d",
        "//mapping/mapping_utils:frame_data",
        "//mapping/mapping_utils:simple_pose3d_interpolator",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/mapping_utils:timestamp_aligner",
        "//mapping/protos:frame_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:opencv",
    ],
)
