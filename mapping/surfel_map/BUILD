package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "surfel_map",
    srcs = ["surfel_map.cc"],
    hdrs = ["surfel_map.h"],
    deps = [
        "//mapping/lidar_topdown:lidar_topdown",
        "@system_libs//:eigen",
        "@system_libs//:glog",
        "@system_libs//:opencv",
        "@system_libs//:pcl",
    ],
)

cc_library(
    name = "surfel_map_builder",
    srcs = ["surfel_map_builder.cc"],
    hdrs = ["surfel_map_builder.h"],
    deps = [
        ":surfel_map",
        "//mapping/camera:camera",
        "//mapping/common:local_data_reader",
        "//mapping/common:pcl_types",
        "//mapping/common:pose3d",
        "//mapping/mapping_utils:frame_data",
        "//mapping/mapping_utils:simple_pose3d_interpolator",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/mapping_utils:timestamp_aligner",
        "//mapping/protos:frame_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:glog",
        "@system_libs//:opencv",
    ],
)
