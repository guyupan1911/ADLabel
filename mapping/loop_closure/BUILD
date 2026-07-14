package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "matching_pair_searcher",
    srcs = ["matching_pair_searcher.cc"],
    hdrs = ["matching_pair_searcher.h"],
    deps = [
        "//mapping/protos:frame_cc_proto",
        "//mapping/protos:frame_pair_cc_proto",
        "//mapping/common:pose3d",
        "@system_libs//:glog",
        "@system_libs//:opencv",
        "@system_libs//:eigen",
        "@system_libs//:geographiclib",
    ],
)

cc_library(
    name = "base_loop_verifier",
    srcs = ["base_loop_verifier.cc"],
    hdrs = ["base_loop_verifier.h"],
    deps = [
        "//mapping/common:local_data_reader",
        "//mapping/common:pose3d",
        "//mapping/lidar_topdown:lidar_topdown",
        "//mapping/mapping_utils:frame_data",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/protos:frame_cc_proto",
        "//mapping/protos:frame_pair_cc_proto",
        "//mapping/protos:pose_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:geographiclib",
        "@system_libs//:glog",
        "@system_libs//:opencv",
    ],
)
