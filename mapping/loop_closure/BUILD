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
    name = "voxel_grid_covariance",
    hdrs = ["voxel_grid_covariance.h"],
    defines = ["USE_UNORDERED_MAP=0"],
    deps = [
        "@system_libs//:pcl",
    ],
)

cc_library(
    name = "ndt_d2d",
    srcs = ["ndt_d2d.cc"],
    hdrs = [
        "ndt_d2d.h",
        "d2d_cost_functor.h",
    ],
    defines = ["USE_UNORDERED_MAP=0"],
    deps = [
        ":voxel_grid_covariance",
        "//mapping/loop_closure/proto:ndt_d2d_config_cc_proto",
        "@system_libs//:ceres",
        "@system_libs//:eigen",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)

cc_library(
    name = "ndt_d2d_loop_verifier",
    srcs = ["ndt_d2d_loop_verifier.cc"],
    hdrs = ["ndt_d2d_loop_verifier.h"],
    deps = [
        ":ndt_d2d",
        "//mapping/loop_closure/proto:ndt_d2d_config_cc_proto",
        "//mapping/common:local_data_reader",
        "//mapping/common:pose3d",
        "//mapping/mapping_utils:frame_data",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/protos:frame_cc_proto",
        "//mapping/protos:frame_pair_cc_proto",
        "//mapping/protos:pose_cc_proto",
        "//mapping/registration:registration_visualization",
        "@system_libs//:eigen",
        "@system_libs//:geographiclib",
        "@system_libs//:glog",
        "@system_libs//:opencv",
        "@system_libs//:pcl",
    ],
)

cc_library(
    name = "pose_graph_optimizer",
    srcs = ["pose_graph_optimizer.cc"],
    hdrs = ["pose_graph_optimizer.h"],
    deps = [
        "//mapping/protos:frame_cc_proto",
        "//mapping/protos:pose_cc_proto",
        "@local_libs//:gtsam",
        "@system_libs//:eigen",
        "@system_libs//:glog",
    ],
)
