package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "road_layering",
    hdrs = [
        "road_layering.h",
        "trip_segment.h"
    ],
    srcs = [
        "road_layering.cc"
    ],
    deps = [
        "//mapping/common:pose3d",
        "//mapping/mapping_utils:geographic_transform",
        "//mapping/protos:frame_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:glog",
    ],
)
