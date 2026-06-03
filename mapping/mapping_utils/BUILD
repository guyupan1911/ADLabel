package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
  name = "frame_data",
  hdrs = [
    "frame_data.h",
  ],
  deps = [
    "//mapping/common:pcl_types",
    "@system_libs//:eigen",
  ],
)

cc_library(
  name = "stitch_utils",
  hdrs = [
    "stitch_utils.h",
  ],
  srcs = [
    "stitch_utils.cc",
  ],
  deps = [
    ":frame_data",
    "//mapping/common:local_data_reader",
    "//mapping/common:pose3d",
    "//mapping/protos:frame_cc_proto",
    "//mapping/protos:lidar_detection_3d_cc_proto",
    "@system_libs//:eigen",
    "@system_libs//:glog",
  ],
)
