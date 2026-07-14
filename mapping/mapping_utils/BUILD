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
  name = "simple_pose3d_interpolator",
  hdrs = [
    "simple_pose3d_interpolator.h",
  ],
  deps = [
    "//mapping/common:pose3d",
    "@system_libs//:glog",
  ],
)

cc_library(
  name = "timestamp_aligner",
  hdrs = [
    "timestamp_aligner.h",
  ],
  srcs = [
    "timestamp_aligner.cc",
  ],
  deps = [
    "//mapping/protos:frame_cc_proto",
    "@system_libs//:glog",
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
    "//mapping/mapping_utils:simple_pose3d_interpolator",
    "//mapping/protos:frame_cc_proto",
    "//mapping/protos:lidar_detection_3d_cc_proto",
    "@system_libs//:eigen",
    "@system_libs//:glog",
    "@system_libs//:pcl",
  ],
)
