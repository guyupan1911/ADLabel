package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
  name = "lidar_topdown",
  hdrs = [
    "grid_frame.h",
    "lidar_lossless_map_node.h",
    "lossless_map_cell.h",
    "lossless_map_matrix.h",
  ],
  srcs = [
    "lidar_lossless_map_node.cc",
  ],
  deps = [
    "@system_libs//:eigen",
    "@system_libs//:opencv",
  ],
)
