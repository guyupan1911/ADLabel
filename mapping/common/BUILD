package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
  name = "file",
  hdrs = [
    "file.h"
  ],
  deps = [
    "@system_libs//:glog",
  ]
)

cc_library(
  name = "scoped_timer",
  hdrs = [
    "scoped_timer.h",
  ],
  srcs = [
    "scoped_timer.cc",
  ],
  deps = [
    "@system_libs//:glog",
  ],
)

cc_library(
  name = "pcl_types",
  hdrs = [
    "pcl_types.h"
  ],
  # PCL only precompiles its template algorithms (VoxelGrid, PassThrough,
  # PCLBase, ...) for the built-in point types. Using a custom point type
  # like PointXYZIRT requires PCL_NO_PRECOMPILE so the headers pull in their
  # impl/*.hpp definitions and instantiate the templates for our type.
  # `defines` propagates to every target that depends on :pcl_types.
  defines = ["PCL_NO_PRECOMPILE"],
  deps = [
    "@system_libs//:pcl",
  ]
)

cc_library(
  name = "pose3d",
  hdrs = [
    "pose3d.h",
  ],
  srcs = [
    "pose3d.cc",
  ],
  deps = [
    "//mapping/protos:pose_cc_proto",
    "@system_libs//:eigen",
  ],
)

cc_library(
  name = "local_data_reader",
  hdrs = [
    "local_data_reader.h",
  ],
  srcs = [
    "local_data_reader.cc",
  ],
  deps = [
    ":file",
    ":pcl_types",
    "@system_libs//:glog",
    "@system_libs//:opencv",
    "@system_libs//:pcl",
  ],
)
