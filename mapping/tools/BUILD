package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_binary")


cc_binary(
    name = "pointcloud_stitch_cli",
    srcs= [
      "pointcloud_stitch_cli.cc",
    ],
    deps = [
      "//mapping/common:file",
      "//mapping/common:pcl_types",
      "//mapping/common:pose3d",
      "//mapping/protos:frame_cc_proto",
      "@system_libs//:gflags",
      "@system_libs//:glog",
      "@system_libs//:pcl",
    ],
)
