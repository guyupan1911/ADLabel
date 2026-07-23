package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "small_gicp_utils",
    srcs = ["small_gicp_utils.cc"],
    hdrs = ["small_gicp_utils.h"],
    deps = [
        "//mapping/common:pcl_types",
        "//third_party/small_gicp:small_gicp",
    ],
)
