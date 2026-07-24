package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "small_gicp_adapter",
    srcs = ["small_gicp_adapter.cc"],
    hdrs = [
        "small_gicp_adapter.h",
        "small_gicp_types.h",
    ],
    deps = [
        "//mapping/common:pcl_types",
        "//third_party/small_gicp:small_gicp",
        "@system_libs//:eigen",
    ],
)

cc_library(
    name = "registration_visualization",
    srcs = ["registration_visualization.cc"],
    hdrs = ["registration_visualization.h"],
    deps = [
        ":small_gicp_adapter",
        "//mapping/common:pcl_types",
        "@system_libs//:eigen",
        "@system_libs//:opencv",
        "@system_libs//:pcl",
    ],
)
