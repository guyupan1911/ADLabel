load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_manager",
    srcs = ["map_manager.cc"],
    hdrs = ["map_manager.h"],
    deps = [
        "//mapping/common:pcl_types",
        "//mapping/registration:small_gicp_adapter",
        "@system_libs//:eigen",
    ],
)
