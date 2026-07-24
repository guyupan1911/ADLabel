load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "small_gicp_utils_cli",
    srcs = ["small_gicp_utils_cli.cc"],
    deps = [
        "//mapping/common:scoped_timer",
        "//mapping/registration:small_gicp_adapter",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)

cc_binary(
    name = "small_gicp_registration_cli",
    srcs = ["small_gicp_registration_cli.cc"],
    deps = [
        "//mapping/common:scoped_timer",
        "//mapping/registration:registration_visualization",
        "//mapping/registration:small_gicp_adapter",
        "@system_libs//:eigen",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)
