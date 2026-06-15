package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "trajectory_io",
    srcs = [
        "trajectory_io.cc",
    ],
    hdrs = [
        "trajectory_io.h",
    ],
    deps = [
        "@local_libs//:gtsam",
    ],
)

cc_binary(
    name = "odometry_example",
    srcs = [
        "odometry_example.cc",
    ],
    deps = [
        ":trajectory_io",
        "@system_libs//:glog",
        "@local_libs//:gtsam",
    ],
)

cc_binary(
    name = "imu_gps_kitti_example",
    srcs = [
        "imu_gps_kitti_example.cc",
    ],
    deps = [
        "@system_libs//:glog",
        "@local_libs//:gtsam",
    ],
)