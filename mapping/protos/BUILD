package(default_visibility = ["//visibility:public"])

load("//bazel:proto_defs.bzl", "proto_gen")

proto_gen(
    name = "hello_world",
    srcs = ["hello_world.proto"],
)

proto_gen(
    name = "pose",
    srcs = ["pose.proto"],
)

proto_gen(
    name = "camera_calibration",
    srcs = ["camera_calibration.proto"],
)

proto_gen(
    name = "frame",
    srcs = ["frame.proto"],
    deps = [
        ":pose",
        ":camera_calibration",
    ],
)

proto_gen(
    name = "lidar_detection_3d",
    srcs = ["lidar_detection_3d.proto"],
    deps = [
        ":pose",
    ],
)
