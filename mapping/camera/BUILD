package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "camera",
    hdrs = ["camera.h"],
    srcs = ["camera.cc"],
    deps = [
        "//mapping/protos:camera_calibration_cc_proto",
        "@system_libs//:eigen",
        "@system_libs//:glog",
    ],
)
