package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "matching_pair_searcher",
    srcs = ["matching_pair_searcher.cc"],
    hdrs = ["matching_pair_searcher.h"],
    deps = [
        "//mapping/protos:frame_cc_proto",
        "@system_libs//:glog",
        "@system_libs//:eigen",
        "@system_libs//:geographiclib",
    ],
)
