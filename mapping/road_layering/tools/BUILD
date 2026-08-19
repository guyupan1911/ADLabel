package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_binary")



cc_binary(
    name = "road_layering_cli",
    srcs = ["road_layering_cli.cc"],
    deps = [
        "//mapping/common:file",
        "//mapping/common:local_data_reader",
        "//mapping/common:pcl_types",
        "//mapping/mapping_utils:stitch_utils",
        "//mapping/protos:frame_cc_proto",
        "//mapping/road_layering:road_layering",
        "@system_libs//:eigen",
        "@system_libs//:gflags",
        "@system_libs//:glog",
        "@system_libs//:pcl",
    ],
)
