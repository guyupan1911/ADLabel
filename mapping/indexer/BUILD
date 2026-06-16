package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "plus_indexer",
    srcs = ["plus_indexer.py"],
    deps = [
        "//mapping/protos:camera_calibration_py_proto",
        "//mapping/protos:frame_py_proto",
    ],
)
