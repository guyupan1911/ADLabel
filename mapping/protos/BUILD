load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

proto_library(
    name = "hello_world_proto",
    srcs = ["hello_world.proto"],
    visibility = ["//mapping:__subpackages__"],
)

cc_proto_library(
    name = "hello_world_cc_proto",
    deps = [":hello_world_proto"],
    visibility = ["//mapping:__subpackages__"],
)
