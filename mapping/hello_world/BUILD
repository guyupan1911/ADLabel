load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "hello_world",
    srcs = ["hello_world.cc"],
    deps = [
        "@system_libs//:eigen",
        "@system_libs//:glog",
        "@system_libs//:opencv",
        "@system_libs//:pcl",
    ],
)

cc_binary(
    name = "test_proto",
    srcs = ["test_proto.cc"],
    deps = [
        "//mapping/protos:hello_world_cc_proto",
        "@system_libs//:glog",
    ],
)

cc_binary(
    name = "test_gtsam",
    srcs = ["test_gtsam.cc"],
    deps = [
        "@local_libs//:gtsam",
        "@system_libs//:glog",
    ],
)