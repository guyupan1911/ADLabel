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