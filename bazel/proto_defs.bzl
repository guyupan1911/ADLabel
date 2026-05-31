"""Proto compilation rules for ADLabel project."""

load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@protobuf//bazel:py_proto_library.bzl", "py_proto_library")

def proto_gen(name, srcs, deps = [], visibility = ["//visibility:public"]):
    """Generate proto_library, cc_proto_library, and py_proto_library.

    Args:
        name: Base name (without suffix)
        srcs: List of .proto files
        deps: List of proto dependencies
        visibility: Visibility control

    Generates:
        {name}_proto      - proto_library
        {name}_cc_proto   - C++ bindings
        {name}_py_proto   - Python bindings
    """

    # Normalize deps to _proto suffix
    proto_deps = []
    for dep in deps:
        if dep.endswith("_proto"):
            proto_deps.append(dep)
        else:
            proto_deps.append(dep + "_proto")

    proto_library(
        name = name + "_proto",
        srcs = srcs,
        deps = proto_deps,
        visibility = visibility,
    )

    cc_proto_library(
        name = name + "_cc_proto",
        deps = [":" + name + "_proto"],
        visibility = visibility,
    )

    py_proto_library(
        name = name + "_py_proto",
        deps = [":" + name + "_proto"],
        visibility = visibility,
    )

