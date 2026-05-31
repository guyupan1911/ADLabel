# BUILD file for system libraries installed via apt in the Docker image.
# All paths are relative to /usr (the new_local_repository root).
#
# Ubuntu 22.04 apt versions:
#   OpenCV 4.5.x  (libopencv-dev)
#   PCL 1.12.x    (libpcl-dev)
#   Eigen 3.4.x   (libeigen3-dev)

load("@rules_cc//cc:defs.bzl", "cc_library")

# ---------------------------------------------------------------------------
# gflags
# ---------------------------------------------------------------------------
cc_library(
    name = "gflags",
    hdrs = glob(["include/gflags/**/*.h"]),
    includes = ["include"],
    linkopts = ["-lgflags"],
    visibility = ["//visibility:public"],
)

# ---------------------------------------------------------------------------
# glog (depends on gflags)
# ---------------------------------------------------------------------------
cc_library(
    name = "glog",
    hdrs = glob(["include/glog/**/*.h"]),
    includes = ["include"],
    deps = [":gflags"],
    linkopts = ["-lglog"],
    visibility = ["//visibility:public"],
)

# ---------------------------------------------------------------------------
# Eigen (header-only)
# ---------------------------------------------------------------------------
cc_library(
    name = "eigen",
    hdrs = glob(["include/eigen3/**"]),
    includes = ["include/eigen3"],
    visibility = ["//visibility:public"],
)

# ---------------------------------------------------------------------------
# OpenCV 4.5
# Headers live under /usr/include/opencv4/
# Shared libs are in /usr/lib/x86_64-linux-gnu/ — let the linker find them.
# Add or remove modules below to match what your targets actually use.
# ---------------------------------------------------------------------------
cc_library(
    name = "opencv",
    hdrs = glob([
        "include/opencv4/**/*.h",
        "include/opencv4/**/*.hpp",
    ]),
    includes = ["include/opencv4"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
        "-lopencv_imgcodecs",
        "-lopencv_calib3d",
        "-lopencv_features2d",
        "-lopencv_flann",
        "-lopencv_video",
        "-lopencv_videoio",
    ],
    visibility = ["//visibility:public"],
)

# ---------------------------------------------------------------------------
# PCL 1.12
# Headers live under /usr/include/pcl-1.12/
# PCL depends on Eigen, Boost, and flann — all present in the Docker image.
# ---------------------------------------------------------------------------
cc_library(
    name = "pcl",
    hdrs = glob([
        "include/pcl-1.12/**/*.h",
        "include/pcl-1.12/**/*.hpp",
    ]),
    includes = ["include/pcl-1.12"],
    deps = [":eigen"],
    linkopts = [
        "-lpcl_common",
        "-lpcl_io",
        "-lpcl_filters",
        "-lpcl_segmentation",
        "-lpcl_registration",
        "-lpcl_kdtree",
        "-lpcl_octree",
        "-lpcl_search",
        "-lpcl_features",
        "-lboost_system",
        "-lflann_cpp",
    ],
    visibility = ["//visibility:public"],
)
