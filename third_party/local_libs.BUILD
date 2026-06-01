load("@rules_cc//cc:defs.bzl", "cc_library")

# ---------------------------------------------------------------------------
# GTSAM 4.2 (built from source, installed to /usr/local)
# Headers: /usr/local/include/gtsam/
# Libs:    /usr/local/lib/libgtsam.so
# ---------------------------------------------------------------------------
cc_library(
    name = "gtsam",
    hdrs = glob(["include/gtsam/**/*.h"]),
    includes = ["include"],
    deps = [
        "@system_libs//:eigen",
        "@system_libs//:tbb",
    ],
    linkopts = [
        "-L/usr/local/lib",
        "-lgtsam",
        "-Wl,-rpath,/usr/local/lib",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtsam_unstable",
    hdrs = glob(["include/gtsam_unstable/**/*.h"]),
    includes = ["include"],
    deps = [":gtsam"],
    linkopts = [
        "-L/usr/local/lib",
        "-lgtsam_unstable",
        "-Wl,-rpath,/usr/local/lib",
    ],
    visibility = ["//visibility:public"],
)
