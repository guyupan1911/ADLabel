package(default_visibility = ["//visibility:public"])

load("@rules_cc//cc:defs.bzl", "cc_library")

# Header-only small_gicp core. Consumers must compile as C++17.
# TBB is declared because the vendored core includes optional *_tbb.hpp
# backends; OpenMP users must additionally compile their target with
# -fopenmp when selecting an *_omp.hpp backend.
cc_library(
    name = "small_gicp",
    hdrs = glob(["include/small_gicp/**/*.hpp"]),
    includes = ["include"],
    deps = [
        "@system_libs//:eigen",
        "@system_libs//:tbb",
    ],
)
