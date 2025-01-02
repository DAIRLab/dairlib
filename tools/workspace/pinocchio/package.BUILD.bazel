# -*- python -*-
#
# Custom build file to use the package with Bazel.

package(default_visibility = ["//visibility:public"])

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "pinocchio",
    lib_source = ":srcs",
    cache_entries = {
        "BUILD_ADVANCED_TESTING": "OFF",
        "BUILD_BENCHMARK": "OFF",
        "BUILD_DOCUMENTATION": "OFF",
        "BUILD_PYTHON_INTERFACE": "OFF",
        "BUILD_TESTING": "OFF",
        "BUILD_UTILS": "OFF",
        "BUILD_WITH_AUTODIFF_SUPPORT": "OFF",
        "BUILD_WITH_CASADI_SUPPORT": "OFF",
        "BUILD_WITH_CODEGEN_SUPPORT": "OFF",
        "BUILD_WITH_COLLISION_SUPPORT": "OFF",
        "BUILD_WITH_COMMIT_VERSION": "OFF",
        "BUILD_WITH_EXTRA_SUPPORT": "OFF",
        "BUILD_WITH_OPENMP_SUPPORT": "OFF",
        "BUILD_WITH_SDF_SUPPORT": "OFF",
        "BUILD_WITH_URDF_SUPPORT": "ON",  # this one is ON
        "CMAKE_BUILD_TYPE": "Release",
        "CMAKE_INSTALL_LIBDIR": "lib",
        "ENABLE_COVERAGE": "OFF",
        "ENABLE_TEMPLATE_INSTANTIATION": "ON",  # this one is ON
        "GENERATE_PYTHON_STUBS": "OFF",
        "INSTALL_DOCUMENTATION": "OFF",
    },
    deps = [
        "@eigen",
    ],
    out_include_dir = "include",
    out_shared_libs = [
        "libpinocchio_default.so.3.2.0",
    ],
)
