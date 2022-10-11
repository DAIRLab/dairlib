# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def proxsuite_repository(
        name,
        mirrors = None):
    github_archive(
        name = "proxsuite",
        repository = "Simple-Robotics/proxsuite",
        commit = "v0.2.0",
        sha256 = "0" * 64,  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )