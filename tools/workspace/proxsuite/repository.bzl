# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def proxsuite_repository(
        name,
        mirrors = None):
    github_archive(
        name = "proxsuite",
        repository = "Simple-Robotics/proxsuite",
        commit = "v0.2.0",
        sha256 = "dc4e9abd3e45dfdcd23b1546d46426dcfad6566edb718d3ca1db35d49b2a1eec",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )