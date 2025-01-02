# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def pinocchio_repository(
        version = "3.2.0",
        sha256 = "b6a7e6f6f6e3f175dd7010aa998a018f88d712477caa6c41f6ae038310c2fd7d"):
    """
    Download release archive from GitHub.

    Args:
        version: Version of the library to download.
        sha256: SHA-256 checksum of the downloaded archive.
    """
    http_archive(
        name = "pinocchio",
        urls = [
            "https://github.com/stack-of-tasks/pinocchio/releases/download/v{}/pinocchio-{}.tar.gz".format(version, version),
        ],
        sha256 = sha256,
        strip_prefix = "pinocchio-{}".format(version),
        build_file = Label("//tools/workspace/pinocchio:package.BUILD.bazel"),
    )
