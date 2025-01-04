# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def rules_foreign_cc_repository(
        version = "0.13.0",
        sha256 = "8e5605dc2d16a4229cb8fbe398514b10528553ed4f5f7737b663fdd92f48e1c2"):
    """
    Download repository from GitHub as a ZIP archive, decompress it, and make
    its targets available for binding.

    Args:
        version: version of the library to get.
        sha256: SHA-256 checksum of the downloaded archive.
    """
    http_archive(
        name = "rules_foreign_cc",
        urls = [
            "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/{}.tar.gz".format(version),
        ],
        sha256 = sha256,
        strip_prefix = "rules_foreign_cc-{}".format(version),
    )
