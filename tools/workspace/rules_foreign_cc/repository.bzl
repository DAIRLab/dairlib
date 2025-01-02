# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def rules_foreign_cc_repository(
        version = "0.9.0",
        sha256 = "2a4d07cd64b0719b39a7c12218a3e507672b82a97b98c6a89d38565894cf7c51"):
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
