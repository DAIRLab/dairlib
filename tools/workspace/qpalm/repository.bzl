# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def qpalm_repository(
        version = "1.2.5",
        sha256 = "2324da99a5aa0661a9567ea38e155d8515e807ebc3ea796d209aa6937c436914",
        platform = "Linux"):
    """
    Download release archive from GitHub.

    Args:
        version: Version of the library to download.
        sha256: SHA-256 checksum of the downloaded archive.
    """
    http_archive(
        name = "qpalm",
        urls = [
            "https://github.com/kul-optec/QPALM/releases/download/{}/QPALM-{}-{}.tar.gz".format(version, version, platform),
        ],
        sha256 = sha256,
        strip_prefix = "QPALM-{}-{}".format(version, platform),
        build_file = "//tools/workspace/qpalm:package.BUILD.bazel",
    )
