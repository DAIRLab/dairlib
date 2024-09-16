# -*- python -*-
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def realsense2_pkgconfig_repository(
        name,
        licenses = [
            "notice",  # BSD-3-Clause
            "reciprocal",  # MPL-2.0
            "unencumbered",  # Public-Domain
        ],
        modname = "realsense2",
        pkg_config_paths = [
            "/opt/homebrew/lib/pkgconfig/",
        ],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )