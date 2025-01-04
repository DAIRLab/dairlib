# -*- python -*-
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def pinocchio_pkgconfig_repository(
        name,
        modname = "pinocchio",
        pkg_config_paths = [
            "/opt/homebrew/lib/pkgconfig",
        ],
        **kwargs):
    pkg_config_repository(
        name = name,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
