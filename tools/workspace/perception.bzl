# -*- python -*-
# vi: set ft=python :

# Reference external software libraries and tools needed for Cassie's perception
# stack. Some software will come from the host system (Ubuntu or macOS); other
# software will be downloaded in source or binary form from github or other
# sites.

load("//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/opencv:repository.bzl", "opencv_repository")
load("//tools/workspace/cgal:repository.bzl", "cgal_repository")
load("//tools/workspace/realsense2:repository.bzl", "realsense2_repository")
load("//tools/workspace/realsense2_pkgconfig:repository.bzl", "realsense2_pkgconfig_repository")
load("//tools/workspace/acd2d:repository.bzl", "acd2d_repository")
load("@os_type//:os.bzl", "OSTYPE")

def add_perception_repositories(excludes = [], mirrors = DEFAULT_MIRRORS):
    os = "macos" if "darwin" in OSTYPE else "linux"

    print("Detected {} as OS for adding perception deps.".format(os))

    if "acd2d" not in excludes:
        acd2d_repository(name = "acd2d")
    if "opencv" not in excludes:
        opencv_repository(name = "opencv")
    if "cgal" not in excludes:
        cgal_repository(name = "cgal")
    if "realsense2" not in excludes and "macos" not in os:
        realsense2_repository(name = "realsense2")
    if "realsense2" not in excludes and "macos" in os:
        realsense2_pkgconfig_repository(name = "realsense2")
