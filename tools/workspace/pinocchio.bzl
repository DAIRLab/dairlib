# -*- python -*-
# vi: set ft=python :

# Reference external software libraries and tools needed for Cassie's perception
# stack. Some software will come from the host system (Ubuntu or macOS); other
# software will be downloaded in source or binary form from github or other
# sites.

load("//tools/workspace/pinocchio:repository.bzl", "pinocchio_repository")
load("//tools/workspace/pinocchio_pkgconfig:repository.bzl", "pinocchio_pkgconfig_repository")
load("@os_type//:os.bzl", "OSTYPE")

def add_pinocchio_repositories(excludes = []):
    os = "macos" if "darwin" in OSTYPE else "linux"

    print("Detected {} as OS for adding pinocchio deps.".format(os))

    if "pinocchio" not in excludes:
        pinocchio_repository()
