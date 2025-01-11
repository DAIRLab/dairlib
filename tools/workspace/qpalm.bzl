# -*- python -*-
# vi: set ft=python :

# Reference external software libraries and tools needed for Cassie's perception
# stack. Some software will come from the host system (Ubuntu or macOS); other
# software will be downloaded in source or binary form from github or other
# sites.

load("//tools/workspace/qpalm:repository.bzl", "qpalm_repository")
load("@os_type//:os.bzl", "OSTYPE")

def add_qpalm():
    os = "macos" if "darwin" in OSTYPE else "linux"

    print("Detected {} as OS for adding QPALM.".format(os))

    if "macos" in os:
        qpalm_repository(platform = "Darwin")
    if "linux" in os:
        qpalm_repository(platform = "Linux")
