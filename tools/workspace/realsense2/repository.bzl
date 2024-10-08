# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:deb.bzl",
    "new_deb_archive",
)

# Use librealsense1 debs provided by
# https://software.intel.com/sites/products/realsense/intro/getting_started.html
#
# To update the URLs, add the apt repository as noted in the instructions then
#   apt -y install --print-uris librealsense2-dev

# The full list of debs:
# "librealsense2-dkms_1.2.0-0ubuntu5_all.deb",
# "librealsense2-udev-rules_2.10.3-0~realsense0.66_amd64.deb",
# "librealsense2_2.10.3-0~realsense0.66_amd64.deb",
# "librealsense2-utils_2.10.3-0~realsense0.66_amd64.deb",
# "librealsense2-dev_2.10.3-0~realsense0.66_amd64.deb",
# "librealsense2-dbg_2.10.3-0~realsense0.66_amd64.deb",

def realsense2_repository(
        name,
        # TODO(jeremy.nimmer@tri.global) Ideally we'd mate this mirrors list
        # with the mirrors.bzl design, once we are using that.
        realsense_mirrors = [
            "https://librealsense.intel.com/Debian/apt-repo/pool/jammy/main",  # noqa
        ],
        filenames = [
            "librealsense2_2.54.2-0~realsense.10773_amd64.deb",
            "librealsense2-dev_2.54.2-0~realsense.10773_amd64.deb",
            "librealsense2-udev-rules_2.54.2-0~realsense.10773_amd64.deb",
        ],
        sha256s = [
            "7f4289ee0be1cc9476b6def5478c50bcf5e9c928eb700e80372f26a0c90c180a",
            "56368d0986cba3e7b077dfcc187398c0c93df257a455235fb82665041cb0baaf",
            "17e372051d5cecbc10b39c137be82117892c6d79b4fb3907b0c844105bcb6abe",
        ],
        build_file = "//tools/workspace/realsense2:package.BUILD.bazel",
        **kwargs):
    new_deb_archive(
        name = name,
        mirrors = realsense_mirrors,
        filenames = filenames,
        sha256s = sha256s,
        build_file = build_file,
    )
