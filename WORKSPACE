# -*- mode: python -*-
# vi: set ft=python :

workspace(name = "dairlib")

# dairlib can use either a local version of drake or a pegged revision
# If the environment variable DAIRLIB_LOCAL_DRAKE_PATH is set, it will use
# a local version, ad the specified path. Otherwise, it will get a pegged
# revision from github.
# As an example,
#  export DAIRLIB_LOCAL_DRAKE_PATH=/home/user/workspace/drake

# Choose a revision of Drake to use.
DRAKE_COMMIT = "f2cdfd2f9855c7f6cfc6ab880c8e7d09df0c0cd4"

DRAKE_CHECKSUM = "bb0cef45e69ea5462425a3125ee12894384893b1da7d8510277616add054d207"
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Load an environment variable.
load("//:environ.bzl", "environ_repository")

environ_repository(
    name = "environ",
    vars = ["DAIRLIB_LOCAL_DRAKE_PATH"],
)

load("@environ//:environ.bzl", "DAIRLIB_LOCAL_DRAKE_PATH")

# The WORKSPACE file does not permit `if` statements, so we handle the local
# option by toying with the repository names.  The selected repository is named
# "@drake", the other is named "@drake_ignored".
(_http_drake_repo_name, _local_drake_repo_name) = (
    "drake_ignored" if DAIRLIB_LOCAL_DRAKE_PATH else "drake",
    "drake" if DAIRLIB_LOCAL_DRAKE_PATH else "drake_ignored",
)

# Maybe download Drake.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = _http_drake_repo_name,
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# Maybe use a local checkout of Drake.
print("Using DAIRLIB_LOCAL_DRAKE_PATH={}".format(DAIRLIB_LOCAL_DRAKE_PATH)) if DAIRLIB_LOCAL_DRAKE_PATH else None  # noqa

local_repository(
    name = _local_drake_repo_name,
    path = DAIRLIB_LOCAL_DRAKE_PATH,
)

# Reference external software libraries and tools per Drake's defaults.  Some
# software will come from the host system (Ubuntu or macOS); other software
# will be downloaded in source or binary form from github or other sites.
load("@drake//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@dairlib//tools/workspace/signal_scope:repository.bzl", "signal_scope_repository")

signal_scope_repository(name = "signal_scope")

# Prebuilt ROS workspace
new_local_repository(
    name = "ros",
    build_file = "tools/workspace/ros/ros.bazel",
    path = "tools/workspace/ros/bundle_ws/install",
)

# Other catkin packages from source
# TODO: generate this automatically from rosinstall_generator

http_archive(
    name = "genmsg_repo",
    build_file = "@//tools/workspace/ros/bazel:genmsg.BUILD",
    sha256 = "d7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2",
    strip_prefix = "genmsg-0.5.8",
    urls = ["https://github.com/ros/genmsg/archive/0.5.8.tar.gz"],
)

http_archive(
    name = "genpy_repo",
    build_file = "@//tools/workspace/ros/bazel:genpy.BUILD",
    sha256 = "35e5cd2032f52a1f77190df5c31c02134dc460bfeda3f28b5a860a95309342b9",
    strip_prefix = "genpy-0.6.5",
    urls = ["https://github.com/ros/genpy/archive/0.6.5.tar.gz"],
)

# dairlib can use either a local version of invariant-ekf or a pegged revision
# If the environment variable DAIRLIB_LOCAL_INEKF_PATH is set, it will use
# a local version, ad the specified path. Otherwise, it will get a pegged
# revision from github.
# As an example,
#  export DAIRLIB_LOCAL_INEKF_PATH=/home/user/workspace/invariant-ekf

# Choose a revision of InEKF to use.
INEKF_COMMIT = "7fde9f84dbe536ba9439a3b8c319efb51ff760dd"

INEKF_CHECKSUM = "f87e3262b0c9c9237881fcd539acd1c60000f97dfdfa47b0ae53cb7a0f3256e4"

# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# INEKF_CHECKSUM = "0" * 64

# Load an environment variable.
environ_repository(
    name = "environ_inekf",
    vars = ["DAIRLIB_LOCAL_INEKF_PATH"],
)

load("@environ_inekf//:environ.bzl", "DAIRLIB_LOCAL_INEKF_PATH")

# The WORKSPACE file does not permit `if` statements, so we handle the local
# option by toying with the repository names.  The selected repository is named
# "@inekf", the other is named "@inekf_ignored".
(_http_inekf_repo_name, _local_inekf_repo_name) = (
    "inekf_ignored" if DAIRLIB_LOCAL_INEKF_PATH else "inekf",
    "inekf" if DAIRLIB_LOCAL_INEKF_PATH else "inekf_ignored",
)

# Maybe download InEKF.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = _http_inekf_repo_name,
    sha256 = INEKF_CHECKSUM,
    strip_prefix = "invariant-ekf-{}".format(INEKF_COMMIT),
    urls = [x.format(INEKF_COMMIT) for x in [
        "https://github.com/DAIRLab/invariant-ekf/archive/{}.tar.gz",
    ]],
)

# Maybe use a local checkout of InEKF.
print("Using DAIRLIB_LOCAL_INEKF_PATH={}".format(DAIRLIB_LOCAL_INEKF_PATH)) if DAIRLIB_LOCAL_INEKF_PATH else None  # noqa

local_repository(
    name = _local_inekf_repo_name,
    path = DAIRLIB_LOCAL_INEKF_PATH,
)
