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
DRAKE_COMMIT = "23bf58705ba95d8dc3c1101bf9ac327a94520f19"
DRAKE_CHECKSUM = "db8b5af3bec5eb50de391f02614090b0a01472f33c859e1f2e5d8cfd99194020"
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Load an environment variable.
load("//:environ.bzl", "environ_repository")
environ_repository(name = "environ", vars = ["DAIRLIB_LOCAL_DRAKE_PATH"])
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
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
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
    name='ros',
    path='tools/workspace/ros/bundle_ws/install',
    build_file='tools/workspace/ros/ros.bazel',
)

# Other catkin packages from source
# TODO: generate this automatically from rosinstall_generator

http_archive(
    name='genmsg_repo',
    build_file='@//tools/workspace/ros/bazel:genmsg.BUILD',
    sha256='d7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2',
    urls = ['https://github.com/ros/genmsg/archive/0.5.8.tar.gz'],
    strip_prefix='genmsg-0.5.8',
)

http_archive(
    name='genpy_repo',
    build_file='@//tools/workspace/ros/bazel:genpy.BUILD',
    sha256='35e5cd2032f52a1f77190df5c31c02134dc460bfeda3f28b5a860a95309342b9',
    urls = ['https://github.com/ros/genpy/archive/0.6.5.tar.gz'],
    strip_prefix='genpy-0.6.5',
)
