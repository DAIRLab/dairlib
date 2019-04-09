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
DRAKE_COMMIT = "a57d3198a17e12bff803b3421d0c99eb7a1a0b93"
DRAKE_CHECKSUM = "44dc2a817ea7b652e41b3d6560468699abb68967ad8fa67b88576bd23472760e"
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