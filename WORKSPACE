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
DRAKE_COMMIT = "v1.24.0"

DRAKE_CHECKSUM = "35874238af2c0305525a6f32c28692e3fdbed0581055b0b491669f7534cf6cae"
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
#DRAKE_CHECKSUM = "0" * 64

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
    strip_prefix = "drake-{}".format(DRAKE_COMMIT.strip("v")),
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
load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@dairlib//tools/workspace/osqp:repository.bzl", "osqp_repository")

osqp_repository(name = "osqp")

load("@dairlib//tools/workspace/fcc_qp:repository.bzl", "fcc_qp_repository")

fcc_qp_repository(name = "fcc_qp")

load("@dairlib//tools/workspace/signal_scope:repository.bzl", "signal_scope_repository")

signal_scope_repository(name = "signal_scope")

#load("@dairlib//tools/workspace/pydrake:repository.bzl", "pydrake_repository")

#pydrake_repository(name = "pydrake_pegged")

# elevation mapping dependencies
ELEVATION_MAPPING_COMMIT = "bazel"

ELEVATION_MAPPING_CHECKSUM = "fc526a61dcf19dd6b03d3d4202cbc13103f2262a739ccdf430acf87d47fa7b8c"

http_archive(
    name = "elevation_mapping",
    sha256 = ELEVATION_MAPPING_CHECKSUM,
    strip_prefix = "elevation_mapping-{}".format(ELEVATION_MAPPING_COMMIT),
    urls = [x.format(ELEVATION_MAPPING_COMMIT) for x in [
        "https://github.com/Brian-Acosta/elevation_mapping/archive/{}.tar.gz",
    ]],
)

load(
    "@elevation_mapping//tools/workspace:deps.bzl",
    "add_elevation_mapping_dependencies",
)

# we already have drake
add_elevation_mapping_dependencies(excludes = ["drake"])

# setup
load("@grid_map//tools/workspace:deps.bzl", "add_grid_map_dependencies")

add_grid_map_dependencies(excludes = ["gtest"])

load("@rules_pcl//bzl:repositories.bzl", "pcl_repositories")

# exclude pcl dependencies brought in by drake
pcl_repositories(
    excludes = [
        "gtest",
        "eigen",
        "libpng",
        "zlib",
    ],
)

load("@grid_map//tools/workspace/pcl:setup.bzl", "setup_pcl")

setup_pcl()

# Prebuilt ROS workspace
new_local_repository(
    name = "ros",
    build_file = "tools/workspace/ros/ros.bazel",
    path = "tools/workspace/ros/bundle_ws/install",
)

# Locally developed and installed ROS packages
environ_repository(
    name = "environ_local_ros",
    vars = ["LOCAL_ROS_INSTALL_PATH"],
)

load("@environ_local_ros//:environ.bzl", "LOCAL_ROS_INSTALL_PATH")

new_local_repository(
    name = "ros-local",
    build_file = "tools/workspace/ros/ros-local.bazel",
    path = LOCAL_ROS_INSTALL_PATH,
)

http_archive(
    name = "acd2d",
    build_file = "@//tools/workspace/acd2d:acd2d.bazel",
    sha256 = "d357ac363a74598c60b2fb05b0222fcc9c874b5f34ff27f83f441d3e8a16a81f",
    strip_prefix = "acd2d-master",
    urls = ["https://github.com/DAIRLab/acd2d/archive/master.tar.gz"],
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
INEKF_COMMIT = "bazel-opt"

INEKF_CHECKSUM = "297ac0d64fd2c9e7fe36d01bd4b34db0592872234438f9ef4e3221ac2f0f5e40"

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

# Maybe download InEKF
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

# buildifier is written in Go and hence needs rules_go to be built.
# See https://github.com/bazelbuild/rules_go for the up to date setup instructions.
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "d6b2513456fe2229811da7eb67a444be7785f5323c6708b38d851d2b51e54d83",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.30.0/rules_go-v0.30.0.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.30.0/rules_go-v0.30.0.zip",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

go_rules_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")

go_register_toolchains(version = "1.17.2")

http_archive(
    name = "bazel_gazelle",
    sha256 = "de69a09dc70417580aabf20a28619bb3ef60d038470c7cf8442fafcf627c21cb",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
    ],
)

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

# If you use WORKSPACE.bazel, use the following line instead of the bare gazelle_dependencies():
# gazelle_dependencies(go_repository_default_config = "@//:WORKSPACE.bazel")
gazelle_dependencies()

http_archive(
    name = "com_google_protobuf",
    sha256 = "3bd7828aa5af4b13b99c191e8b1e884ebfa9ad371b0ce264605d347f135d2568",
    strip_prefix = "protobuf-3.19.4",
    urls = [
        "https://github.com/protocolbuffers/protobuf/archive/v3.19.4.tar.gz",
    ],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "ae34c344514e08c23e90da0e2d6cb700fcd28e80c02e23e4d5715dddcb42f7b3",
    strip_prefix = "buildtools-4.2.2",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/refs/tags/4.2.2.tar.gz",
    ],
)