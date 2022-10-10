load("@drake//tools/workspace:github.bzl", "github_archive")

def proxsuite_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osqp/osqp",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.1.2",
        sha256 = "0" * 64,  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
