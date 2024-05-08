load("@drake//tools/workspace:github.bzl", "github_archive")

def cgal_repository(
        name,
        mirrors = {"github": [
            "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",  # noqa
            "https://github.com/{repository}/archive/{commit_sha}.tar.gz",
        ]}):
    github_archive(
        name = name,
        repository = "CGAL/cgal",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "v5.6.1",
        sha256 = "0" * 64,  # noqa
        build_file = "//tools/workspace/cgal:package.BUILD.bazel",
        mirrors = mirrors,
    )
