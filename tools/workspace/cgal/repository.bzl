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
        sha256 = "46f3054a8afd1cf49969f4fd7324ba2c8f552b1aef6ecea72e862cbeb3b4a8eb",  # noqa
        build_file = "//tools/workspace/cgal:package.BUILD.bazel",
        mirrors = mirrors,
    )
