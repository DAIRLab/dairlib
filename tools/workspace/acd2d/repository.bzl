load("@drake//tools/workspace:github.bzl", "github_archive")

def acd2d_repository(
        name,
        mirrors = {"github": [
            "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",  # noqa
            "https://github.com/{repository}/archive/{commit_sha}.tar.gz",
        ]}):
    github_archive(
        name = name,
        repository = "DAIRLab/acd2d",
        upgrade_advice = """
            Update checksum to update to latest version
        """,
        commit = "2eb86381ad560b1d02142db943630dc196720a46",
        sha256 = "a9610404e1b6c375fc98ee838c654aa1d94511a4765b5104ab7f8164872eaecf",  # noqa
        build_file = "//tools/workspace/acd2d:package.BUILD.bazel",
        mirrors = mirrors,
    )
