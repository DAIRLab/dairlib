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
        commit = "master",
        sha256 = "d357ac363a74598c60b2fb05b0222fcc9c874b5f34ff27f83f441d3e8a16a81f",  # noqa
        build_file = "//tools/workspace/acd2d:package.BUILD.bazel",
        mirrors = mirrors,
    )
