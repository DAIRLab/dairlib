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
        commit = "fc853459457a7d55115816db290a042c0fe67b87",
        sha256 = "b78335d5f1a55f15061a8fac0306f66d2001b04a9df41f4ea3c2d820d0c1fffc",  # noqa
        build_file = "//tools/workspace/acd2d:package.BUILD.bazel",
        mirrors = mirrors,
    )
