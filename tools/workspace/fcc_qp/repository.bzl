load("@drake//tools/workspace:github.bzl", "github_archive")

def fcc_qp_repository(
        name,
        mirrors = {"github": [
            "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",  # noqa
            "https://github.com/{repository}/archive/{commit}.tar.gz",
        ]}):
    github_archive(
        name = name,
        repository = "Brian-Acosta/fcc_qp",
        upgrade_advice = """
            Update commit to upgrade fcc_qp
        """,
        commit = "main",
        sha256 = "36b6ac15f63331d37da4ca756436c7d635f815c2f338082f50715ed50618f417",  # noqa
        build_file = "//tools/workspace/fcc_qp:package.BUILD.bazel",
        mirrors = mirrors,
    )