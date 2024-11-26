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
        sha256 = "fab02f537838adcc72a17970937867e923d830abc8ec77fbdd681992c5c309d8",  # noqa
        build_file = "//tools/workspace/fcc_qp:package.BUILD.bazel",
        mirrors = mirrors,
    )
