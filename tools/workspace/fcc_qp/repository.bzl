load("@drake//tools/workspace:github.bzl", "github_archive")

def fcc_qp_repository(
        name,
        mirrors = {"github": [
            "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",  # noqa
            "https://github.com/{repository}/archive/{commit_sha}.tar.gz",
        ]}):
    github_archive(
        name = name,
        repository = "Brian-Acosta/fcc_qp",
        upgrade_advice = """
            Update commit to upgrade fcc_qp
        """,
        commit = "1ce613cf6da378d640a9947087d1ed5152b45ca3",
        sha256 = "61086ce78ee9c9f2a0212f0c0754ade8a454592ae4aafa398ed95607555b48ec",  # noqa
        build_file = "//tools/workspace/fcc_qp:package.BUILD.bazel",
        mirrors = mirrors,
    )