load("@drake//tools/workspace:github.bzl", "github_archive")

def plane_seg_repository(
        name,
        mirrors = {"github": [
            "https://github.com/{repository}/archive/refs/tags/{tag_name}.tar.gz",  # noqa
            "https://github.com/{repository}/archive/{commit_sha}.tar.gz",
        ]}):
    github_archive(
        name = name,
        repository = "Brian-Acosta/plane_seg",
        upgrade_advice = """
            Update commit to upgrade plane_seg
        """,
        commit = "45e6d6b1a98a2cae43c2cb28e6fa310329adb2ff",
        sha256 = "fc2a8cafb92864896e16a2e88255b21059d951c6449a4d340928035b55f848ec",  # noqa
        build_file = "//tools/workspace/plane_seg:package.BUILD.bazel",
        mirrors = mirrors,
    )