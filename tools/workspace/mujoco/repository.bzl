"""Module extension for fetching prebuilt binary MuJoCo."""
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _mujoco_extension_impl(module_ctx):
    http_archive(
        name = "mujoco",
        build_file = "@dairlib//tools/workspace/mujoco:package.BUILD.bazel",
        sha256 = "049204172901afad251070385a6badf46d795ebe47403d093f8469557eeeab5a",
        strip_prefix = "mujoco-3.3.6",
        urls = ["https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz"],
    )

# The extension definition is now much simpler without any tag_classes
mujoco_extension = module_extension(
    implementation = _mujoco_extension_impl,
)
