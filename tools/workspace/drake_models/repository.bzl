load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    commit = "f1be4aae99d702170ea5ddbaa7286baabbde25fe"
    sha256 = "ac9b60b79be73fd454f203ccd64d67fcbab165e28a1a2e41d153d9f4a97696df"
    http_archive(
        name = "my_drake_models",
        build_file = ":package.BUILD.bazel",
        urls = ["https://github.com/xuanhien070594/models/archive/{}.tar.gz".format(commit)],
        sha256 = sha256,  # noqa
        strip_prefix = "models-{}".format(commit),
    )

drake_models_extension = module_extension(
    implementation = _drake_models_extension_impl,
)
