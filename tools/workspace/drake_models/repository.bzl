load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    commit = "7247d2fe5392bc3a07a120d3c0a114e4e48bd109"
    sha256 = "c7e79662c466469fe5819a16b402d8c08da31d537a355d025bcb1e29d4da9ab5"
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
