load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    commit = "8d25c7e4a627631caab0abe9391dfa2fc7d9bc75"
    sha256 = "2387332ba8b4c2f525c6c529217f228d3ff1b2e15fe083c8474c673871e80370"
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
