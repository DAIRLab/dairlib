load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    commit = "16514419a2c25636e163f15e51fbbee6a155d572"
    sha256 = "5888782ee3d9081c987604e02e235e60d8ccbe574c5c7761852cebe9e688f7b9"
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
