load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    commit = "c246859157feed71279fcfc232b4b874391f6494"
    sha256 = "e0b9b8203feff96b0ba6b3b9106138590c849289b0ab1dd520573c3f7ca5cbbc"
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
