load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _drake_models_extension_impl(module_ctx):
    http_archive(
        name = "my_drake_models",
        build_file = ":package.BUILD.bazel",
        urls = ["https://github.com/xuanhien070594/models/archive/297a2ee67900e278077c467a9ed60bc7bea664cd.tar.gz"],
        sha256 = "c754210bf0d63238049c97260af0f4f0c23cb3b96755df98f7d5474f55416fc1",  # noqa
        strip_prefix = "models-297a2ee67900e278077c467a9ed60bc7bea664cd",
    )

drake_models_extension = module_extension(
    implementation = _drake_models_extension_impl,
)
