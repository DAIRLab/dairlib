# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-mac.zip"]
        prefix = "signal-scope-mac"
        sha256 = "c34b4dd4d85f9575b51cc2e03236420d8517f5fdb232bffacc09bf35ee0bd473"
    elif os_result.ubuntu_release == "16.04":
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-16.04.zip"]
        sha256 = "d51cc26c5130cdf51ee10602d2a32032219bfdf25f96bfaa1f7dbd9d9623e1cf"
        prefix = "signal-scope-16.04"
    elif os_result.ubuntu_release == "18.04":
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-18.04.zip"]
        sha256 = "b9dd567bde8f4a1043ae9f13194b7a2cf9c8c716f8e7373f11a98a770c404d54"
        prefix = "signal-scope-18.04"
    elif os_result.ubuntu_release == "20.04":
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-20.04.zip"]
        sha256 = "6bb42fdec9767984d21cd07ab0eb13e11189c3a1b5931a5fabef4c23de247fa6"
        prefix = "signal-scope-20.04"
    else:
        fail("Operating system is NOT supported", attr = os_result)

    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(urls, "", sha256 = sha256, stripPrefix = prefix)

    file_content = """# -*- python -*-

# DO NOT EDIT: generated signal_scope_repository()

filegroup(
    name = "signal_scope",
    srcs = glob(["**"]),    
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

signal_scope_repository = repository_rule(
    implementation = _impl,
)
