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
        sha256 = "4bfc1d5d346eb4678ef97e35a19282c2a956487be05b20dfebdd49e37a90ec50"
    elif os_result.ubuntu_release == "16.04":
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-16.04.zip"]
        sha256 = "68aa30355d6c094f817ace4c46c786b5c83765a4fddac164d5228994915e10ed"
        prefix = "signal-scope-16.04"
    elif os_result.ubuntu_release == "18.04":
        urls = ["https://www.seas.upenn.edu/~posa/files/signal-scope/signal-scope-18.04.zip"]
        sha256 = "b9dd567bde8f4a1043ae9f13194b7a2cf9c8c716f8e7373f11a98a770c404d54"
        prefix = "signal-scope-18.04"
    else:
        fail("Operating system is NOT supported", attr = os_result)

    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(urls, "", sha256=sha256, stripPrefix=prefix)

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
