# -*- python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:execute.bzl",
    "execute_and_return",
)

def _os_impl(repo_ctx):
    # Find the Ubuntu version number.

    repo_ctx.file("echo.sh", """
#!/bin/bash
echo $OSTYPE
                  """)
    result = execute_and_return(repo_ctx, ["bash", "echo.sh"])
    if result.error:
        fail(result.error)
    os_type = result.stdout.strip()

    repo_ctx.file("BUILD.bazel", "")

    constants = """
OSTYPE = "{os_type}"
    """.format(
        os_type = os_type,
    )
    repo_ctx.file("os.bzl", constants)

os_repository = repository_rule(
    implementation = _os_impl,
)