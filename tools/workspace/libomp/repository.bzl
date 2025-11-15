"""Module extension for libomp on macOS."""

def _libomp_repository_impl(repository_ctx):
    # Check if we're on macOS
    if repository_ctx.os.name.startswith("mac"):
        # Use Homebrew's libomp
        result = repository_ctx.execute(["brew", "--prefix", "libomp"])
        if result.return_code == 0:
            homebrew_prefix = result.stdout.strip()
            repository_ctx.symlink(homebrew_prefix + "/include", "include")
            repository_ctx.symlink(homebrew_prefix + "/lib", "lib")
        else:
            # Fallback to default Homebrew location
            repository_ctx.symlink("/opt/homebrew/opt/libomp/include", "include")
            repository_ctx.symlink("/opt/homebrew/opt/libomp/lib", "lib")

        # Create BUILD file for macOS using template
        repository_ctx.template(
            "BUILD.bazel",
            Label("@dairlib//tools/workspace/libomp:package.BUILD.bazel"),
            substitutions = {},
            executable = False,
        )
    else:
        # On Linux, create a no-op library (OpenMP provided by compiler via -fopenmp)
        repository_ctx.file(
            "BUILD.bazel",
            content = """
package(default_visibility = ["//visibility:public"])

# No-op library on Linux - OpenMP is provided by the compiler
cc_library(
    name = "libomp",
)
""",
        )

libomp_repository = repository_rule(
    implementation = _libomp_repository_impl,
    configure = True,
    local = True,
)

def _libomp_extension_impl(module_ctx):
    # Create libomp repository (will be a no-op on Linux, real library on macOS)
    libomp_repository(name = "libomp")

libomp_extension = module_extension(
    implementation = _libomp_extension_impl,
)
