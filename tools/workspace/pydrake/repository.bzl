load("@drake//tools/workspace:pypi_wheel.bzl", "setup_pypi_wheel")

def _impl(repository_ctx):
    mirrors = {
        "pypi_wheel": [
            "https://www.seas.upenn.edu/~posa/files/{package}-{version}-{tag}.whl",  # noqa
        ],
    }

    result = setup_pypi_wheel(
        repository_ctx,
        package = "drake",
        version = "0.0.2022.2.2",
        pypi_tag = "cp38-cp38-manylinux_2_27_x86_64",
        blake2_256 = "0",  # noqa
        sha256 = "6ffd53c9889eaa3a2ce27784ee9a21f65f849bcfab47d1958c234bd218c2d163",  # noqa
        mirrors = mirrors,
        data = "glob([\"pydrake/share/drake/**\"])",
    )
    if result.error != None:
        fail("Unable to complete setup for @{} repository: {}".format(
            # (forced line break)
            repository_ctx.name,
            result.error,
        ))

pydrake_repository = repository_rule(
    implementation = _impl,
)
