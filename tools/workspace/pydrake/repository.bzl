load("@drake//tools/workspace:pypi_wheel.bzl", "setup_pypi_wheel")

def _impl(repository_ctx):
    mirrors = {
        "pypi_wheel": [
            "https://files.pythonhosted.org/packages/{blake2_256_01}/{blake2_256_23}/{blake2_256_4p}/{package}-{version}-{tag}.whl",  # noqa
        ],
    }

    result = setup_pypi_wheel(
        repository_ctx,
        package = "drake",
        version = "0.38.0",
        pypi_tag = "cp38-cp38-manylinux_2_27_x86_64",
        blake2_256 = "206586a5675ddfef7773b225b120499aace536df9e66f6ee91fd10664d04b517",  # noqa
        sha256 = "5dbf066815b3108aa8b52e51c7244368555bef4d3d643e06cf6a36f40da9a169",  # noqa
        mirrors = mirrors,
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