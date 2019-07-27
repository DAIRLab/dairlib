_BUILD_FILE = """
py_library(
    name = 'libraries',
    srcs = glob(
        include = ['bin/**/*.py', 'site-packages/**/*.py'],
    ),
    data = glob(
        include = ['bin/**/*', 'site-packages/**/*'],
        exclude = ['**/*.py', '**/*.pyc']
    ),
    imports=['site-packages'],
    visibility = ['//visibility:public']
)
"""


def pip_package_impl(ctx):
    getpip = ctx.path(ctx.attr._getpip)
    path = ctx.path('site-packages')

    command = ['python', str(getpip)]
    command += list(ctx.attr.packages)
    command += ['--target', str(path)]
    command += ['--install-option', '--install-scripts=%s' % ctx.path('bin')]
    command += ['--no-cache-dir']

    print(command)
    result = ctx.execute(command)
    if result.return_code != 0:
      fail('Failed to execute %s.\n%s\n%s' % (
          ' '.join(command), result.stdout, result.stderr))
    ctx.file('BUILD', _BUILD_FILE, False)


pip_requirements = repository_rule(
    pip_package_impl,
    attrs={
        'packages': attr.string_list(),
        '_getpip': attr.label(
            default=Label('@pip//file:get-pip.py'),
            allow_single_file=True,
            executable=True,
            cfg='host'
        )
    }
)


def pip():
  native.http_file(
      name="pip",
      url="https://bootstrap.pypa.io/get-pip.py",
      sha256="19dae841a150c86e2a09d475b5eb0602861f2a5b7761ec268049a662dbd2bd0c"
  )