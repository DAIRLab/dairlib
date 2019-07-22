# Reference:
# https://docs.bazel.build/versions/master/skylark/cookbook.html
# https://github.com/RobotLocomotion/drake/blob/eefddbee62439156b6faaf3b0cecdd0c57e704d7/tools/lcm.bzl

load('//tools/workspace/ros/bazel:path_utils.bzl', 'basename', 'dirname', 'join_paths')


def _genmsg_outs(srcs, ros_package_name, extension):
    """ Given a list of *.msg files, return the expected paths
    to the generated code with that extension. """

    (extension in ['.py', '.h']
        or fail('Unknown extension %s' % extension))

    msg_names = []
    for item in srcs:
        if not item.endswith('.msg'):
            fail('%s does not end in .msg' % item)
        item_name = basename(item)[:-len('.msg')]

        if extension == '.py':
            item_name = '_' + item_name

        msg_names.append(item_name)

    outs = [
        join_paths(ros_package_name, 'msg', msg_name + extension)
        for msg_name in msg_names
    ]

    if extension == '.py':
        outs += [
            join_paths(ros_package_name, 'msg', '__init__.py'),
            join_paths(ros_package_name, '__init__.py'),
        ]

    return outs


def _genpy_impl(ctx):
    """Implementation for the genpy rule. Shells out to the scripts
    shipped with genpy."""

    srcpath = ctx.files.srcs[0].dirname
    outpath = ctx.outputs.outs[0].dirname

    # Generate __init__.py for package
    ctx.actions.write(
        output=ctx.outputs.outs[-1],
        content='',
    )

    # Generate the actual messages
    ctx.actions.run(
        inputs=ctx.files.srcs,
        outputs=ctx.outputs.outs[:-2],
        executable=ctx.executable._gen_script,
        arguments=[
            '-o', outpath,
            '-p', ctx.attr.ros_package_name,
            # Include path for the current package
            '-I', '%s:%s' % (ctx.attr.ros_package_name, srcpath),
            # TODO: include paths of dependent packages
        ] + [
            f.path for f in ctx.files.srcs
        ],
    )

    # Generate __init__.py for msg module
    # NOTE: it looks at the .py files in its output path, so it also
    # needs to depend on the previous step.
    ctx.actions.run(
        inputs=ctx.files.srcs + ctx.outputs.outs[:-2],
        outputs=[ctx.outputs.outs[-2]],
        executable=ctx.executable._gen_script,
        arguments=[
            '--initpy',
            '-o', outpath,
            '-p', ctx.attr.ros_package_name,
        ],
    )

    return struct()


_genpy = rule(
    implementation=_genpy_impl,
    output_to_genfiles=True,
    attrs={
        'srcs': attr.label_list(allow_files=True),
        'ros_package_name': attr.string(),
        '_gen_script': attr.label(
            default=Label('@genpy_repo//:genmsg_py'),
            executable=True,
            cfg='host'),
        'outs': attr.output_list(),
    },
)


def generate_messages(srcs=None,
                      ros_package_name=None):
    """ Wraps all message generation functionality. Uses the _genpy
    and _gencpp to shell out to the code generation scripts, then wraps
    the resulting files into Python and C++ libraries.
    We use macros to hide some of the book-keeping of input & output
    files. """

    if not srcs:
        fail('srcs is required (*.msg files).')
    if not ros_package_name:
        fail('ros_package_name is required.')

    outs = _genmsg_outs(srcs, ros_package_name, '.py')

    _genpy(
        name='lkfjaklsjfklasd',
        srcs=srcs,
        ros_package_name=ros_package_name,
        outs=outs,
    )

    native.py_library(
        name='msgs_py',
        srcs=outs,
        imports=['.'],
        deps=[
            '@genpy_repo//:genpy'
        ],
    )
