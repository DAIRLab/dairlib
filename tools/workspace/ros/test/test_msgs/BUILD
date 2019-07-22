load('//tools/workspace/ros/bazel:message_generation.bzl', 'generate_messages')

# Implicitly creates a msgs_py target in the current scope
generate_messages(
    srcs=glob(['msg/*.msg']),
    ros_package_name='test_msgs',
)

py_binary(
    name='main',
    srcs=['scripts/main.py'],
    deps=[
        ':msgs_py',
    ],
    tags=["ros"],
)
