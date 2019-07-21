# -*- python -*-

package(default_visibility = ["//visibility:public"])

# Prebuilt C++ and Python ROS libraries

cc_library(
    name='ros',
    srcs=glob(['lib/*.so']),
    hdrs=glob(['include/**/*.h', 'include/**/*.hpp']),
    strip_include_prefix='include'
)

py_library(
    name='ros_python',
    srcs=glob(['lib/python2.7/dist-packages/**/*.py']),
    imports=['lib/python2.7/dist-packages/']
)
