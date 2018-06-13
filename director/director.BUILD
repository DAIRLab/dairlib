package(default_visibility = ["//visibility:public"])


py_library(
    name = "visualizer-bin",
    data = ["build/install/bin/drake-visualizer"],
    deps = ["@drake//lcmtypes:lcmtypes_py",
            "@lcm//:lcm-python"]
)

py_library(
    name = "signal-scope-bin",
    data = ["build/install/bin/signal-scope"],
    deps = ["@drake//lcmtypes:lcmtypes_py",
            "@lcm//:lcm-python"]
)

