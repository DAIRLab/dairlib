package(default_visibility = ["//visibility:public"])

# Prebuilt C++ libraries

cc_library(
    name = "towr",
    srcs = [
        "libtowr.so",
        "libifopt_core.so",
        "libifopt_ipopt.so",
        ],
    hdrs = glob([
        "usr/local/include/**/*.h",
        "usr/local/include/**/*.hpp"]),
)
