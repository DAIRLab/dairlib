# -*- python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:mirrors.bzl",
    DRAKE_MIRRORS = "DEFAULT_MIRRORS",
)

# For externals beyond what Drake offers, we don't want to use Drake's mirrors.
DEFAULT_MIRRORS = dict([
    (
        key,
        [x for x in patterns if x.find("/drake-mirror/") == -1],
    )
    for key, patterns in DRAKE_MIRRORS.items()
])
