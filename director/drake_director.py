"""
Lightweight wrapper around Drake's wrapped version of Director.
"""

import os
import sys

RUNFILES_DIR = os.environ.get("DRAKE_BAZEL_RUNFILES")


def main():
    assert RUNFILES_DIR, (
        "This must be called by a script generated using the "
        "`drake_runfiles_binary` macro.")
    drake_visualizer_real = os.path.join(
        RUNFILES_DIR, "external/drake/tools/drake_visualizer_py")
    assert os.path.isfile(drake_visualizer_real), drake_visualizer_real
    argv = [drake_visualizer_real] + sys.argv[1:]
    os.execv(argv[0], argv)


assert __name__ == "__main__"
main()
