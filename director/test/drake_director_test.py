"""
Lightweight wrapper around Drake's wrapped version of Director.
"""

import os
import sys
import subprocess

print(os.getcwd())

# For the test, manually set DRAKE_BAZEL_RUNFILES
os.environ["DRAKE_BAZEL_RUNFILES"] = os.getcwd()

RUNFILES_DIR = os.environ.get("DRAKE_BAZEL_RUNFILES")


def main():
    print(os.getcwd())
    assert RUNFILES_DIR, (
        "This must be called by a script generated using the "
        "`drake_runfiles_binary` macro.")
    drake_visualizer_real = os.path.join(
        os.getcwd(), "external/drake/tools/drake_visualizer_py")
    assert os.path.isfile(drake_visualizer_real), drake_visualizer_real
    argv = [drake_visualizer_real] + sys.argv[1:]

    # Run drake visualizer for 2 seconds, to ensure it doesn't crash
    try:
      r = subprocess.check_call(argv, timeout=5)
      print(r)
    except subprocess.TimeoutExpired as e:
      print(e)


assert __name__ == "__main__"
main()
