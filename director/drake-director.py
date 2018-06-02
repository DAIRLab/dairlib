import os
import subprocess
import sys

os.environ['LD_LIBRARY_PATH'] = '/opt/drake/lib'

runfiles_dir = os.environ.get("DRAKE_BAZEL_RUNFILES")
assert runfiles_dir, (
    "This must be called by a script generated using the " +
    "`drake_runfiles_binary` macro.")

def resolve_path(relpath):
    abspath = os.path.join(runfiles_dir, relpath)
    assert os.path.exists(abspath), "Path does not exist: {}".format(abspath)
    return abspath

bin_path = resolve_path("external/director/build/install/bin/drake-visualizer")
args = [bin_path] + sys.argv[1:]
os.execv(bin_path, args)