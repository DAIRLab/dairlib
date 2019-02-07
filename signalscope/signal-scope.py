import os
import subprocess
import sys

runfiles_dir = os.environ.get("DRAKE_BAZEL_RUNFILES")
assert runfiles_dir, (
    "This must be called by a script generated using the " +
    "`drake_runfiles_binary` macro.")

def resolve_path(relpath):
    abspath = os.path.join(runfiles_dir, relpath)
    assert os.path.exists(abspath), "Path does not exist: {}".format(abspath)
    return abspath

os.environ['LD_LIBRARY_PATH'] = resolve_path("../signal_scope/")
os.environ['DYLD_LIBRARY_PATH'] = resolve_path("../signal_scope/")

bin_path = resolve_path("../signal_scope/signal-scope")
args = [bin_path] + sys.argv[1:]
os.execv(bin_path, args)