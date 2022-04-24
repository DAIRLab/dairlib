# I created a file `~/bazel_is_building` to make sure that we don't don't have
# multiple nodes call bazel build at the same time (they seem to share resources
# and would fight against each other)
# However, if there is any error happen (or the job got terminated by SLURM)
# during building, the file would still be there. So I need to create this
# script to actively check the age of the file. If the file is too old, I delete
# it.

import os, time

def file_age(filepath):
  return time.time() - os.path.getmtime(filepath)

file_path = "/home/yminchen/bazel_is_building"  # for some reason, we need full path

while True:
  if os.path.exists(file_path):
    seconds = file_age(file_path)
    if seconds / 3600 > 2:
      print("file is too old, deleting")
      os.remove(file_path)

  time.sleep(60)

