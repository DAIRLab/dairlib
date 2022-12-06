import sys
import subprocess
import time
import os
import numpy as np
import argparse

# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

### argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--dir_path", help="", default="/home/yuming/Desktop/temp/1205", type=str)
parser.add_argument("--file_name", help="", default="lcmlog-2022-12-05.10", type=str)
args = parser.parse_args()

temp_log_path=args.dir_path
log_name=args.file_name

rom_idx = 1  # this is just a dummy variable in this script. doesn't do anything.

###
if temp_log_path[-1] != "/":
  temp_log_path = temp_log_path + "/"
print("log file path =", temp_log_path + log_name)

temp_output_dir=temp_log_path + "temp_cost_eval__can_be_deleted/"
cmd = "rm -rf " + temp_output_dir
RunCommand(cmd, True)

cmd = "bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance %s%s ROM_WALKING true %s %d" % (temp_log_path, log_name, temp_output_dir, rom_idx)
RunCommand(cmd, True)

print("\n(stride_length, pelvis_height, cost):")
ave_cost = []
for valid_data_idx in range(1000):
  path = temp_output_dir + '%d_%d_cost_values.csv' % (rom_idx, valid_data_idx)
  if os.path.exists(path): 
    cost = np.loadtxt(path, delimiter=',')[-2]  # -2 is the main cost; while -1 is the cost including regularization cost

    path2 = temp_output_dir + '%d_%d_ave_stride_length.csv' % (rom_idx, valid_data_idx)
    path3 = temp_output_dir + '%d_%d_ave_pelvis_height.csv' % (rom_idx, valid_data_idx)
    ave_stride_length = np.loadtxt(path2, delimiter=',').item()  # 0-dim scalar
    ave_pelvis_height = np.loadtxt(path3, delimiter=',').item()  # 0-dim scalar

    print("%.3f, %.3f, %.3f" % (ave_stride_length, ave_pelvis_height, cost))

    ave_cost.append(cost)

ave_cost = [float(i) for i in ave_cost]
print("average cost = %.3f" % (sum(ave_cost) / len(ave_cost)))

