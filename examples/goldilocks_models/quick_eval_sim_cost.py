import sys
import subprocess
import time
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import pathlib

# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)

### argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--file_path", help="", default="", type=str)
parser.add_argument("--dir_path", help="", default="/home/yuming/Desktop/temp/1205", type=str)
parser.add_argument("--file_name", help="", default="lcmlog-2022-12-05.10", type=str)
args = parser.parse_args()

temp_log_path = args.dir_path
log_name = args.file_name
if len(args.file_path) > 0:
  temp_log_path = str(pathlib.Path(args.file_path).parent)
  log_name = args.file_path.split("/")[-1]

rom_idx = 1  # this is just a dummy variable in this script. doesn't do anything.

###
if temp_log_path[-1] != "/":
  temp_log_path = temp_log_path + "/"
print("log file path =", temp_log_path + log_name)

temp_output_dir=temp_log_path + "temp_cost_eval__can_be_deleted/" + log_name + "/"
print("temp_output_dir = ", temp_output_dir)
cmd = "rm -rf " + temp_output_dir
RunCommand(cmd, True)

cmd = "bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance %s%s ROM_WALKING true %s %d" % (temp_log_path, log_name, temp_output_dir, rom_idx)
RunCommand(cmd, True)

print("\n(t, t_rt_walking_switch): (stride_length, pelvis_height, torque_cost, cost)")
ave_cost = []
ave_torque_cost = []
stride_lengths = []
for valid_data_idx in range(1000):
  path = temp_output_dir + '%d_%d_cost_values.csv' % (rom_idx, valid_data_idx)
  if os.path.exists(path):
    cost = np.loadtxt(path, delimiter=',')[-2]  # -2 is the main cost; while -1 is the cost including regularization cost
    torque_cost = np.loadtxt(path, delimiter=',')[1]

    path2 = temp_output_dir + '%d_%d_ave_stride_length.csv' % (rom_idx, valid_data_idx)
    path3 = temp_output_dir + '%d_%d_ave_pelvis_height.csv' % (rom_idx, valid_data_idx)
    path4 = temp_output_dir + '%d_%d_start_time_rt_walking_controller_switch_time.csv' % (rom_idx, valid_data_idx)
    path5 = temp_output_dir + '%d_%d_start_time.csv' % (rom_idx, valid_data_idx)
    ave_stride_length = np.loadtxt(path2, delimiter=',').item()  # 0-dim scalar
    ave_pelvis_height = np.loadtxt(path3, delimiter=',').item()  # 0-dim scalar
    start_time_rt_walking_controller_switch_time = np.loadtxt(path4, delimiter=',').item()  # 0-dim scalar
    start_time = np.loadtxt(path5, delimiter=',').item()  # 0-dim scalar

    print("(%.3f, %.3f): (%.3f, %.3f, %.3f, %.3f)" % (start_time, start_time_rt_walking_controller_switch_time, ave_stride_length, ave_pelvis_height, torque_cost, cost))

    ave_cost.append(cost)
    ave_torque_cost.append(torque_cost)
    stride_lengths.append(ave_stride_length)

# Print average cost
if len(ave_cost) > 0:
  ave_cost = [float(i) for i in ave_cost]
  ave_torque_cost = [float(i) for i in ave_torque_cost]
  print("average cost = %.3f" % (sum(ave_cost) / len(ave_cost)))
  print("average torque cost = %.3f" % (sum(ave_torque_cost) / len(ave_torque_cost)))

# plot stride length vs cost (beware that foot spread and pelvis height can affect the cost)
if len(ave_cost) > 0:
  delta_x = 0.05  # sets minimal x range of the plot
  delta_y = 0.1  # sets minimal y range of the plot
  x_max = max(stride_lengths) + delta_x
  x_min = min(stride_lengths) - delta_x
  y_max = max((sum(ave_torque_cost) / len(ave_torque_cost)) + delta_y, max(ave_torque_cost))
  y_min = min((sum(ave_torque_cost) / len(ave_torque_cost)) - delta_y, min(ave_torque_cost))
  plt.figure("1D torque cost landscape -- " + log_name)
  plt.plot(stride_lengths, ave_torque_cost, ".")
  plt.xlim([x_min, x_max])
  plt.ylim([y_min, y_max])
  plt.xlabel("stride length (m)")
  plt.ylabel("torque cost")
  plt.show()

