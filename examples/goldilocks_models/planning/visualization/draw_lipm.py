import subprocess
import time
import os
from pathlib import Path
from datetime import datetime
import sys

import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
import matplotlib.tri as mtri

if __name__ == "__main__":
  # parameters
  solve_idx = 0
  draw_lipm_mpc = True  # independent MPC or cascaded MPC

  # visualization
  dt_wait = 0.05
  x_lb = -0.5
  x_ub = 10
  y_lb = -5
  y_ub = 5

  # input arguments
  idx_list = [str(solve_idx)] if solve_idx >= 0 else ["debug"]
  if len(sys.argv) == 2:
    solve_idx = int(sys.argv[1])
    idx_list = [str(solve_idx)] if solve_idx >= 0 else ["debug"]
  if len(sys.argv) == 3:
    solve_idx_start = int(sys.argv[1])
    solve_idx_end = int(sys.argv[2])
    idx_list = list(range(solve_idx_start, solve_idx_end + 1))
    idx_list = [str(ele) for ele in idx_list]
  print("idx_list = ")
  print(idx_list)

  if (len(idx_list) == 1) & (idx_list[0] == "-1"):
    idx_list = ["debug"]

  # file name
  pref = "global_preprocess_" if draw_lipm_mpc else "global_"

  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/controller/osc_rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  model_dir = parsed_yaml_file.get('dir_model')
  data_dir = parsed_yaml_file.get('dir_data')

  for j in range(len(idx_list)):
    idx = idx_list[j]
    print("drawing " + idx)

    # Read LIPM state and input (foot position)
    global_x_lipm = np.loadtxt(data_dir + "%s_%sx_lipm.csv" % (idx, pref),
      delimiter=',')
    global_u_lipm = np.loadtxt(data_dir + "%s_%su_lipm.csv" % (idx, pref),
      delimiter=',')

    #
    line_data = np.zeros((2, global_x_lipm.shape[1] + global_u_lipm.shape[1]))
    line_data[:, 0] = global_x_lipm[0:2, 0]
    for i in range(global_x_lipm.shape[1] - 1):
      line_data[:, 2 * i + 1] = global_u_lipm[:, i]
      line_data[:, 2 * i + 2] = global_x_lipm[0:2, i + 1]

    figname = "LIPM position and feet position at touchdown"
    plt.figure(figname, figsize=(6.4, 4.8))
    plt.plot(global_x_lipm[0, :], global_x_lipm[1, :], 'ko', markersize=12,
      label='body')
    plt.plot(global_u_lipm[0, :], global_u_lipm[1, :], 'bo', label='foot')
    plt.plot(line_data[0, :], line_data[1, :], 'k-')
    for i in range(global_x_lipm.shape[1]):
      plt.arrow(global_x_lipm[0, i], global_x_lipm[1, i],
        0.1 * global_x_lipm[2, i], 0.1 * global_x_lipm[3, i],
        head_width=0.03, width=0.02, label='velocity')

    plt.legend()
    plt.ylabel('y (m)')
    plt.xlabel('x (m)')
    plt.axes().set_aspect('equal', 'datalim')
    plt.axes().axes.set_xlim([x_lb, x_ub])
    plt.axes().axes.set_ylim([y_lb, y_ub])

    if j == len(idx_list) - 1:
      print("global_x_lipm = ")
      print(global_x_lipm)
      print("global_u_lipm = ")
      print(global_u_lipm)
      plt.show()
    else:
      plt.draw()
      plt.pause(dt_wait)
      plt.clf()
