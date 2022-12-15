# When seeing "_tkinter.TclError: no display name and no $DISPLAY environment variable",
# uncomment the following two lines code (or just restart computer because it has something to do with ssh)
# import matplotlib
# matplotlib.use('Agg')

import sys
import subprocess
import time
import os
from pathlib import Path
from datetime import datetime
import psutil
import copy  # for deepcopying a list

import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
import matplotlib.tri as mtri
import matplotlib
from scipy.interpolate import LinearNDInterpolator
import matplotlib.patches as mpatches
import codecs
import math
from matplotlib.patches import Rectangle

if __name__ == "__main__":
  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/evaluation_scripts/visualize_rom_dynamics_function/settings_for_rom_dyn_eval.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  dir_script_backup = parsed_yaml_file.get('dir_script_backup')
  dir_model = parsed_yaml_file.get('dir_model')
  dir_data = parsed_yaml_file.get('dir_data')
  model_iter = parsed_yaml_file.get('model_iter')
  x_min = parsed_yaml_file.get('x_min')
  x_max = parsed_yaml_file.get('x_max')
  n_samples_x = parsed_yaml_file.get('n_samples_x')
  y_min = parsed_yaml_file.get('y_min')
  y_max = parsed_yaml_file.get('y_max')
  n_samples_y = parsed_yaml_file.get('n_samples_y')
  z_min = parsed_yaml_file.get('z_min')
  z_max = parsed_yaml_file.get('z_max')
  n_samples_z = parsed_yaml_file.get('n_samples_z')
  plot_xz_or_yz = parsed_yaml_file.get('plot_xz_or_yz')
  save_fig = parsed_yaml_file.get('save_fig')

  # Get model name to create subfolder in data folder
  # Extract unique_folder_name
  unique_folder_name = dir_model.split("/")[-3]
  if unique_folder_name == "robot_1" or unique_folder_name == "find_models":
      print("Warning: didn't extract unique_folder_name correctly")
  print("unique_folder_name = %s" % unique_folder_name)

  dir_plots = dir_script_backup + "plots/" + unique_folder_name + "/"
  Path(dir_plots).mkdir(parents=True, exist_ok=True)

  # Construct samples
  x_samples = np.linspace(x_min, x_max, n_samples_x)
  y_samples = np.linspace(y_min, y_max, n_samples_y)
  z_samples = np.linspace(z_min, z_max, n_samples_z)

  # 1D plot (given COM y and COM z)

  # Vector Field (x-z plane)
  if plot_xz_or_yz == 0:
    for i in range(n_samples_y):
      y_val = y_samples[i]

      x_accel = np.loadtxt("%sx_accel_%d.csv" % (dir_data, i), delimiter=',')
      z_accel = np.loadtxt("%sz_accel_%d.csv" % (dir_data, i), delimiter=',')

      plt.figure(figsize=(6.4, 4.8))
      plt.rcParams.update({'font.size': 12})
      grid_x, grid_z = np.meshgrid(x_samples,z_samples)
      plt.quiver(grid_x,grid_z,x_accel,z_accel)
      plt.xlabel('CoM x (m)')
      plt.ylabel('CoM z (m)')
      padding = 0.045
      plt.xlim([np.min(x_samples)-padding, np.max(x_samples)+padding])
      plt.title("CoM accel vector field (CoM y=%.2fm; iter%d).png" % (y_val, model_iter))
      if save_fig:
        plt.savefig("%s%d_CoM_accel_vector_field__CoM_y=%dcm__iter%dm.png" % (dir_plots, i, y_val*100, model_iter))

  # Vector Field (y-z plane)
  elif plot_xz_or_yz == 1:
    for i in range(n_samples_x):
      x_val = x_samples[i]

      y_accel = np.loadtxt("%sy_accel_%d.csv" % (dir_data, i), delimiter=',')
      z_accel = np.loadtxt("%sz_accel_%d.csv" % (dir_data, i), delimiter=',')

      plt.figure(figsize=(6.4, 4.8))
      plt.rcParams.update({'font.size': 12})
      grid_y, grid_z = np.meshgrid(y_samples,z_samples)
      plt.quiver(grid_y,grid_z,y_accel,z_accel)
      plt.xlabel('CoM y (m)')
      plt.ylabel('CoM z (m)')
      padding = 0.045
      plt.xlim([np.min(y_samples)-padding, np.max(y_samples)+padding])
      plt.title("CoM accel vector field (CoM x=%.2fm; iter%d).png" % (x_val, model_iter))
      if save_fig:
        plt.savefig("%s%d_CoM_accel_vector_field__CoM_x=%dcm__iter%dm.png" % (dir_plots, i, x_val*100, model_iter))

  # plt.show()
