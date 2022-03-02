# This file is copied and modified from run_sim_cost_study.py 2022/02/21

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
import matplotlib
from mpl_toolkits import mplot3d
from matplotlib import cm
import matplotlib.tri as mtri
from scipy.interpolate import LinearNDInterpolator
import codecs
import math

from py_utils import FindVarValueInString


def GetSamplesToPlot(model_indices, log_indices):
  print("model_indices = ", model_indices)
  print("log_indices = ", log_indices)

  # cmt_l stores cost, model index, task value, and log index
  cmt = np.zeros((0, 2 + len(varying_task_element_indices)))
  log = np.zeros((0, 1))
  for rom_iter in model_indices:
    for idx in log_indices:
      path0 = trajopt_data_dir + '%d_%d_is_success.csv' % (rom_iter, idx)
      path1 = trajopt_data_dir + '%d_%d_%s.csv' % (rom_iter, idx, cost_file_name)
      path_task = trajopt_data_dir + '%d_%d_task.csv' % (rom_iter, idx)
      # print("path0 = ", path0)
      if os.path.exists(path0):
        # print("%s exists" % path0)
        current_cmt = np.zeros((1, 2 + len(varying_task_element_indices)))
        ### Read cost
        cost = np.loadtxt(path1, delimiter=',').item()   # 0-dim scalar
        current_cmt[0, 0] = cost
        if cost > max_cost_to_ignore:
          continue
        ### model iteration
        current_cmt[0, 1] = rom_iter
        ### Read actual task
        add_this_element = True
        col = 2
        for key in varying_task_element_indices:
          task = np.loadtxt(path_task, delimiter=',')[varying_task_element_indices[key]]
          current_cmt[0, col] = task
          if (task < min_max_task_filter_for_viz[key][0]) or (task > min_max_task_filter_for_viz[key][1]):
            add_this_element = False
          col += 1
        if not add_this_element:
          continue
        ### Assign values -- cmt
        # if (cost > 2.25) & (current_cmt[0, 2] < 0.3):
        #   continue
        # print('Add (iter,idx) = (%d,%d)' % (rom_iter, idx))
        cmt = np.vstack([cmt, current_cmt])
        ### Assign values -- log index
        log = np.vstack([log, idx])
        ### For debugging
        # if (cost > 2.25) & (current_cmt[0, 2] < 0.3):
        #   print("(iter, log) = (%.0f, %.0f) has cost %.3f (outlier)" %
        #         (current_cmt[0, 0], idx, current_cmt[0, 2]))
  print("cmt.shape = " + str(cmt.shape))

  ### Testing -- find the log idx with high cost
  cost_threshold = 3
  for i in range(len(cmt)):
    mem = cmt[i]
    if mem[0] > cost_threshold:
      print("(iter, log) = (%.0f, %.0f) has high cost %.3f" %
            (mem[1], log[i], mem[0]))
    if mem[0] < 0.4:
      print("(iter, log) = (%.0f, %.0f) has low cost %.3f" %
            (mem[1], log[i], mem[0]))

  return cmt


def AdjustSlices(model_slices):
  max_model_iter = model_indices[-1]
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    if len(cmt[cmt[:, 1] == model_iter, 2]) == 0:
      max_model_iter = model_indices[i - 1]  # this is general to 1-element case
      break

  if len(model_slices) == 0:
    n_slice = 5
    model_slices = list(range(1, max_model_iter, int(max_model_iter/n_slice)))
  elif model_slices[-1] > max_model_iter:
    model_slices = list(range(1, max_model_iter, int(max_model_iter/len(model_slices))))

  return model_slices


def Generate4dPlots(cmt):
  ### Generate 4D plots (cost, model, task1, task2)
  print("\nPlotting 4D scatter plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = fig.add_subplot(111, projection='3d')

  img = ax.scatter(cmt[:,1], cmt[:,2], cmt[:,3], c=cmt[:,0], cmap=plt.hot())
  fig.colorbar(img)

  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('pelvis height (m)')

  if save_fig:
    ax.view_init(90, -90)  # look from +z axis. model iter vs task
    plt.savefig("%smodel_ter_vs_task1_4Dscatterplot.png" % (output_dir))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    plt.savefig("%stask2_vs_task1_4Dscatterplot.png" % (output_dir))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    plt.savefig("%stask2_vs_model_iter_4Dscatterplot.png" % (output_dir))
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration


def Generate3dPlots(cmt):
  cmt = copy.deepcopy(cmt)

  # Project tasks to the specified walking height and get the corresponding cost
  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])
  z = interpolator(np.vstack((cmt[:, 1], cmt[:, 2], task_slice_value_ph * np.ones(len(cmt[:, 2])))).T)
  # Remove the rows corresponding to nan cost (from interpolation outside the region)
  cmt = cmt[~np.isnan(z), :]
  z = z[~np.isnan(z)]
  # Assign interpolated cost
  cmt[:, 0] = z
  print("interpolated cmt for 3D viz = " + str(cmt.shape))

  app = ""
  ### scatter plot
  print("\nPlotting 3D scatter plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = plt.axes(projection="3d")
  ax.scatter3D(cmt[:, 1], cmt[:, 2], cmt[:, 0], color="green")
  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('total cost')
  # plt.title("")
  if save_fig:
    ax.view_init(90, -90)  # look from +z axis. model iter vs task
    plt.savefig("%smodel_ter_vs_task_scatterplot%s_ph%.2f.png" % (output_dir, app, task_slice_value_ph))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    plt.savefig("%scost_vs_task_scatterplot%s_ph%.2f.png" % (output_dir, app, task_slice_value_ph))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    plt.savefig("%scost_vs_model_iter_scatterplot%s_ph%.2f.png" % (output_dir, app, task_slice_value_ph))
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration

  ### level set plot
  print("\nPlotting 3D level set plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = plt.axes(projection="3d")
  print("If a deprecation warning comes up here, it's from within tricontour()")
  tcf = ax.tricontour(cmt[:, 1], cmt[:, 2], cmt[:, 0], zdir='y',
                      cmap=cm.coolwarm)
  fig.colorbar(tcf)
  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('total cost')
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
  if save_fig:
    plt.savefig("%scost_vs_model_iter_contour%s_ph%.2f.png" % (output_dir, app, task_slice_value_ph))


def Generate2dPlots(model_indices, cmt):
  app = ""

  if cmt.shape[1] != 4:
    raise ValueError("The code assumes cmt is 4D (two dimensinoal task)")

  ### 2D plot (cost vs iteration)
  print("\nPlotting cost vs iterations...")

  # The line along which we evaluate the cost (using interpolation)
  n_model_iter = model_indices[-1] - model_indices[0]  # number of iterations between iter_start and iter_end
  m = np.linspace(0, n_model_iter, n_model_iter + 1)

  plt.figure(figsize=(6.4, 4.8))
  plt.rcParams.update({'font.size': 14})

  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])
  for task_slice_value in task_slice_value_list:
    t = np.array(task_slice_value).reshape(2, 1) * np.ones(n_model_iter + 1)
    print("task_slice_value = " + str(task_slice_value))
    z = interpolator(np.vstack((m, t)).T)
    plt.plot(m, z, linewidth=3, label='(sl, ph) = (%.2f, %.2f) m' % tuple(task_slice_value))
    # plt.plot(m, z, linewidth=3, label='stride length ' + str(task_slice_value) + " m (Drake sim)")
    # plt.plot(m, z, 'k-', linewidth=3, label="Drake simulation")


  # plt.xlim([0, 135])
  # plt.ylim([0.53, 1])
  plt.xlabel('model iterations')
  plt.ylabel('total cost')
  # plt.legend()
  plt.legend(loc='upper right')
  # plt.title('stride length ' + str(task_slice_value) + " m")
  # plt.title('speed %.2f m/s' % (task_slice_value / 0.4))
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%scost_vs_model_iter%s_ph%.2f.png" % (output_dir, app, task_slice_value_ph))

  ### 2D plot (cost vs tasks)
  print("\nPlotting cost vs task...")

  plt.figure(figsize=(6.4, 4.8))
  plt.rcParams.update({'font.size': 14})
  for i in range(len(model_slices)):
    model_iter = model_slices[i]
    # The line along which we evaluate the cost (using interpolation)
    m = model_iter * np.ones(500)
    t_sl = np.linspace(-0.8, 0.8, 500)
    t_ph = task_slice_value_ph * np.ones(500)

    interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])
    z = interpolator(np.vstack((m, t_sl, t_ph)).T)
    plt.plot(t_sl, z, '-',  # color=color_names[i],
             linewidth=3, label="iter " + str(model_iter))
    # if plot_nominal:
    #   interpolator = LinearNDInterpolator(nominal_cmt[:, 1:], nominal_cmt[:, 0])
    #   z = interpolator(np.vstack((m, t_sl, t_ph)).T)
    #   plt.plot(m, z, 'k--', linewidth=3, label="trajectory optimization")

  plt.xlabel('stride length (m)')
  plt.ylabel('total cost')
  plt.legend()
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%scost_vs_task_ph%.2f.png" % (output_dir, task_slice_value_ph))

  ### 2D plot (iter vs tasks)
  print("\nPlotting iterations vs task...")

  data_list = [cmt]
  title_list = ["(Open loop)", ""]
  app_list = ["", "_nom"]
  for i in range(1):
    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()

    data = copy.deepcopy(data_list[i])

    # Interpolate to get the cost at specific pelvis height
    interpolator = LinearNDInterpolator(data[:, 1:], data[:, 0])
    z = interpolator(np.vstack((data[:, 1], data[:, 2], task_slice_value_ph * np.ones(len(data[:, 2])))).T)

    # Remove the rows correponding to nan cost (from interpolation outside the region)
    data = data[~np.isnan(z), :]
    z = z[~np.isnan(z)]

    n_levels = 50
    levels = list(set(
      np.linspace(min(z), max(z), n_levels).round(
        decimals=2)))  # set() is used to get rid of duplicates
    levels.sort()
    levels[0] -= 0.01
    levels[-1] += 0.01
    # levels = list(set(np.linspace(0.4, 3, n_levels)))
    # levels.sort()
    surf = ax.tricontourf(data[:, 1], data[:, 2], z, levels=levels, cmap='coolwarm')
    fig.colorbar(surf, shrink=0.9, aspect=15)

    # plt.xlim([0, 135])
    plt.xlabel('model iterations')
    plt.ylabel('stride length (m)')
    plt.title('1D cost landscape at pelvis height %.2f m ' % task_slice_value_ph + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape_iter%s_ph%.2f.png" % (output_dir, app_list[i], task_slice_value_ph))


  ### 2D plot; cost landscape (task1 vs task2; cost visualized in contours)
  print("\nPlotting 2D cost landscape (task1 vs task2)...")
  for model_slice_value in model_slices_cost_landsacpe:
    Generate2dCostLandscape(cmt, model_slice_value)

  ### 2D plot; cost landscape comparison (task1 vs task2; cost visualized in contours)
  Generate2dCostLandscapeComparison(cmt)


def Generate2dCostLandscapeComparison(cmt):
  iter1 = 1
  iter2 = model_slices_cost_landsacpe[-1]

  ct1 = Generate2dCostLandscape(cmt, iter1, True)
  ct2 = Generate2dCostLandscape(cmt, iter2, True)

  # Grid of the whole task space
  nx, ny = (100, 100)
  stride_length_vec = np.linspace(-0.8, 0.8, nx)
  pelvis_height_vec = np.linspace(0.3, 1.3, ny)
  x, y = np.meshgrid(stride_length_vec, pelvis_height_vec)
  x = x.flatten()
  y = y.flatten()

  # Interpolate landscape1
  interpolator = LinearNDInterpolator(ct1[:, 1:3], ct1[:, 0])
  z1 = interpolator(np.vstack((x, y)).T)

  # Interpolate landscape2
  interpolator = LinearNDInterpolator(ct2[:, 1:3], ct2[:, 0])
  z2 = interpolator(np.vstack((x, y)).T)

  # z = z2/z1
  z = np.zeros(x.size)
  for i in range(x.size):
    if np.isnan(z1[i]):
      if np.isnan(z2[i]):
        z[i] = z2[i]
      else:
        z[i] = -1e-8
    else:
      if np.isnan(z2[i]):
        z[i] = np.inf
      else:
        z[i] = z2[i]/z1[i]

  # Remove the rows correponding to nan cost (from interpolation outside the region)
  x = x[~np.isnan(z)]
  y = y[~np.isnan(z)]
  z = z[~np.isnan(z)]

  # discrete color map
  levels = [0, 0.7, 0.8, 0.9, 1, 2]
  colors = ['darkgreen', 'green', 'seagreen', 'mediumseagreen', 'blue']
  cmap, norm = matplotlib.colors.from_levels_and_colors(levels, colors)
  cmap.set_over('yellow')
  cmap.set_under('red')

  plt.rcParams.update({'font.size': 14})
  fig, ax = plt.subplots()
  surf = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, levels=levels, extend='both')
  cbar = fig.colorbar(surf, shrink=0.9, aspect=10, extend='both')
  cbar.ax.set_yticklabels(['0', '0.7', '0.8', '0.9', '1', 'Inf'])

  # plt.xlim([0, 135])
  plt.xlabel('stride length (m)')
  plt.ylabel('pelvis height (m)')
  plt.title('Cost comparison between iteration %d and %d ' % (iter1, iter2) + "(Open loop)")
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%scost_landscape_comparison_btwn_iter_%d_and_%d.png" % (output_dir, iter1, iter2))



def Generate2dCostLandscape(cmt, model_slice_value, no_plotting=False):
  data_list = [cmt]
  title_list = ["(Open loop)", ""]
  app_list = ["", "_nom"]
  for i in range(1):
    data = copy.deepcopy(data_list[i])

    # Interpolate to get the cost at specific pelvis height
    interpolator = LinearNDInterpolator(data[:, 1:], data[:, 0])
    z = interpolator(np.vstack((model_slice_value * np.ones(len(data[:, 1])), data[:, 2], data[:, 3])).T)

    # Remove the rows correponding to nan cost (from interpolation outside the region)
    data = data[~np.isnan(z), :]
    z = z[~np.isnan(z)]

    if no_plotting:
      # Return [task1, task2, cost] with shape (N, 3)
      return copy.deepcopy(np.vstack([z, data[:, 2], data[:, 3]]).T)

    # get levels for contour plots
    n_levels = 50
    levels = list(set(
      np.linspace(min(z), max(z), n_levels).round(
        decimals=2)))  # set() is used to get rid of duplicates
    levels.sort()
    levels[0] -= 0.01
    levels[-1] += 0.01
    # levels = list(set(np.linspace(0.4, 3, n_levels)))
    # levels.sort()

    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()
    surf = ax.tricontourf(data[:, 2], data[:, 3], z, levels=levels, cmap='coolwarm')
    fig.colorbar(surf, shrink=0.9, aspect=15)

    # plt.xlim([0, 135])
    plt.xlabel('stride length (m)')
    plt.ylabel('pelvis height (m)')
    plt.title('Cost landscape at iteration %d ' % model_slice_value + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape%s_model_iter_%d.png" % (output_dir, app_list[i], model_slice_value))


def ComputeExpectedCostOverTask(model_indices, cmt, stride_length_range_to_average):
  if len(stride_length_range_to_average) == 0:
    return
  elif len(stride_length_range_to_average) != 2:
    raise ValueError("the range list has to be 2 dimensional")
  elif stride_length_range_to_average[0] > stride_length_range_to_average[1]:
    raise ValueError("first element should be the lower bound of the range")

  print("\nPlotting expected cost over task...")

  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])

  # Trim model iteration list with final_iter_ave_cost
  model_indices_cp = [i for i in model_indices if i <= final_iter_ave_cost]

  # Correct the range so that it's within the achieveable task space for all model iter
  viable_min = -math.inf
  viable_max = math.inf
  effective_length = len(model_indices_cp)
  for i in range(len(model_indices_cp)):
    model_iter = model_indices_cp[i]
    try:
      n_sample = 1000
      m = model_iter * np.ones(n_sample)
      t_sl = np.linspace(stride_length_range_to_average[0], stride_length_range_to_average[1], n_sample)
      t_ph = task_slice_value_ph * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_sl, t_ph)).T)
      t_sl_masked = t_sl[~np.isnan(z)]

      viable_min = max(viable_min, min(t_sl_masked))
      viable_max = min(viable_max, max(t_sl_masked))
    except ValueError:
      effective_length = i + 1
      print("Iteration %d doesn't have successful sample, so we stop plotting expected cost after this iter" % model_iter)
      break
  if viable_min > stride_length_range_to_average[0]:
    print("Warning: increase the lower bound to %f because it's outside achievable space" % viable_min)
    stride_length_range_to_average[0] = viable_min
  if viable_max < stride_length_range_to_average[1]:
    print("Warning: decrease the upper bound to %f because it's outside achievable space" % viable_max)
    stride_length_range_to_average[1] = viable_max

  ### 2D plot (averaged cost vs iteration)
  n_sample = 500
  averaged_cost = np.zeros(effective_length)
  for i in range(effective_length):
    model_iter = model_indices_cp[i]
    # The line along which we evaluate the cost (using interpolation)
    m = model_iter * np.ones(n_sample)
    t_sl = np.linspace(stride_length_range_to_average[0], stride_length_range_to_average[1], n_sample)
    t_ph = task_slice_value_ph * np.ones(n_sample)
    z = interpolator(np.vstack((m, t_sl, t_ph)).T)

    averaged_cost[i] = z.sum() / n_sample

  plt.figure(figsize=(6.4, 4.8))
  plt.plot(model_indices_cp[:effective_length], averaged_cost, 'k-', linewidth=3)
  plt.xlabel('model iteration')
  plt.ylabel('averaged cost')
  plt.title("Cost averaged over stride length [%.3f, %.3f] m" % tuple(stride_length_range_to_average))
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%save_cost_vs_model_iter_range_%.2fto%.2f_ph%.2f.png" % (output_dir, stride_length_range_to_average[0], stride_length_range_to_average[1], task_slice_value_ph))


def ComputeAchievableTaskRangeOverIter(cmt):
  print("\nPlotting achievable task range over iter...")
  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])

  ### 2D plot (task range vs iteration)
  n_sample = 1000
  max_task_value = 1
  min_task_value = -1

  delta_task = (max_task_value - min_task_value) / n_sample
  t_sl = np.linspace(min_task_value, max_task_value, n_sample)
  t_ph = task_slice_value_ph * np.ones(n_sample)

  # Get max range
  min_sl_across_iter = 1
  max_sl_across_iter = 0
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      m = model_iter * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_sl, t_ph)).T)
      t_sl_masked = t_sl[~np.isnan(z)]

      min_sl_across_iter = min(min_sl_across_iter, min(t_sl_masked))
      max_sl_across_iter = max(max_sl_across_iter, max(t_sl_masked))
    except ValueError:
      continue
  min_sl_across_iter -= delta_task
  max_sl_across_iter += delta_task

  # Get range
  task_range = np.zeros(len(model_indices))
  t_sl = np.linspace(min_sl_across_iter, max_sl_across_iter, n_sample)
  t_ph = task_slice_value_ph * np.ones(n_sample)
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      m = model_iter * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_sl, t_ph)).T)
      t_sl_masked = t_sl[~np.isnan(z)]

      task_range[i] = max(t_sl_masked) - min(t_sl_masked)
    except ValueError:
      task_range[i] = 0
      print("Iteration %d doesn't have successful sample. Set achievable task range to 0" % model_iter)

  plt.figure(figsize=(6.4, 4.8))
  plt.plot(model_indices, task_range, 'k-', linewidth=3)
  plt.xlabel('model iteration')
  plt.ylabel('achievable task space size (m)')
  plt.title("Achievable task space size (stride length)")
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%stask_space_vs_model_iter_ph%.2f.png" % (output_dir, task_slice_value_ph))


def GetVaryingTaskElementIdx(nominal_task_names):
  indices = {}
  names = ["stride_length", "pelvis_height"]
  for name in names:
    indices[name] = nominal_task_names.index(name)
  return indices


if __name__ == "__main__":
  trajopt_base_dir = "/home/yuming/workspace/dairlib_data/goldilocks_models/trajopt_cost_eval/20220218_explore_task_boundary--20220131_rom17_much_smaller_range__only_walking_forward__more_forward/"
  if len(sys.argv) == 2:
    trajopt_base_dir = sys.argv[1]
  print("trajopt_base_dir = ", trajopt_base_dir)

  # Check directories
  trajopt_data_dir = trajopt_base_dir + "robot_1/"
  output_dir = trajopt_base_dir + "plots/"
  if not os.path.exists(trajopt_data_dir):
    raise ValueError("%s doesn't exist" % trajopt_data_dir)
  Path(output_dir).mkdir(parents=True, exist_ok=True)

  ### Parameters for plotting
  model_iter_idx_start = 1
  model_iter_idx_end = 231
  model_iter_idx_delta = 10
  model_indices = list(range(model_iter_idx_start, model_iter_idx_end+1, model_iter_idx_delta))

  log_indices = list(range(int(np.loadtxt(trajopt_data_dir + "n_sample.csv"))))

  save_fig = True
  plot_main_cost = True  # main cost is the cost of which we take gradient during model optimization
  cost_file_name = "c_main" if plot_main_cost else "c"

  # 2D plot (cost vs model)
  # task_slice_value_sl = [-0.16, 0, 0.16]
  # task_slice_value_sl = [-0.2, -0.1, 0, 0.1, 0.2]
  # task_slice_value_sl = [-0.4, -0.2, 0, 0.2, 0.4]
  task_slice_value_sl = [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3]
  task_slice_value_ph = 0.95
  task_slice_value_list = [[sl, task_slice_value_ph] for sl in task_slice_value_sl]

  # 2D plot (cost vs task)
  # model_slices = []
  model_slices = [1, 50, 100, 150]
  # model_slices = [1, 25, 50, 75, 100]
  # model_slices = list(range(1, 50, 5))
  # color_names = ["darkblue", "maroon"]
  # color_names = ["k", "maroon"]

  # 2D landscape (task1 vs task2)
  # model_slices_cost_landsacpe = []
  model_slices_cost_landsacpe = [1, 2, 11, 200]

  # Expected (averaged) cost over a task range
  stride_length_range_to_average = [-0.4, 0.4]
  # stride_length_range_to_average = [0.2, 0.4]
  final_iter_ave_cost = model_iter_idx_end
  # final_iter_ave_cost = 30

  # Parameters for visualization
  max_cost_to_ignore = 10  # 2
  # mean_sl = 0.2
  # delta_sl = 0.1  # 0.1 #0.005
  # min_sl = mean_sl - delta_sl
  # max_sl = mean_sl + delta_sl
  min_sl = -100
  max_sl = 100
  min_max_task_filter_for_viz = {}
  min_max_task_filter_for_viz['stride_length'] = (min_sl, max_sl)
  min_max_task_filter_for_viz['pelvis_height'] = (0, 2)

  ### Create task list
  nominal_task_names = np.loadtxt(trajopt_data_dir + "task_names.csv", dtype=str, delimiter=',')
  # Index of task vector where we sweep through
  varying_task_element_indices = GetVaryingTaskElementIdx(list(nominal_task_names))
  print("varying_task_element_indices = " + str(varying_task_element_indices))

  ### Get samples to plot
  # cmt is a list of (model index, task value, and cost)
  cmt = GetSamplesToPlot(model_indices, log_indices)

  # Adjust slices value (for 2D plots)
  model_slices = AdjustSlices(model_slices)

  ### Plot
  Generate4dPlots(cmt)
  Generate3dPlots(cmt)
  Generate2dPlots(model_indices, cmt)

  ### Compute expected (averaged) cost
  ComputeExpectedCostOverTask(model_indices, cmt, stride_length_range_to_average)

  ### Compute task range over iteration
  ComputeAchievableTaskRangeOverIter(cmt)

  plt.show()
