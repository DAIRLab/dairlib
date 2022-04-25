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
import matplotlib.patches as mpatches
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
  max_model_iter_in_successful_samples = int(max(cmt[:, 1]))
  max_model_iter_in_slices = model_slices[-1]
  for i in range(len(model_slices)):
    if model_slices[i] > max_model_iter_in_successful_samples:
      max_model_iter_in_slices = model_slices[i - 1]  # this is general to 1-element case
      break

  print("max_model_iter_in_slices = ", max_model_iter_in_slices)
  if len(model_slices) == 0:
    n_slice = 5
    model_slices = list(range(1, max_model_iter_in_slices, int(max_model_iter_in_slices/n_slice)))
  else:
    model_slices = [m for m in model_slices if m <= max_model_iter_in_slices]

  return model_slices


def Generate4dPlots(cmt):
  ### Generate 4D plots (cost, model, task1, task2)
  print("\nPlotting 4D scatter plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = fig.add_subplot(111, projection='3d')

  img = ax.scatter(cmt[:,1], cmt[:,2], cmt[:,3], c=cmt[:,0], cmap=plt.hot())
  fig.colorbar(img)

  ax.set_xlabel('model iterations')
  ax.set_ylabel(name_with_unit[task_to_plot[0]])
  ax.set_zlabel(name_with_unit[task_to_plot[1]])

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
  z = interpolator(np.vstack((cmt[:, 1], cmt[:, 2], second_task_value * np.ones(len(cmt[:, 2])))).T)
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
  ax.set_ylabel(name_with_unit[task_to_plot[0]])
  ax.set_zlabel('total cost')
  # plt.title("")
  if save_fig:
    ax.view_init(90, -90)  # look from +z axis. model iter vs task
    plt.savefig("%smodel_ter_vs_task_scatterplot%s_%s%.2f.png" % (output_dir, app, name_abbrev[task_to_plot[1]], second_task_value))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    plt.savefig("%scost_vs_task_scatterplot%s_%s%.2f.png" % (output_dir, app, name_abbrev[task_to_plot[1]], second_task_value))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    plt.savefig("%scost_vs_model_iter_scatterplot%s_%s%.2f.png" % (output_dir, app, name_abbrev[task_to_plot[1]], second_task_value))
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
  ax.set_ylabel(name_with_unit[task_to_plot[0]])
  ax.set_zlabel('total cost')
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
  if save_fig:
    plt.savefig("%scost_vs_model_iter_contour%s_%s%.2f.png" % (output_dir, app, name_abbrev[task_to_plot[1]], second_task_value))


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
    plt.plot(m, z, linewidth=3, label='(%s, %s) = (%.2f, %.2f) m' % (name_abbrev[task_to_plot[0]], name_abbrev[task_to_plot[1]], task_slice_value[0], task_slice_value[1]))
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
    plt.savefig("%scost_vs_model_iter%s_%s%.2f.png" % (output_dir, app, name_abbrev[task_to_plot[1]], second_task_value))

  ### 2D plot (cost vs tasks)
  print("\nPlotting cost vs task...")

  plt.figure(figsize=(6.4, 4.8))
  plt.rcParams.update({'font.size': 14})
  for i in range(len(model_slices)):
    model_iter = model_slices[i]
    # The line along which we evaluate the cost (using interpolation)
    m = model_iter * np.ones(500)
    t_1st = np.linspace(task_boundary_outer_box[task_to_plot[0]][0], task_boundary_outer_box[task_to_plot[0]][1], 500)
    t_2nd = second_task_value * np.ones(500)

    interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])
    z = interpolator(np.vstack((m, t_1st, t_2nd)).T)
    plt.plot(t_1st, z, '-',  # color=color_names[i],
             linewidth=3, label="iter " + str(model_iter))
    # if plot_nominal:
    #   interpolator = LinearNDInterpolator(nominal_cmt[:, 1:], nominal_cmt[:, 0])
    #   z = interpolator(np.vstack((m, t_1st, t_2nd)).T)
    #   plt.plot(m, z, 'k--', linewidth=3, label="trajectory optimization")

  plt.xlabel(name_with_unit[task_to_plot[0]])
  plt.ylabel('total cost')
  plt.legend()
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%scost_vs_task_%s%.2f.png" % (output_dir, name_abbrev[task_to_plot[1]], second_task_value))

  ### 2D plot (iter vs tasks)
  print("\nPlotting iterations vs task...")

  data_list = [cmt]
  title_list = ["(Open loop)", ""]
  app_list = ["", "_nom"]
  for i in range(1):
    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()

    data = copy.deepcopy(data_list[i])

    # Interpolate to get the cost at specific value of the second task
    interpolator = LinearNDInterpolator(data[:, 1:], data[:, 0])
    z = interpolator(np.vstack((data[:, 1], data[:, 2], second_task_value * np.ones(len(data[:, 2])))).T)

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
    plt.ylabel(name_with_unit[task_to_plot[0]])
    plt.title('1D cost landscape at %s %.2f m ' % (task_to_plot[1], second_task_value) + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape_iter%s_%s%.2f.png" % (output_dir, app_list[i], name_abbrev[task_to_plot[1]], second_task_value))


  ### 2D plot; cost landscape (task1 vs task2; cost visualized in contours)
  print("\nPlotting 2D cost landscape (%s vs %s)..." % tuple(task_to_plot))
  for model_slice_value in model_slices_cost_landsacpe:
    Generate2dCostLandscape(cmt, model_slice_value)

  ### 2D plot; cost landscape comparison (task1 vs task2; cost visualized in contours)
  for model_slice_value in model_slices_cost_landsacpe:
    if model_slice_value == 1:
      continue
    Generate2dCostLandscapeComparison(cmt, model_slice_value)


def Generate2dCostLandscapeComparison(cmt, model_slice_value):
  iter1 = 1
  iter2 = model_slice_value

  ct1 = Generate2dCostLandscape(cmt, iter1, True)
  ct2 = Generate2dCostLandscape(cmt, iter2, True)

  # Grid of the whole task space
  nx, ny = (500, 500)
  first_task_vec = np.linspace(task_boundary_outer_box[task_to_plot[0]][0], task_boundary_outer_box[task_to_plot[0]][1], nx)
  second_task_vec = np.linspace(task_boundary_outer_box[task_to_plot[1]][0], task_boundary_outer_box[task_to_plot[1]][1], ny)
  x, y = np.meshgrid(first_task_vec, second_task_vec)
  x = x.flatten()
  y = y.flatten()

  # Interpolate landscape1
  interpolator = LinearNDInterpolator(ct1[:, 1:3], ct1[:, 0])
  z1 = interpolator(np.vstack((x, y)).T)

  # Interpolate landscape2
  interpolator = LinearNDInterpolator(ct2[:, 1:3], ct2[:, 0])
  z2 = interpolator(np.vstack((x, y)).T)

  # z = z2/z1
  big_val = 1000000
  small_val = -1e-8
  z = np.zeros(x.size)
  for i in range(x.size):
    if np.isnan(z1[i]):
      if np.isnan(z2[i]):
        z[i] = z2[i]
      else:
        z[i] = small_val
    else:
      if np.isnan(z2[i]):
        z[i] = big_val  # np.inf # a very big number
      else:
        z[i] = z2[i]/z1[i]

  # Remove the rows correponding to nan cost (from interpolation outside the region)
  x = x[~np.isnan(z)]
  y = y[~np.isnan(z)]
  z = z[~np.isnan(z)]

  # Colors for 0 and inf
  color_0 = (0, 0.6, 0, 0.5)  # translucent green
  color_inf = 'darkred'

  min_nonzero_ratio = min(z[np.logical_and(z > 0, z < big_val)])
  max_nonzero_ratio = max(z[np.logical_and(z > 0, z < big_val)])
  # min_nonzero_ratio = 0.5

  # Flags
  plot_the_ratio_bigger_than_1 = max_nonzero_ratio > 1
  plot_the_ratio_bigger_than_1 = True  # sometimes we want to manually set this to false because the ratio bigger than 1 was from bad solves at boundary
  plot_lost_task = True

  # discrete color map
  n_level = 6
  delta_level = (min(1, max_nonzero_ratio) - min_nonzero_ratio) / (n_level - 1)

  order = 3
  levels = []
  for i in range(n_level)[::-1]:
    levels.append(round(min(1, max_nonzero_ratio) - i * delta_level, order))
  levels[0] = levels[0] - 0.1**order
  # levels[-1] = levels[-1] + 0.1**order

  start_color = np.array([0.1, 0.1, 0.5])
  end_color = np.array([0.3, 0.3, 1])
  colors = [tuple(start_color + (end_color - start_color) * i / (n_level - 2)) for i in range(n_level - 1)]


  # Extend levels and colors for values bigger than 1
  if plot_the_ratio_bigger_than_1:
    levels.append(round(max_nonzero_ratio, order) + 0.1**order)
    colors.append('red')

  # # Extend levels and colors to include 0 and inf if we want to do it manually
  # colors.insert(0, color_0)
  # levels.insert(0, small_val-0.1)
  # colors.append(color_inf)
  # levels.append(big_val+0.1)

  # print("levels = ", levels)
  # print("colors = ", colors)
  cmap, norm = matplotlib.colors.from_levels_and_colors(levels, colors)
  cmap.set_under(color_0)
  if plot_lost_task:
    cmap.set_over(color_inf)

  plt.rcParams.update({'font.size': 14})
  fig, ax = plt.subplots()

  surf = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, levels=levels, extend='both')
  # surf = ax.tricontourf(x, y, z, cmap=cmap, norm=norm, levels=levels)

  # cbar = fig.colorbar(surf, shrink=0.9, aspect=10, extend='both')
  # cbar = fig.colorbar(surf, shrink=0.9, aspect=10)
  cbar = fig.colorbar(surf, shrink=0.9, aspect=10, extendfrac=0)  # remove the extended part from the color bar

  # Add extended color into legend manually
  new_skill_patch = mpatches.Patch(color=color_0, label='new tasks')
  if plot_lost_task:
    lost_skill_patch = mpatches.Patch(color=color_inf, label='lost tasks')
    plt.legend(handles=[new_skill_patch, lost_skill_patch])
  else:
    plt.legend(handles=[new_skill_patch])

  cbar.set_ticks([round(m, 3) for m in levels])
  cbar.ax.set_yticklabels([round(m, 3) for m in levels])

  # plt.xlim([-1, 1])
  plt.ylim([0.65, 1.05])
  plt.xlabel(name_with_unit[task_to_plot[0]])
  plt.ylabel(name_with_unit[task_to_plot[1]])
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

    # Interpolate to get the cost at specific value of the second task
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
    plt.xlabel(name_with_unit[task_to_plot[0]])
    plt.ylabel(name_with_unit[task_to_plot[1]])
    plt.title('Cost landscape at iteration %d ' % model_slice_value + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape%s_model_iter_%d.png" % (output_dir, app_list[i], model_slice_value))


def ComputeExpectedCostOverTask(model_indices, cmt, task_range_to_average_over):
  if len(task_range_to_average_over) == 0:
    return
  elif len(task_range_to_average_over) != 2:
    raise ValueError("the range list has to be 2 dimensional")
  elif task_range_to_average_over[0] > task_range_to_average_over[1]:
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
      t_1st = np.linspace(task_range_to_average_over[0], task_range_to_average_over[1], n_sample)
      t_2nd = second_task_value * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_1st, t_2nd)).T)
      t_1st_masked = t_1st[~np.isnan(z)]

      viable_min = max(viable_min, min(t_1st_masked))
      viable_max = min(viable_max, max(t_1st_masked))
    except ValueError:
      effective_length = i + 1
      print("Iteration %d doesn't have successful sample, so we stop plotting expected cost after this iter" % model_iter)
      break
  if viable_min > task_range_to_average_over[0]:
    print("Warning: increase the lower bound to %f because it's outside achievable space" % viable_min)
    task_range_to_average_over[0] = viable_min
  if viable_max < task_range_to_average_over[1]:
    print("Warning: decrease the upper bound to %f because it's outside achievable space" % viable_max)
    task_range_to_average_over[1] = viable_max

  ### 2D plot (averaged cost vs iteration)
  n_sample = 500
  averaged_cost = np.zeros(effective_length)
  for i in range(effective_length):
    model_iter = model_indices_cp[i]
    # The line along which we evaluate the cost (using interpolation)
    m = model_iter * np.ones(n_sample)
    t_1st = np.linspace(task_range_to_average_over[0], task_range_to_average_over[1], n_sample)
    t_2nd = second_task_value * np.ones(n_sample)
    z = interpolator(np.vstack((m, t_1st, t_2nd)).T)

    averaged_cost[i] = z.sum() / n_sample

  plt.figure(figsize=(6.4, 4.8))
  plt.plot(model_indices_cp[:effective_length], averaged_cost, 'k-', linewidth=3)
  plt.xlabel('model iteration')
  plt.ylabel('averaged cost')
  plt.title("Cost averaged over %s [%.3f, %.3f] m" % (task_to_plot[0], task_range_to_average_over[0], task_range_to_average_over[1]))
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%save_cost_vs_model_iter_range_%.2fto%.2f_%s%.2f.png" % (output_dir, task_range_to_average_over[0], task_range_to_average_over[1], name_abbrev[task_to_plot[1]], second_task_value))


def ComputeAchievableTaskRangeOverIter(cmt):
  print("\nPlotting achievable task range over iter...")
  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])

  ### 2D plot (task range vs iteration)
  n_sample = 1000
  max_task_value = 1
  min_task_value = -1

  delta_task = (max_task_value - min_task_value) / n_sample
  t_1st = np.linspace(min_task_value, max_task_value, n_sample)
  t_2nd = second_task_value * np.ones(n_sample)

  # Get max range
  min_sl_across_iter = 1
  max_sl_across_iter = 0
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      m = model_iter * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_1st, t_2nd)).T)
      t_1st_masked = t_1st[~np.isnan(z)]

      min_sl_across_iter = min(min_sl_across_iter, min(t_1st_masked))
      max_sl_across_iter = max(max_sl_across_iter, max(t_1st_masked))
    except ValueError:
      continue
  min_sl_across_iter -= delta_task
  max_sl_across_iter += delta_task

  # Get range
  task_range = np.zeros(len(model_indices))
  t_1st = np.linspace(min_sl_across_iter, max_sl_across_iter, n_sample)
  t_2nd = second_task_value * np.ones(n_sample)
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      m = model_iter * np.ones(n_sample)
      z = interpolator(np.vstack((m, t_1st, t_2nd)).T)
      t_1st_masked = t_1st[~np.isnan(z)]

      task_range[i] = max(t_1st_masked) - min(t_1st_masked)
    except ValueError:
      task_range[i] = 0
      print("Iteration %d doesn't have successful sample. Set achievable task range to 0" % model_iter)

  plt.figure(figsize=(6.4, 4.8))
  plt.plot(model_indices, task_range, 'k-', linewidth=3)
  plt.xlabel('model iteration')
  plt.ylabel('achievable task space size (m)')
  plt.title("Achievable task space size (%s)" % task_to_plot[0])
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%stask_space_vs_model_iter_%s%.2f.png" % (output_dir, name_abbrev[task_to_plot[1]], second_task_value))


def GetVaryingTaskElementIdx(nominal_task_names):
  indices = {}
  for name in task_to_plot:
    indices[name] = nominal_task_names.index(name)
  return indices


if __name__ == "__main__":
  trajopt_base_dir = "/home/yuming/workspace/dairlib_data/goldilocks_models/trajopt_cost_eval/20220218_explore_task_boundary--20220131_rom17_much_smaller_range__only_walking_forward__more_forward/"
  trajopt_base_dir = "/home/yuming/workspace/dairlib_data/goldilocks_models/trajopt_cost_eval/20220224_explore_task_boundary_2D--20220131_rom17_much_smaller_range__only_walking_forward__more_forward/"
  trajopt_base_dir = "/home/yuming/workspace/dairlib_data/goldilocks_models/trajopt_cost_eval/20220302_explore_task_boundary_2D_gi_tr--20220131_rom17_much_smaller_range__only_walking_forward__more_forward/"
  # trajopt_base_dir = "/home/yuming/workspace/dairlib_data/goldilocks_models/planning/robot_1/20220316_rom24_big_range/"
  trajopt_base_dir = "/home/yuming/Desktop/temp/0405/20220108_big_vel_weight_and_grad_main_cost_and_big_range_0_pelvis_omega/"
  trajopt_base_dir = "/home/yuming/Desktop/temp/20220224_explore_task_boundary_2D--20220131_rom17_much_smaller_range__only_walking_forward__more_forward/"
  if len(sys.argv) == 2:
    trajopt_base_dir = sys.argv[1]
  print("trajopt_base_dir = ", trajopt_base_dir)

  # Check directories
  trajopt_data_dir = trajopt_base_dir + "robot_1/"
  output_dir = trajopt_base_dir + "plots/"
  output_dir = "../temp_plots/"
  if not os.path.exists(trajopt_data_dir):
    raise ValueError("%s doesn't exist" % trajopt_data_dir)
  Path(output_dir).mkdir(parents=True, exist_ok=True)

  ### Parameters for plotting
  model_iter_idx_start = 1
  model_iter_idx_end = 300
  model_iter_idx_delta = 20
  model_indices = list(range(model_iter_idx_start, model_iter_idx_end+1, model_iter_idx_delta))

  log_indices = list(range(int(np.loadtxt(trajopt_data_dir + "n_sample.csv"))))

  save_fig = True
  plot_main_cost = True  # main cost is the cost of which we take gradient during model optimization
  cost_file_name = "c_main" if plot_main_cost else "c"

  # 2D plot (cost vs model)
  task_to_plot = ['stride_length', 'pelvis_height']
  # task_to_plot = ['ground_incline', 'turning_rate']
  all_task_slice_value_map = {}
  # all_task_slice_value_map['stride_length'] = [-0.16, 0, 0.16]
  # all_task_slice_value_map['stride_length'] = [-0.2, -0.1, 0, 0.1, 0.2]
  # all_task_slice_value_map['stride_length'] = [-0.4, -0.2, 0, 0.2, 0.4]
  all_task_slice_value_map['stride_length'] = [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3]
  all_task_slice_value_map['pelvis_height'] = [0.95]
  all_task_slice_value_map['ground_incline'] = [0.0]
  all_task_slice_value_map['turning_rate'] = [0.0]
  # Setup
  if len(task_to_plot) != 2:
    raise ValueError("task_to_plot needs to have length of 2")
  if len(all_task_slice_value_map[task_to_plot[1]]) != 1:
    raise ValueError("the second task can only have one slice")
  second_task_value = all_task_slice_value_map[task_to_plot[1]][0]
  task_slice_value_list = [[slice, second_task_value] for slice in all_task_slice_value_map[task_to_plot[0]]]
  name_abbrev = {'stride_length': "sl", 'pelvis_height': "ph",
                 'ground_incline': "gi", 'turning_rate': "tr"}
  name_with_unit = {'stride_length': "stride_length (m)",
                    'pelvis_height': "pelvis_height (m)",
                    'ground_incline': "ground_incline (rad)",
                    'turning_rate': "turning_rate (rad/s)"}

  # 2D plot (cost vs task)
  # model_slices = []
  model_slices = [1, 50, 100, 150, 200]
  # model_slices = [1, 25, 50, 75, 100]
  # model_slices = list(range(1, 50, 5))
  # color_names = ["darkblue", "maroon"]
  # color_names = ["k", "maroon"]

  # 2D landscape (task1 vs task2)
  # model_slices_cost_landsacpe = []
  model_slices_cost_landsacpe = [1, 2, 11, 110]
  model_slices_cost_landsacpe = [1, 11, 50, 100, 150, 200]
  # model_slices_cost_landsacpe = [1, 11, 50, 100, 150]
  # model_slices_cost_landsacpe = [1, 11, 50, 75, 90, 100, 125, 150]
  # model_slices_cost_landsacpe = [1, 10, 20, 30, 40, 50, 60]
  # model_slices_cost_landsacpe = [50]

  # Expected (averaged) cost over a task range
  task_ranges_to_average_over = {}
  task_ranges_to_average_over["stride_length"] = [-0.4, 0.4]
  # task_ranges_to_average_over["stride_length"] = [0.2, 0.4]
  task_ranges_to_average_over["pelvis_height"] = [0.3, 1.3]
  task_ranges_to_average_over["ground_incline"] = [-0.4, 0.4]
  task_ranges_to_average_over["turning_rate"] = [-4, 4]
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
  min_max_task_filter_for_viz['ground_incline'] = (-2, 2)
  min_max_task_filter_for_viz['turning_rate'] = (-5, 5)
  task_boundary_outer_box = {}
  task_boundary_outer_box['stride_length'] = (-0.8, 0.8)
  task_boundary_outer_box['pelvis_height'] = (0.3, 1.3)
  task_boundary_outer_box['ground_incline'] = (-2, 2)
  task_boundary_outer_box['turning_rate'] = (-5, 5)

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
  model_slices_cost_landsacpe = AdjustSlices(model_slices_cost_landsacpe)

  ### Plot
  Generate4dPlots(cmt)
  Generate3dPlots(cmt)
  Generate2dPlots(model_indices, cmt)

  ### Compute expected (averaged) cost
  ComputeExpectedCostOverTask(model_indices, cmt, task_ranges_to_average_over[task_to_plot[0]])

  ### Compute task range over iteration
  ComputeAchievableTaskRangeOverIter(cmt)

  plt.show()
