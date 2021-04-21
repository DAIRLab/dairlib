import subprocess
import time
import os
from pathlib import Path
from datetime import datetime

import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
import matplotlib.tri as mtri


def build_files(bazel_file_argument):
  build_cmd = ['bazel', 'build', bazel_file_argument, ]
  build_process = subprocess.Popen(build_cmd)
  while build_process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)


def lcmlog_file_path(rom_iter_idx, sample_idx):
  return eval_dir + 'lcmlog-idx_%d_%d' % (rom_iter_idx, sample_idx)


def get_nominal_task_given_sample_idx(sample_idx, task_name):
  # Get task element index by name
  names = np.loadtxt(model_dir + "task_names.csv", dtype=str, delimiter=',')
  task_element_idx = np.where(names == task_name)[0][0]

  task = np.loadtxt(model_dir + "%d_%d_task.csv" % (0, sample_idx))
  return task[task_element_idx]


# Set `get_init_file` to True if you want to generate the initial traj for both
# planner and controller
# sample_idx is used to initialize the guess for the planner
def run_sim_and_controller(sim_end_time, rom_iter_idx, sample_idx, fix_task,
    get_init_file):
  # Hacky heuristic parameter
  stride_length_scaling = 1.0
  # stride_length_scaling = 1 + min(rom_iter_idx / 30.0, 1) * 0.15

  # Get task to evaluate
  target_stride_length = get_nominal_task_given_sample_idx(sample_idx,
    'stride length')  # It's possible that we use space rather than _
  if fix_task:
    target_stride_length = parsed_yaml_file.get('stride_length')

  # simulation arguments
  target_realtime_rate = 1.0  # 0.04
  pause_second = 2.0 if get_init_file else 0
  init_traj_file = '' if get_init_file else '0_rom_trajectory'

  # planner arguments
  realtime_rate_for_time_limit = target_realtime_rate
  dynamic_time_limit = True
  use_ipopt = False
  knots_per_mode = 10
  feas_tol = 1e-2
  n_step = 2
  # time_limit is optional, set = 0 for realtime
  time_limit = 0.0 if dynamic_time_limit else 1.0 / target_realtime_rate * 0.2
  time_limit = 0.0 if get_init_file else time_limit
  planner_init_file = '' if get_init_file else '0_z.csv'

  planner_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_planner_process',
    '--fix_duration=true',
    '--zero_touchdown_impact=true',
    '--log_solver_info=false',
    '--iter=%d' % rom_iter_idx,
    '--sample=%d' % sample_idx,
    '--knots_per_mode=%d' % knots_per_mode,
    '--n_step=%d' % n_step,
    '--feas_tol=%.6f' % feas_tol,
    '--stride_length=%.3f' % target_stride_length,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--time_limit=%.3f' % time_limit,
    '--realtime_rate_for_time_limit=%.3f' % realtime_rate_for_time_limit,
    '--init_file=%s' % planner_init_file,
    '--use_ipopt=%s' % "true" if use_ipopt else str(get_init_file).lower(),
    '--log_data=%s' % str(get_init_file).lower(),
    '--run_one_loop_to_get_init_file=%s' % str(get_init_file).lower(),
  ]
  controller_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_controller',
    '--channel_u=ROM_WALKING',
    '--const_walking_speed=true',
    '--stride_length=%.3f' % target_stride_length,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--iter=%d' % rom_iter_idx,
    '--init_traj_file_name=%s' % init_traj_file,
  ]
  simulator_cmd = [
    'bazel-bin/examples/Cassie/multibody_sim',
    '--channel_u=ROM_WALKING',
    '--end_time=%.3f' % sim_end_time,
    '--pause_second=%.3f' % pause_second,
    '--init_height=%.3f' % 1.0,
    '--target_realtime_rate=%.3f' % target_realtime_rate,
  ]
  lcm_logger_cmd = [
    'lcm-logger',
    '-f',
    lcmlog_file_path(rom_iter_idx, sample_idx),
  ]

  planner_process = subprocess.Popen(planner_cmd)
  controller_process = subprocess.Popen(controller_cmd)
  if not get_init_file:
    logger_process = subprocess.Popen(lcm_logger_cmd)
  time.sleep(3)
  simulator_process = subprocess.Popen(simulator_cmd)

  if get_init_file:
    # Wait for planner to end
    while planner_process.poll() is None:  # while subprocess is alive
      time.sleep(0.1)
    # Kill the rest of process
    controller_process.kill()
    simulator_process.kill()
  else:
    # Wait for simluation to end
    while simulator_process.poll() is None:  # while subprocess is alive
      time.sleep(0.1)
    # Kill the rest of process
    planner_process.kill()
    controller_process.kill()
    logger_process.kill()


# sim_end_time is used to check if the simulation ended early
# sample_idx here is used to name the file
def eval_cost(sim_end_time, rom_iter_idx, sample_idx, multithread=False):
  eval_cost_cmd = [
    'bazel-bin/examples/goldilocks_models/eval_single_sim_performance',
    lcmlog_file_path(rom_iter_idx, sample_idx),
    'ROM_WALKING',
    str(rom_iter_idx),
    str(sample_idx),
    str(sim_end_time),
  ]
  print(' '.join(eval_cost_cmd))
  eval_cost_process = subprocess.Popen(eval_cost_cmd)

  if multithread:
    return eval_cost_process
  else:
    # Wait for evaluation to end
    while eval_cost_process.poll() is None:  # while subprocess is alive
      time.sleep(0.1)


# TODO: I don't like the method of scaling the stride length and repeat the sim to get achieve the desired task
#  What I'm doing currently is running all samples for one time, and then get a 3D plot (model iter - task - cost)
#  Then we can pick a task to slice the 3D plot!
def run_sim_and_eval_cost(model_indices, sample_indices, do_eval_cost=False):
  fix_task = False
  max_n_fail = 0

  n_total_sim = len(model_indices) * len(sample_indices)
  i = 0
  for rom_iter in model_indices:
    for sample in sample_indices:
      print("\n===========\n")
      # print("progress " + str(int(float(i) / n_total_sim * 100)) + "%")
      print("progress %.1f%%" % (float(i) / n_total_sim * 100))
      print("run sim for model %d and sample %d" % (rom_iter, sample))

      path = eval_dir + '%d_%d_success.csv' % (rom_iter, sample)
      n_fail = 0
      # while True:
      while not os.path.exists(path):
        # Get the initial traj
        run_sim_and_controller(sim_end_time, rom_iter, sample, fix_task, True)
        # Run the simulation
        run_sim_and_controller(sim_end_time, rom_iter, sample, fix_task, False)

        # Evaluate the cost
        if do_eval_cost:
          eval_cost(sim_end_time, rom_iter, sample)

        # Delete the lcmlog
        # os.remove(lcmlog_file_path(rom_iter_idx, sample_idx))

        if not os.path.exists(path):
          n_fail += 1
        if n_fail > max_n_fail:
          break
      i += 1

  print("Finished evaluating. Current time = " + str(datetime.now()))


# This function assumes that simulation has been run and there exist lcm logs
def eval_cost_in_multithread(model_indices, sample_indices):
  working_threads = []
  n_max_thread = 12

  n_total_sim = len(model_indices) * len(sample_indices)
  i = 0
  for rom_iter in model_indices:
    for sample in sample_indices:
      print("\n===========\n")
      # print("progress " + str(int(float(i) / n_total_sim * 100)) + "%")
      print("progress %.1f%%" % (float(i) / n_total_sim * 100))
      print("run sim for model %d and sample %d" % (rom_iter, sample))

      # Evaluate the cost
      path = eval_dir + '%d_%d_success.csv' % (rom_iter, sample)
      if not os.path.exists(path):
        working_threads.append(eval_cost(sim_end_time, rom_iter, sample, True))
      i += 1

      # Wait for threads to finish once is more than n_max_thread
      while len(working_threads) >= n_max_thread:
        for j in range(len(working_threads)):
          if working_threads[j].poll() is None:  # subprocess is alive
            time.sleep(0.1)
          else:
            del working_threads[j]
            break

  print("Wait for all threads to join")
  while len(working_threads) > 0:
    for j in range(len(working_threads)):
      if working_threads[j].poll() is None:  # subprocess is alive
        time.sleep(0.1)
      else:
        del working_threads[j]
        break
  print("Finished evaluating. Current time = " + str(datetime.now()))


def find_cost_in_string(file_string, string_to_search):
  # We search from the end of the file
  word_location = file_string.rfind(string_to_search)
  number_idx_start = 0
  number_idx_end = 0
  idx = word_location
  while True:
    if file_string[idx] == '=':
      number_idx_start = idx
    elif file_string[idx] == '\n':
      number_idx_end = idx
      break
    idx += 1
  cost_value = float(file_string[number_idx_start + 1: number_idx_end])
  return cost_value


def plot_nominal_cost(model_indices, sample_idx):
  filename = '_' + str(sample_idx) + '_trajopt_settings_and_cost_breakdown.txt'

  costs = np.zeros((0, 1))
  for rom_iter_idx in model_indices:
    with open(model_dir + str(rom_iter_idx) + filename, 'rt') as f:
      contents = f.read()
    cost_x = find_cost_in_string(contents, "cost_x =")
    cost_u = find_cost_in_string(contents, "cost_u =")
    total_cost = cost_x + cost_u
    costs = np.vstack([costs, total_cost])

  # figname = "Nominal cost over model iterations"
  # plt.figure(figname, figsize=(6.4, 4.8))
  # plt.plot(model_indices, costs)
  # plt.ylabel('cost')
  # plt.xlabel('model iterations')
  # plt.legend(["total_cost"])
  # plt.show()
  return costs


def plot_cost_vs_model_iter_given_a_sample_idx(model_indices, sample_idx,
    only_plot_total_cost=True, plot_nominal=False, savefig=False):
  # Get names
  names = np.loadtxt(eval_dir + "cost_names.csv", dtype=str,
    delimiter=',').tolist()

  for i in range(len(names)):
    names[i] = names[i] + " (Drake sim)"

  # Get values
  costs = np.zeros((0, len(names)))
  for iter_idx in model_indices:
    path = eval_dir + "%d_%d_cost_values.csv" % (iter_idx, sample_idx)
    costs = np.vstack([costs, np.loadtxt(path, delimiter=',')])

  figname = "cost_vs_model_iterations"
  if only_plot_total_cost:
    figname = "total_" + figname
  plt.figure(figname, figsize=(6.4, 4.8))
  if only_plot_total_cost:
    plt.plot(model_indices, costs[:, -1], 'k-', linewidth=3)
    if plot_nominal:
      nominal_cost = plot_nominal_cost(model_indices, sample_idx)
      plt.plot(model_indices, nominal_cost, 'k--', linewidth=3)
    plt.ylabel('total cost')
    plt.xlabel('model iterations')
    plt.legend(['Drake simulation', 'trajectory opt.'])
  else:
    plt.plot(model_indices, costs)
    if plot_nominal:
      nominal_cost = plot_nominal_cost(model_indices, sample_idx)
      plt.plot(model_indices, nominal_cost)
      names = names + ["total cost (trajopt)"]
    plt.ylabel('cost')
    plt.xlabel('model iterations')
    plt.legend(names)

  if savefig:
    plt.savefig(eval_dir + figname + ".png")
  else:
    plt.show()


def plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    plot_3d = True, plot_nominal=False, save=False):
  # Parameters for visualization
  max_cost_to_ignore = 3  # 2
  mean_sl = 0.2
  delta_sl = 0.1  # 0.1 #0.005
  min_sl = mean_sl - delta_sl
  max_sl = mean_sl + delta_sl
  # min_sl = -100
  # max_sl = 100

  # mtc that stores model index, task value and cost
  mtc = np.zeros((0, 3))
  for rom_iter in model_indices:
    for sample in sample_indices:
      path0 = eval_dir + '%d_%d_success.csv' % (rom_iter, sample)
      path1 = eval_dir + '%d_%d_cost_values.csv' % (rom_iter, sample)
      path2 = eval_dir + '%d_%d_ave_stride_length.csv' % (rom_iter, sample)
      # if os.path.exists(path1) and os.path.exists(path2):
      if os.path.exists(path0) and os.path.exists(path1) and os.path.exists(
          path2):
        current_mtc = np.zeros((1, 3))
        ### Read cost
        cost = np.loadtxt(path1, delimiter=',')
        current_mtc[0, 2] = cost[-1]
        if cost[-1] > max_cost_to_ignore:
          continue
        ### Read desired task
        # task = np.loadtxt(model_dir + "%d_%d_task.csv" % (rom_iter, sample))
        # current_mtc[0, 1] = task[task_element_idx]
        ### Read actual task
        task = np.loadtxt(path2, delimiter=',').item()  # 0-dim scalar
        current_mtc[0, 1] = task
        if (task > max_sl) or (task < min_sl):
          continue
        ### Read model iteration
        current_mtc[0, 0] = rom_iter
        ### Assign values
        # print('Add (iter,sample) = (%d,%d)' % (rom_iter, sample))
        mtc = np.vstack([mtc, current_mtc])
      else:
        # It's not suppose to get here. (most likely the experiement didn't start with an empty folder)
        if os.path.exists(path0):
          os.remove(path0)
        if os.path.exists(path1):
          os.remove(path1)
        if os.path.exists(path2):
          os.remove(path2)
  print(mtc.shape)

  nominal_mtc = np.zeros((0, 3))
  if plot_nominal:
    for rom_iter in model_indices:
      for sample in sample_indices:
        sub_mtc = np.zeros((1, 3))
        ### Read cost
        cost = np.loadtxt(model_dir + "%d_%d_c.csv" % (rom_iter, sample))
        sub_mtc[0, 2] = cost.item()
        if cost.item() > max_cost_to_ignore:
          continue
        ### Read nominal task
        task = np.loadtxt(model_dir + "%d_%d_task.csv" % (rom_iter, sample))[
          task_element_idx]
        sub_mtc[0, 1] = task
        if (task > max_sl) or (task < min_sl):
          continue
        ### Read model iteration
        sub_mtc[0, 0] = rom_iter
        ### Assign values
        nominal_mtc = np.vstack(
          [nominal_mtc, sub_mtc])
  print(nominal_mtc.shape)

  # Plot
  app = "_w_nom" if plot_nominal else ""
  if plot_3d:
    fig = plt.figure(figsize=(10, 7))

    ###
    ax = plt.axes(projection="3d")
    ax.scatter3D(mtc[:, 0], mtc[:, 1],
      mtc[:, 2], color="green")
    if plot_nominal:
      ax.scatter3D(nominal_mtc[:, 0], nominal_mtc[:, 1], nominal_mtc[:, 2], "b")
    ax.set_xlabel('model iterations')
    ax.set_ylabel('stride length (m)')
    ax.set_zlabel('total cost')
    # plt.title("")
    ax.view_init(90, -90)  # look from +z axis. model iter vs task
    if save:
      plt.savefig("%smodel_ter_vs_task_scatterplot%s.png" % (eval_dir, app))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    if save:
      plt.savefig("%scost_vs_task_scatterplot%s.png" % (eval_dir, app))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    if save:
      plt.savefig("%scost_vs_model_iter_scatterplot%s.png" % (eval_dir, app))

    ###
    ax = plt.axes(projection="3d")
    if plot_nominal:
      # tcf = ax.tricontour(nominal_mtc[:, 0], nominal_mtc[:, 1],
      #   nominal_mtc[:, 2], zdir='y', cmap=cm.coolwarm)
      ax.scatter3D(nominal_mtc[:, 0], nominal_mtc[:, 1],
        nominal_mtc[:, 2], "b")
      # tcf = ax.plot_trisurf(nominal_mtc[:, 0], nominal_mtc[:, 1],
      #   nominal_mtc[:, 2], cmap=cm.coolwarm)
      pass
    tcf = ax.tricontour(mtc[:, 0], mtc[:, 1],
      mtc[:, 2], zdir='y', cmap=cm.coolwarm)
    fig.colorbar(tcf)
    ax.set_xlabel('model iterations')
    ax.set_ylabel('stride length (m)')
    ax.set_zlabel('total cost')
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    if save:
      plt.savefig("%scost_vs_model_iter_contour%s.png" % (eval_dir, app))

  else:
    # The line along which we evaluate the cost (using interpolation)
    task = 0.21
    x = np.linspace(0, 100, 101)
    y = task * np.ones(101)

    plt.figure(figsize=(6.4, 4.8))
    triang = mtri.Triangulation(mtc[:, 0], mtc[:, 1])
    interpolator = mtri.LinearTriInterpolator(triang, mtc[:, 2])
    z = interpolator(x, y)
    plt.plot(x, z, label="Drake simulation")
    if plot_nominal:
      triang = mtri.Triangulation(nominal_mtc[:, 0], nominal_mtc[:, 1])
      interpolator = mtri.LinearTriInterpolator(triang, nominal_mtc[:, 2])
      z = interpolator(x, y)
      plt.plot(x, z, label="trajectory optimization")

    plt.xlabel('model iterations')
    plt.ylabel('total cost')
    plt.legend()
    plt.title('stride length ' + str(task))
    if save:
      plt.savefig("%scost_vs_model_iter%s.png" % (eval_dir, app))


if __name__ == "__main__":
  # Build files just in case forgetting
  build_files('examples/goldilocks_models/...')
  build_files('examples/Cassie:multibody_sim')

  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/controller/osc_rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  model_dir = parsed_yaml_file.get('dir_model')

  eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval/"

  # global parameters
  sim_end_time = 8.0

  # Create folder if not exist
  Path(eval_dir).mkdir(parents=True, exist_ok=True)

  model_iter_idx_start = 1  # 1
  model_iter_idx_end = 100
  idx_spacing = 5

  model_indices = list(
    range(model_iter_idx_start - 1, model_iter_idx_end + 1, idx_spacing))
  model_indices[0] += 1
  # example list: [1, 5, 10, 15]

  # Remove some indices (remove by value)
  # model_indices.remove(56)

  # sample_indices = range(0, 39)
  # sample_indices = range(1, 5, 3)
  sample_indices = range(1, 39, 3)
  # sample_indices = range(1, 75, 3)
  # sample_indices = list(sample_indices)
  # sample_indices = [37]
  # TODO: automatically find all indices that has flat ground

  ### Toggle the functions here to run simulation or evaluate cost
  # run_sim_and_eval_cost(model_indices, sample_indices)
  # run_sim_and_eval_cost([70], [34])

  # Only evaluate cost
  # eval_cost_in_multithread(model_indices, sample_indices)

  ### Plotting
  print("Nominal cost is from: " + model_dir)
  print("Simulation cost is from: " + eval_dir)

  # 2D plot
  # sample_idx = 37  # related to different tasks
  # plot_cost_vs_model_iter_given_a_sample_idx(model_indices, sample_idx, True, True, True)
  # plot_cost_vs_model_iter_given_a_sample_idx(model_indices, sample_idx, False, True, True)

  # 2D plot
  # model_idx = 1
  # plot_cost_vs_task(model_idx, sample_indices, True)

  # Save plots
  task_element_idx = 0
  plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    True, True, True)
  plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    True, False, True)
  plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    False, True, True)
  plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    False, False, True)

  # 3D plot
  # plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
  #   True, True, False)
  # plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
  #   True, False, False)
  plt.show()

