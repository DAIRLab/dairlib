import subprocess
import time
import os
from pathlib import Path
from datetime import datetime

import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import yaml


def build_files(bazel_file_argument):
  build_cmd = ['bazel', 'build', bazel_file_argument, ]
  build_process = subprocess.Popen(build_cmd)
  while build_process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)


def lcmlog_file_path(rom_iter_idx, sample_idx):
  return eval_dir + '/lcmlog-idx_%d_%d' % (rom_iter_idx, sample_idx)


# Set `get_init_file` to True if you want to generate the initial traj for both
# planner and controller
# sample_idx is used to initialize the guess for the planner
def run_sim_and_controller(sim_end_time, rom_iter_idx, sample_idx,
    get_init_file):
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
    '--use_IK=false',
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
def eval_cost(sim_end_time, rom_iter_idx, sample_idx, multithread = False):
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

  # import pdb; pdb.set_trace()
  if multithread:
    return eval_cost_process
  else:
    # Wait for evaluation to end
    while eval_cost_process.poll() is None:  # while subprocess is alive
      time.sleep(0.1)


# TODO: I don't like the method of scaling the stride length and repeat the sim to get achieve the desired task
#  What I'm doing currently is running all samples for one time, and then get a 3D plot (model iter - task - cost)
#  Then we can pick a task to slice the 3D plot!
def run_sim_and_eval_cost(model_indices, sample_indices):
  n_total_sim = len(model_indices) * len(sample_indices)
  i = 0
  for rom_iter in model_indices:
    for sample in sample_indices:
      print("\n===========\n")
      # print("progress " + str(int(float(i) / n_total_sim * 100)) + "%")
      print("progress %.1f%%" % (float(i) / n_total_sim * 100))
      print("run sim for model %d and sample %d" % (rom_iter, sample))

      path = eval_dir + '/%d_%d_success.csv' % (rom_iter, sample)
      n_fail = 0
      # while True:
      while not os.path.exists(path):
        # Get the initial traj
        run_sim_and_controller(sim_end_time, rom_iter, sample, True)
        # Run the simulation
        run_sim_and_controller(sim_end_time, rom_iter, sample, False)

        # Evaluate the cost
        eval_cost(sim_end_time, rom_iter, sample)

        # Delete the lcmlog
        # os.remove(lcmlog_file_path(rom_iter_idx, sample_idx))

        if not os.path.exists(path):
          n_fail += 1
        if n_fail > 2:
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
      path = eval_dir + '/%d_%d_success.csv' % (rom_iter, sample)
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


def plot_nomial_cost(model_indices, sample_idx):
  filename = '_' + str(sample_idx) + '_trajopt_settings_and_cost_breakdown.txt'

  costs = np.zeros((0, 1))
  for rom_iter_idx in model_indices:
    with open(model_dir + '/' + str(rom_iter_idx) + filename, 'rt') as f:
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


def plot_cost_vs_model_iter(model_indices, sample_idx,
    only_plot_total_cost=True, plot_nominal=False, savefig=False):
  # Get names
  with open(eval_dir + "/cost_names.csv", newline='') as f:
    reader = csv.reader(f)
    data = list(reader)
  names = data[0]

  for i in range(len(names)):
    names[i] = names[i] + " (Drake sim)"

  # Get values
  costs = np.zeros((0, len(names)))
  for iter_idx in model_indices:
    with open(
        eval_dir + "/" + str(iter_idx) + "_" + str(
          sample_idx) + "_cost_values.csv",
        newline='') as f:
      reader = csv.reader(f)
      data = [float(x) for x in list(reader)[0]]
    costs = np.vstack([costs, np.array(data)])

  figname = "cost_vs_model_iterations"
  if only_plot_total_cost:
    figname = "total_" + figname
  plt.figure(figname, figsize=(6.4, 4.8))
  if only_plot_total_cost:
    plt.plot(model_indices, costs[:, -1], 'k-', linewidth=3)
    if plot_nominal:
      nominal_cost = plot_nomial_cost(model_indices, sample_idx)
      plt.plot(model_indices, nominal_cost, 'k--', linewidth=3)
    plt.ylabel('total cost')
    plt.xlabel('model iterations')
    plt.legend(['Drake simulation', 'trajectory opt.'])
  else:
    plt.plot(model_indices, costs)
    if plot_nominal:
      nominal_cost = plot_nomial_cost(model_indices, sample_idx)
      plt.plot(model_indices, nominal_cost)
      names = names + ["total cost (trajopt)"]
    plt.ylabel('cost')
    plt.xlabel('model iterations')
    plt.legend(names)

  if savefig:
    plt.savefig(eval_dir + "/" + figname + ".png")
  else:
    plt.show()


def plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx,
    savefig=False):
  # Parameters for visualization
  max_cost_to_ignore = 2
  mean_sl = 0.2
  delta_sl = 0.1 #0.1 #0.005
  min_sl = mean_sl - delta_sl
  max_sl = mean_sl + delta_sl

  model_task_cost = np.zeros((0, 3))
  for rom_iter in model_indices:
    for sample in sample_indices:
      path0 = eval_dir + '/%d_%d_success.csv' % (rom_iter, sample)
      path1 = eval_dir + '/%d_%d_cost_values.csv' % (rom_iter, sample)
      path2 = eval_dir + '/%d_%d_ave_stride_length.csv' % (rom_iter, sample)
      # if os.path.exists(path1) and os.path.exists(path2):
      if os.path.exists(path0) and os.path.exists(path1) and os.path.exists(
          path2):
        current_model_task_cost = np.zeros((1, 3))
        ### Read cost
        with open(path1, newline='') as f:
          reader = csv.reader(f)
          data = [float(x) for x in list(reader)[0]]
        current_model_task_cost[0, 2] = data[-1]  # Only get the total cost
        if data[-1] > max_cost_to_ignore:
          continue
        ### Read desired task
        # path = model_dir + '/%d_%d_task.csv' % (rom_iter, sample)
        # with open(path, newline='') as f:
        #   reader = csv.reader(f)
        #   data = [float(x) for x in list(reader)[0]]
        # current_model_task_cost[0, 1] = data[task_element_idx]
        ### Read actual task
        with open(path2, newline='') as f:
          reader = csv.reader(f)
          data = [float(x) for x in list(reader)[0]]
        current_model_task_cost[0, 1] = data[0]
        if (data[0] > max_sl) or (data[0] < min_sl):
          continue
        ### Read model iteration
        current_model_task_cost[0, 0] = rom_iter
        ### Assign values
        print('Add (iter,sample) = (%d,%d)' % (rom_iter, sample))
        model_task_cost = np.vstack([model_task_cost, current_model_task_cost])
      else:
        if os.path.exists(path0):
          os.remove(path0)
        if os.path.exists(path1):
          os.remove(path1)
        if os.path.exists(path2):
          os.remove(path2)

  print(model_task_cost.shape)

  # Plot 3D plot here
  fig = plt.figure(figsize=(10, 7))
  ax = plt.axes(projection="3d")
  ax.scatter3D(model_task_cost[:, 0], model_task_cost[:, 1],
    model_task_cost[:, 2], color="green")
  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('total cost')
  # plt.title("")


if __name__ == "__main__":
  # Build files just in case forgetting
  build_files('examples/goldilocks_models/...')
  build_files('examples/Cassie:multibody_sim')

  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/controller/osc_rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  model_dir = parsed_yaml_file.get('dir_model')

  eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval"

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
  sample_indices = range(1, 39, 3)
  # sample_indices = list(sample_indices)
  # sample_indices = [37]

  ### Toggle the functions here depending on whether to generate or plot cost
  # run_sim_and_eval_cost(model_indices, sample_indices)
  # run_sim_and_eval_cost([70], [34])

  # Only evaluate cost
  # eval_cost_in_multithread(model_indices, sample_indices)

  # 2D plot
  # sample_idx = 37  # related to different tasks
  # plot_cost_vs_model_iter(model_indices, sample_idx, True, True, True)
  # plot_cost_vs_model_iter(model_indices, sample_idx, False, True, True)

  # 2D plot
  # model_idx = 1
  # plot_cost_vs_task(model_idx, sample_indices, True)

  # 3D plot
  task_element_idx = 0
  plot_cost_vs_model_and_task(model_indices, sample_indices, task_element_idx)
  plt.show()
