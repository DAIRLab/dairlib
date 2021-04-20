import subprocess
import time
import os
from pathlib import Path

import csv
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


def build_files(bazel_file_argument):
  build_cmd = ['bazel', 'build', bazel_file_argument, ]
  build_process = subprocess.Popen(build_cmd)
  while build_process.poll() is None:  # while subprocess is alive
    time.sleep(1)


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
      time.sleep(1)
    # Kill the rest of process
    controller_process.kill()
    simulator_process.kill()
  else:
    # Wait for simluation to end
    while simulator_process.poll() is None:  # while subprocess is alive
      time.sleep(1)
    # Kill the rest of process
    planner_process.kill()
    controller_process.kill()
    logger_process.kill()


# sim_end_time is used to check if the simulation ended early
# sample_idx here is used to name the file
def eval_cost(sim_end_time, rom_iter_idx, sample_idx):
  eval_cost_cmd = [
    'bazel-bin/examples/goldilocks_models/eval_single_sim_performance',
    lcmlog_file_path(rom_iter_idx, sample_idx),
    'ROM_WALKING',
    str(rom_iter_idx),
    str(sample_idx),
    str(sim_end_time),
  ]
  eval_cost_process = subprocess.Popen(eval_cost_cmd)

  # Wait for evaluation to end
  while eval_cost_process.poll() is None:  # while subprocess is alive
    time.sleep(1)


def lcmlog_file_path(rom_iter_idx, sample_idx):
  return dir + '/lcmlog-idx_%d_%d' % (rom_iter_idx, sample_idx)


def run_sim_and_generate_cost(model_indices, sample_indices):
  sim_end_time = 8.0

  for rom_iter_idx in model_indices:
    for sample_idx in sample_indices:
      print("run sim for model %d and sample %d" % (rom_iter_idx, sample_idx))

      # Get the initial traj
      run_sim_and_controller(sim_end_time, rom_iter_idx, sample_idx, True)
      # Run the simulation
      run_sim_and_controller(sim_end_time, rom_iter_idx, sample_idx, False)

      # Evaluate the cost
      eval_cost(sim_end_time, rom_iter_idx, sample_idx)

      # Delete the lcmlog
      # os.remove(lcmlog_file_path(rom_iter_idx, sample_idx))
  print("Finished simulating. Current time = " + str(datetime.now()))


def plot_cost_vs_model_iter(model_indices, sample_idx,
    only_plot_total_cost=True, plot_nominal=False, savefig=False):
  # Get names
  with open(dir + "/cost_names.csv", newline='') as f:
    reader = csv.reader(f)
    data = list(reader)
  names = data[0]

  for i in range(len(names)):
    names[i] = names[i] + " (Drake sim)"

  # Get values
  costs = np.zeros((0, len(names)))
  for iter_idx in model_indices:
    with open(
        dir + "/" + str(iter_idx) + "_" + str(sample_idx) + "_cost_values.csv",
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
    plt.savefig(dir + "/" + figname + ".png")
  else:
    plt.show()


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
  nom_traj_dir = "../dairlib_data/goldilocks_models/planning/robot_1/models"
  filename = '_' + str(sample_idx) + '_trajopt_settings_and_cost_breakdown.txt'

  costs = np.zeros((0, 1))
  for rom_iter_idx in model_indices:
    with open(nom_traj_dir + '/' + str(rom_iter_idx) + filename, 'rt') as f:
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


if __name__ == "__main__":
  # Build files just in case forgetting
  build_files('examples/goldilocks_models/...')
  build_files('examples/Cassie:multibody_sim')

  global dir
  dir = "../dairlib_data/goldilocks_models/sim_cost_eval"

  # Create folder if not exist
  Path(dir).mkdir(parents=True, exist_ok=True)

  model_iter_idx_start = 1  # 1
  model_iter_idx_end = 20
  idx_spacing = 5

  model_indices = list(
    range(model_iter_idx_start - 1, model_iter_idx_end + 1, idx_spacing))
  model_indices[0] += 1
  # example list: [1, 5, 10, 15]

  # Remove some indices (remove by value)
  # model_indices.remove(56)

  # sample_indices = range(0, 75)
  # sample_indices = list(sample_indices)
  sample_indices = [37]

  # Toggle the functions here depending on whether to generate cost or plot cost
  run_sim_and_generate_cost(model_indices, sample_indices)

  sample_idx = 37  # related to different tasks
  plot_cost_vs_model_iter(model_indices, sample_idx, True, True, True)
  plot_cost_vs_model_iter(model_indices, sample_idx, False, True, True)
  plt.show()

  model_idx = 1
  # plot_cost_vs_task(model_idx, sample_indices, True)
