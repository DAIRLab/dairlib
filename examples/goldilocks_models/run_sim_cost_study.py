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


def lcmlog_file_path(rom_iter_idx, task_idx):
  return eval_dir + 'lcmlog-idx_%d_%d' % (rom_iter_idx, task_idx)


def get_nominal_task_given_sample_idx(sample_idx, name):
  # Get task element index by name
  task_element_idx = np.where(task_names == name)[0][0]
  task = np.loadtxt(model_dir + "%d_%d_task.csv" % (0, sample_idx))
  return task[task_element_idx]


# Set `get_init_file` to True if you want to generate the initial traj for both
# planner and controller
# `sample_idx` is used for planner's initial guess and cost regularization term
def run_sim_and_controller(sim_end_time, task_value, log_idx, rom_iter_idx,
    sample_idx, get_init_file):
  # Hacky heuristic parameter
  stride_length_scaling = 1.0
  # stride_length_scaling = 1 + min(rom_iter_idx / 30.0, 1) * 0.15

  # simulation arguments
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
    '--stride_length=%.3f' % task_value,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--time_limit=%.3f' % time_limit,
    '--realtime_rate_for_time_limit=%.3f' % realtime_rate_for_time_limit,
    '--init_file=%s' % planner_init_file,
    '--use_ipopt=%s' % "true" if use_ipopt else str(get_init_file).lower(),
    '--log_data=%s' % str(get_init_file).lower(),
    '--run_one_loop_to_get_init_file=%s' % str(get_init_file).lower(),
    '--spring_model=%s' % str(spring_model).lower(),
  ]
  controller_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_controller',
    '--channel_u=ROM_WALKING',
    '--const_walking_speed=true',
    '--stride_length=%.3f' % task_value,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--iter=%d' % rom_iter_idx,
    '--init_traj_file_name=%s' % init_traj_file,
    '--spring_model=%s' % str(spring_model).lower(),
    '--get_swing_foot_from_planner=%s' % str(foot_step_from_planner).lower(),
  ]
  simulator_cmd = [
    'bazel-bin/examples/Cassie/multibody_sim',
    '--channel_u=ROM_WALKING',
    '--end_time=%.3f' % sim_end_time,
    '--pause_second=%.3f' % pause_second,
    '--init_height=%.3f' % 1.0,
    '--target_realtime_rate=%.3f' % target_realtime_rate,
    '--spring_model=%s' % str(spring_model).lower(),
  ]
  lcm_logger_cmd = [
    'lcm-logger',
    '-f',
    lcmlog_file_path(rom_iter_idx, log_idx),
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
def eval_cost(sim_end_time, rom_iter_idx, log_idx, multithread=False):
  eval_cost_cmd = [
    'bazel-bin/examples/goldilocks_models/eval_single_sim_performance',
    lcmlog_file_path(rom_iter_idx, log_idx),
    'ROM_WALKING',
    str(rom_iter_idx),
    str(log_idx),
    str(sim_end_time),
    str(spring_model),
  ]
  print(' '.join(eval_cost_cmd))
  eval_cost_process = subprocess.Popen(eval_cost_cmd)

  if multithread:
    return eval_cost_process
  else:
    # Wait for evaluation to end
    while eval_cost_process.poll() is None:  # while subprocess is alive
      time.sleep(0.1)


# sample_indices for visualization
def CollectAllSampleIndicesFromTrajopt(task, task_tolerance):
  # We pick iter 1 since we assume we are using grid tasks (index will be the same across different model iteration)
  # TODO: change this if not using grid task anymore
  rom_iter = 1

  j = 0
  sample_indices = []
  while True:
    path = model_dir + "%d_%d_task.csv" % (rom_iter, j)
    if os.path.exists(path):
      trajopt_task = np.loadtxt(path)
      task_diff = trajopt_task - task
      task_diff[varying_task_element_idx] = 0
      if np.linalg.norm(task_diff) < task_tolerance:
        sample_indices.append(j)
      j += 1
    else:
      break

  if len(sample_indices) == 0:
    raise ValueError("ERROR: no such task in trajopt")

  return sample_indices


# sample_indices for simulation
def ConstructSampleIndicesGivenModelAndTask(model_indices, task_list):
  sample_indices = np.zeros((len(model_indices), len(task_list)),
    dtype=np.dtype(int))
  for i in range(len(model_indices)):
    for j in range(len(task_list)):
      sample_indices[i, j] = GetSampleIndexGivenTask(model_indices[i],
        task_list[j])
  return sample_indices


# Get trajopt sample idx with the most similar task
def GetSampleIndexGivenTask(rom_iter, task):
  j = 0
  dist_list = []
  while True:
    path = model_dir + "%d_%d_task.csv" % (rom_iter, j)
    # print("try " + path)
    if os.path.exists(path):
      trajopt_task = np.loadtxt(path)
      dist_list.append(np.linalg.norm(trajopt_task - task))
      j += 1
    else:
      break
  # print("dist_list = ")
  # print(dist_list)
  if len(dist_list) == 0:
    raise ValueError("ERROR: This path doesn't exist: " + path)
  sample_idx = np.argmin(np.array(dist_list))

  return sample_idx


def SaveLogCorrespondence():
  msg = "log #%d to #%d: %s ranges from %.3f to %.3f\n" % (
    log_indices[0], log_indices[-1], task_names[varying_task_element_idx],
    task_list[0, varying_task_element_idx],
    task_list[-1, varying_task_element_idx])
  print(msg)
  f = open(eval_dir + "task_log_correspondence.txt", "a")
  f.write(msg)
  f.close()


def run_sim_and_eval_cost(model_indices, log_indices, task_list,
    do_eval_cost=False):
  # parameters
  max_n_fail = 0

  SaveLogCorrespondence()

  ### Construct sample indices from the task list for simulation
  # `sample_idx` is for planner's initial guess and cost regularization term
  sample_indices = ConstructSampleIndicesGivenModelAndTask(model_indices,
    task_list)
  print("sample_indices = \n" + str(sample_indices))

  ### Start simulation
  n_total_sim = len(model_indices) * len(task_list)
  counter = 0
  for i in range(len(model_indices)):
    for j in range(len(task_list)):
      rom_iter = model_indices[i]
      task = task_list[j]
      sample_idx = sample_indices[i][j]
      log_idx = log_indices[j]

      print("\n===========\n")
      print("progress %.1f%%" % (float(counter) / n_total_sim * 100))
      print("run sim for model %d and task %.3f" % \
            (rom_iter, task[varying_task_element_idx]))

      path = eval_dir + '%d_%d_success.csv' % (rom_iter, log_idx)
      n_fail = 0
      # while True:
      while not os.path.exists(path):
        # Get the initial traj
        run_sim_and_controller(sim_end_time, task[varying_task_element_idx],
          log_idx, rom_iter, sample_idx, True)
        # Run the simulation
        run_sim_and_controller(sim_end_time, task[varying_task_element_idx],
          log_idx, rom_iter, sample_idx, False)

        # Evaluate the cost
        if do_eval_cost:
          eval_cost(sim_end_time, rom_iter, log_idx)

        # Delete the lcmlog
        # os.remove(lcmlog_file_path(rom_iter_idx, log_idx))

        if not os.path.exists(path):
          n_fail += 1
        if n_fail > max_n_fail:
          break
      counter += 1

  print("Finished evaluating. Current time = " + str(datetime.now()))


# This function assumes that simulation has been run and there exist lcm logs
def eval_cost_in_multithread(model_indices, log_indices):
  working_threads = []
  n_max_thread = 12

  n_total_sim = len(model_indices) * len(log_indices)
  counter = 0
  for rom_iter in model_indices:
    for idx in log_indices:
      print("\n===========\n")
      print("progress %.1f%%" % (float(counter) / n_total_sim * 100))
      print("run sim for model %d and log %d" % (rom_iter, idx))

      # Evaluate the cost
      path = eval_dir + '%d_%d_success.csv' % (rom_iter, idx)
      if not os.path.exists(path):
        working_threads.append(eval_cost(sim_end_time, rom_iter, idx, True))
      counter += 1

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


def plot_cost_vs_model_and_task(model_indices, log_indices, sample_indices=[],
    plot_3d=True):
  # Parameters for visualization
  max_cost_to_ignore = 100  # 2
  mean_sl = 0.2
  delta_sl = 0.1  # 0.1 #0.005
  min_sl = mean_sl - delta_sl
  max_sl = mean_sl + delta_sl
  min_sl = -100
  max_sl = 100
  # min_sl = 0.15
  # max_sl = -0.31

  # mtcl stores model index, task value, cost, and log index
  mtcl = np.zeros((0, 4))
  for rom_iter in model_indices:
    for idx in log_indices:
      path0 = eval_dir + '%d_%d_success.csv' % (rom_iter, idx)
      path1 = eval_dir + '%d_%d_cost_values.csv' % (rom_iter, idx)
      path2 = eval_dir + '%d_%d_ave_stride_length.csv' % (rom_iter, idx)
      if os.path.exists(path0):
        current_mtcl = np.zeros((1, 4))
        ### Read cost
        cost = np.loadtxt(path1, delimiter=',')
        current_mtcl[0, 2] = cost[-1]
        if cost[-1] > max_cost_to_ignore:
          continue
        ### Read actual task
        task = np.loadtxt(path2, delimiter=',').item()  # 0-dim scalar
        current_mtcl[0, 1] = task
        if (task > max_sl) or (task < min_sl):
          continue
        ### model iteration
        current_mtcl[0, 0] = rom_iter
        ### log index
        current_mtcl[0, 3] = idx
        ### Assign values
        # print('Add (iter,idx) = (%d,%d)' % (rom_iter, idx))
        mtcl = np.vstack([mtcl, current_mtcl])
  print("mtcl.shape = " + str(mtcl.shape))

  # nominal_mtc stores model index, task value, and cost from trajopt
  nominal_mtc = np.zeros((0, 3))
  if plot_nominal:
    for rom_iter in model_indices:
      for i in range(len(sample_indices)):
        sub_mtc = np.zeros((1, 3))
        ### Read cost
        cost = plot_nominal_cost([rom_iter], sample_indices[i])[0][0]
        sub_mtc[0, 2] = cost
        if cost.item() > max_cost_to_ignore:
          continue
        ### Read nominal task
        task = np.loadtxt(model_dir + "%d_%d_task.csv" % ( \
          rom_iter, sample_indices[i]))[varying_task_element_idx]
        sub_mtc[0, 1] = task
        if (task > max_sl) or (task < min_sl):
          continue
        ### Read model iteration
        sub_mtc[0, 0] = rom_iter
        ### Assign values
        nominal_mtc = np.vstack([nominal_mtc, sub_mtc])
  print("nominal_mtc.shape = " + str(nominal_mtc.shape))

  # Plot
  app = "_w_nom" if plot_nominal else ""
  if plot_3d:
    ### scatter plot
    fig = plt.figure(figsize=(10, 7))
    ax = plt.axes(projection="3d")
    ax.scatter3D(mtcl[:, 0], mtcl[:, 1], mtcl[:, 2], color="green")
    if plot_nominal:
      ax.scatter3D(nominal_mtc[:, 0], nominal_mtc[:, 1], nominal_mtc[:, 2], "b")
    ax.set_xlabel('model iterations')
    ax.set_ylabel('stride length (m)')
    ax.set_zlabel('total cost')
    # plt.title("")
    if save_fig:
      ax.view_init(90, -90)  # look from +z axis. model iter vs task
      plt.savefig("%smodel_ter_vs_task_scatterplot%s.png" % (eval_dir, app))
      ax.view_init(0, 0)  # look from x axis. cost vs task
      plt.savefig("%scost_vs_task_scatterplot%s.png" % (eval_dir, app))
      ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
      plt.savefig("%scost_vs_model_iter_scatterplot%s.png" % (eval_dir, app))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration

    ### level set plot
    fig = plt.figure(figsize=(10, 7))
    ax = plt.axes(projection="3d")
    if plot_nominal:
      # tcf = ax.tricontour(nominal_mtc[:, 0], nominal_mtc[:, 1],
      #   nominal_mtc[:, 2], zdir='y', cmap=cm.coolwarm)
      ax.scatter3D(nominal_mtc[:, 0], nominal_mtc[:, 1], nominal_mtc[:, 2], "b")
      # tcf = ax.plot_trisurf(nominal_mtc[:, 0], nominal_mtc[:, 1],
      #   nominal_mtc[:, 2], cmap=cm.coolwarm)
      pass
    tcf = ax.tricontour(mtcl[:, 0], mtcl[:, 1], mtcl[:, 2], zdir='y',
      cmap=cm.coolwarm)
    fig.colorbar(tcf)
    ax.set_xlabel('model iterations')
    ax.set_ylabel('stride length (m)')
    ax.set_zlabel('total cost')
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    if save_fig:
      plt.savefig("%scost_vs_model_iter_contour%s.png" % (eval_dir, app))

  else:
    ### 2D plot

    # The line along which we evaluate the cost (using interpolation)
    x = np.linspace(0, 100, 101)
    y = task_slice_value * np.ones(101)

    plt.figure(figsize=(6.4, 4.8))
    plt.rcParams.update({'font.size': 14})
    triang = mtri.Triangulation(mtcl[:, 0], mtcl[:, 1])
    interpolator = mtri.LinearTriInterpolator(triang, mtcl[:, 2])
    z = interpolator(x, y)
    plt.plot(x, z, 'k-', linewidth=3, label="Drake simulation")
    if plot_nominal:
      triang = mtri.Triangulation(nominal_mtc[:, 0], nominal_mtc[:, 1])
      interpolator = mtri.LinearTriInterpolator(triang, nominal_mtc[:, 2])
      z = interpolator(x, y)
      plt.plot(x, z, 'k--', linewidth=3, label="trajectory optimization")
      # plt.plot(model_indices, nominal_mtc[:, 2], 'k--', linewidth=3, label="trajectory optimization")

    plt.xlabel('model iterations')
    plt.ylabel('total cost')
    plt.legend()
    plt.title('stride length ' + str(task_slice_value))
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_vs_model_iter%s.png" % (eval_dir, app))

  ### Testing -- find the log idx with high cost
  cost_threshold = 3
  for mem in mtcl:
    if mem[2] > cost_threshold:
      print("(iter, log) = (%.0f, %.0f) has high cost %.3f" %
            (mem[0], mem[3], mem[2]))


def GetVaryingTaskElementIdx(task_list):
  task_list_copy_for_test = np.copy(task_list)
  # Default to the first element in case all task element are fixed
  task_element_idx = 0

  found_varying_task = False
  for i in range(len(task_names)):
    task_list_copy_for_test[:, i] -= task_list_copy_for_test[0, i]
    if np.linalg.norm(task_list_copy_for_test[:, i]) == 0:
      continue
    else:
      if found_varying_task:
        raise ValueError("ERROR: task element #%d is not fixed" % i)
      else:
        task_element_idx = i
        found_varying_task = True
  return task_element_idx


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
  # eval_dir = "/home/yuming/Desktop/temp/"
  # eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval_2/"
  # eval_dir = "/home/yuming/Desktop/temp/3/sim_cost_eval_20210507/sim_cost_eval/"

  # global parameters
  sim_end_time = 8.0
  spring_model = True
  # Parameters that are modified often
  target_realtime_rate = 0.2  # 0.04
  foot_step_from_planner = True

  ### parameters for model, task, and log indices
  # Model iteration list
  model_iter_idx_start = 1  # 0
  model_iter_idx_end = 10
  idx_spacing = 1

  # Task list
  n_task = 15
  stride_length = np.linspace(-0.6, 0.6, n_task)
  # stride_length = np.linspace(-0.2, -0.1, n_task)
  # stride_length = np.linspace(-0.3, 0, n_task, endpoint=False)
  # stride_length = np.linspace(0.4, 0.5, n_task)
  # stride_length = np.linspace(0, 0, n_task)
  stride_length = np.hstack([np.linspace(-0.6, -0.4, n_task, endpoint=False),
                             -np.linspace(-0.6, -0.4, n_task, endpoint=False)])
  ground_incline = 0.0
  duration = 0.4
  turning_rate = 0.0

  # log indices
  log_idx_offset = 0  # 0

  ### Parameters for plotting
  log_indices_for_plot = list(range(log_idx_offset + n_task))
  save_fig = True
  plot_nominal = True
  task_tolerance = 0.05  # 0.01  # if tasks are not on the grid points exactly

  # 2D plot
  task_slice_value = 0.2

  ### Set up environment
  # Create folder if not exist
  Path(eval_dir).mkdir(parents=True, exist_ok=True)

  ### Create model iter list
  if model_iter_idx_start == 1:
    model_iter_idx_start -= 1
  model_indices = list(
    range(model_iter_idx_start, model_iter_idx_end + 1, idx_spacing))
  if model_indices[0] == 0:
    model_indices[0] += 1
  # example list: [1, 5, 10, 15]
  print("model_indices = \n" + str(np.array(model_indices)))

  ### Create task list
  task_names = np.loadtxt(model_dir + "task_names.csv", dtype=str,
    delimiter=',')

  task_list = np.zeros((n_task, 4))
  task_list[:, 0] = stride_length
  task_list[:, 1] = ground_incline
  task_list[:, 2] = duration
  task_list[:, 3] = turning_rate
  print("task_list = \n" + str(task_list))

  # index of task vector where we sweep through
  varying_task_element_idx = GetVaryingTaskElementIdx(task_list)

  # Make sure the order is correct
  if not ((task_names[0] == "stride_length") &
          (task_names[1] == "ground_incline") &
          (task_names[2] == "duration") &
          (task_names[3] == "turning_rate")):
    raise ValueError("ERROR: unexpected task name or task order")

  ### Construct log indices
  log_indices = list(range(log_idx_offset, log_idx_offset + len(task_list)))
  print("log_indices = \n" + str(log_indices))

  ### Toggle the functions here to run simulation or evaluate cost
  # Simulation
  run_sim_and_eval_cost(model_indices, log_indices, task_list)

  # Cost evaluate only
  eval_cost_in_multithread(model_indices, log_indices)

  ### Plotting
  print("Nominal cost is from: " + model_dir)
  print("Simulation cost is from: " + eval_dir)

  # Manual overwrite log_indices for plotting
  if len(log_indices_for_plot) != 0:
    log_indices = log_indices_for_plot
  print("log_indices for plotting = " + str(log_indices))

  # Get all samples from trajopt for nominal cost
  sample_indices = CollectAllSampleIndicesFromTrajopt(task_list[0],
    task_tolerance) if plot_nominal else []
  print("sample_indices (trajopt) for nominal cost = \n" + str(sample_indices))

  # Plot
  plot_cost_vs_model_and_task(model_indices, log_indices, sample_indices, True)
  plot_cost_vs_model_and_task(model_indices, log_indices, sample_indices, False)

  plt.show()
