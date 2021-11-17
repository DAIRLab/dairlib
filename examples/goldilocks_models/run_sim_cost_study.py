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

import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import cm
import matplotlib.tri as mtri
import codecs
import math

from py_utils import FindVarValueInString

# Collection of all channel names
# Currently this class is useless becuase we use different lcm url for different simulation thread
class ChannelNames:
  def __init__(self, idx = -1):
    self.channel_x = "CASSIE_STATE_SIMULATION"
    self.channel_fsm_t = "FSM_T"
    self.channel_y = "MPC_OUTPUT"
    self.channel_u = "ROM_WALKING"
    if idx >= 0:
      self.channel_x += str(idx)
      self.channel_fsm_t += str(idx)
      self.channel_y += str(idx)
      self.channel_u += str(idx)


def BuildFiles(bazel_file_argument):
  build_cmd = ['bazel', 'build', bazel_file_argument, ]
  build_process = subprocess.Popen(build_cmd)
  while build_process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)


# cmd should be a list if shell=False. Otherwise, a string.
def RunCommand(cmd, use_shell=False):
  process = subprocess.Popen(cmd, shell=use_shell)
  while process.poll() is None:  # while subprocess is alive
    time.sleep(0.1)


def LogSimCostStudySetting():
  f = open(eval_dir + "sim_cost_study_log.txt", "a")
  f.write("\n\n*************************************************************\n")
  f.write("Current time : %s\n" % str(datetime.now()))
  f.write("model_dir = %s\n" % model_dir)
  f.write("spring_model = %s\n" % spring_model)
  f.write("target_realtime_rate = %s\n" % target_realtime_rate)
  f.write("foot_step_from_planner = %s\n" % foot_step_from_planner)

  commit_tag = subprocess.check_output(['git', 'rev-parse', 'HEAD'])
  git_diff = subprocess.check_output(['git', 'diff'])
  f.write("git commit hash: " + commit_tag.decode('ascii').strip() + "\n")
  f.write("\ngit diff:\n\n")
  f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])

  f.close()


def LcmlogFilePath(rom_iter_idx, log_idx, extra_layer=""):
  return eval_dir + extra_layer + 'lcmlog-idx_%d_%d' % (rom_iter_idx, log_idx)

def InitPoseSuccessPath(rom_iter_idx, log_idx):
  return eval_dir + 'sim_init_failed_%d_%d' % (rom_iter_idx, log_idx)

def InitPoseSolverFailed(path, enforce_existence = False):
  if os.path.exists(path):
    fail = int(np.loadtxt(path)) == 1
    return fail
  else:
    if enforce_existence:
      return True
    else:
      return False

# def get_nominal_task_given_sample_idx(trajopt_sample_idx, name):
#   # Get task element index by name
#   task_element_idx = np.where(task_names == name)[0][0]
#   task = np.loadtxt(model_dir + "%d_%d_task.csv" % (0, trajopt_sample_idx))
#   return task[task_element_idx]


# Set `get_init_file` to True if you want to generate the initial traj for both
# planner and controller
# `trajopt_sample_idx` is used for planner's initial guess and cost regularization term
def RunSimAndController(thread_idx, sim_end_time, task_value, log_idx, rom_iter_idx,
    trajopt_sample_idx, get_init_file):
  # Hacky heuristic parameter
  stride_length_scaling = 1.0
  # stride_length_scaling = 1 + min(rom_iter_idx / 30.0, 1) * 0.15

  # simulation arguments
  init_x_vel_reduction_ratio = 0.66  # since the starting configuration is standing pose, setting the task velocity would throw the MPC off. Cassie couldn't catch itself.
  pause_second = 2.0 if get_init_file else 0
  path_init_pose_success = InitPoseSuccessPath(rom_iter_idx, log_idx)

  # planner arguments
  realtime_rate_for_time_limit = target_realtime_rate
  dynamic_time_limit = True
  use_ipopt = False
  knots_per_mode = 5  # can try smaller number like 3 or 5
  feas_tol = 1e-2
  n_step = 2
  # time_limit is optional, set = 0 for realtime
  time_limit = 0.0 if dynamic_time_limit else 1.0 / target_realtime_rate * 0.2
  time_limit = 0.0 if get_init_file else time_limit
  planner_init_file = '' if get_init_file else '0_z.csv'
  data_dir_this_thread = data_dir[:-1] + str(thread_idx) + "/"

  # controller arguments
  init_traj_file_path = '' if get_init_file else data_dir_this_thread + '0_rom_trajectory'

  # other arguments
  port_idx = thread_idx + 1024  # Ports below 1024 are considered to be privileged in Linux. https://stackoverflow.com/questions/31899673/bind-returning-permission-denied-c
  planner_wait_identifier = eval_dir + "planner" + str(time.time())
  control_wait_identifier = eval_dir + "controller" + str(time.time())

  planner_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_planner_process',
    '--channel_x=%s' % ch.channel_x,
    '--channel_fsm_t=%s' % ch.channel_fsm_t,
    '--channel_y=%s' % ch.channel_y,
    '--lcm_url_port=%d' % port_idx,
    '--fix_duration=true',
    '--zero_touchdown_impact=true',
    '--log_solver_info=false',
    '--iter=%d' % rom_iter_idx,
    '--sample=%d' % trajopt_sample_idx,
    '--knots_per_mode=%d' % knots_per_mode,
    '--n_step=%d' % n_step,
    '--feas_tol=%.6f' % feas_tol,
    '--stride_length=%.3f' % task_value,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--time_limit=%.3f' % time_limit,
    '--realtime_rate_for_time_limit=%.3f' % realtime_rate_for_time_limit,
    '--init_file=%s' % planner_init_file,
    '--use_ipopt=%s' % ("true" if use_ipopt else str(get_init_file).lower()),
    '--log_data=%s' % str(get_init_file).lower(),
    '--run_one_loop_to_get_init_file=%s' % str(get_init_file).lower(),
    '--spring_model=%s' % str(spring_model).lower(),
    '--dir_data=%s' % data_dir_this_thread,
    '--path_wait_identifier=%s' % planner_wait_identifier,
    '--print_level=0',
  ]
  controller_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_controller',
    '--channel_x=%s' % ch.channel_x,
    '--channel_fsm_t=%s' % ch.channel_fsm_t,
    '--channel_y=%s' % ch.channel_y,
    '--channel_u=%s' % ch.channel_u,
    '--lcm_url_port=%d' % port_idx,
    '--evalulating_sim_cost=true',
    '--stride_length=%.3f' % task_value,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--iter=%d' % rom_iter_idx,
    '--init_traj_file_path=%s' % init_traj_file_path,
    '--spring_model=%s' % str(spring_model).lower(),
    '--get_swing_foot_from_planner=%s' % str(foot_step_from_planner).lower(),
    '--path_wait_identifier=%s' % control_wait_identifier,
  ]
  simulator_cmd = [
    'bazel-bin/examples/Cassie/multibody_sim_w_ground_incline',
    '--channel_x=%s' % ch.channel_x,
    '--channel_u=%s' % ch.channel_u,
    '--lcm_url_port=%d' % port_idx,
    '--end_time=%.3f' % sim_end_time,
    '--pause_second=%.3f' % pause_second,
    '--init_height=%.3f' % 1.0,
    '--pelvis_x_vel=%.3f' % ((init_x_vel_reduction_ratio * task_value / duration) if init_sim_vel else 0),
    '--target_realtime_rate=%.3f' % target_realtime_rate,
    '--spring_model=%s' % str(spring_model).lower(),
    '--path_init_pose_success=%s' % path_init_pose_success,
  ]
  lcm_logger_cmd = [
    'lcm-logger',
    '--lcm-url=udpm://239.255.76.67:%d' % port_idx,
    '-f',
    LcmlogFilePath(rom_iter_idx, log_idx),
  ]

  # Testing code to get command
  # if (rom_iter_idx == 100) and (log_idx == 37) and (not get_init_file):
  # if not get_init_file:
  #   print(' '.join(planner_cmd))
  #   print(' '.join(controller_cmd))
  #   print(' '.join(simulator_cmd))
  #   print(' '.join(lcm_logger_cmd))
  #   input("type anything to continue")
  # else:
  #   return

  path = eval_dir + "%d_%d_commands.txt" % (rom_iter_idx, log_idx)
  f = open(path, "a")
  f.write(' '.join(planner_cmd) + "\n")
  f.write(' '.join(controller_cmd) + "\n")
  f.write(' '.join(simulator_cmd) + "\n")
  f.write("---\n")
  f.close()

  # Remove file for init pose success/fail flag
  if os.path.exists(path_init_pose_success):
    RunCommand("rm " + path_init_pose_success, True)

  # Run all processes
  planner_process = subprocess.Popen(planner_cmd)
  controller_process = subprocess.Popen(controller_cmd)
  if not get_init_file:
    logger_process = subprocess.Popen(lcm_logger_cmd)
  # We don't run simulation thread until both planner and controller are waiting for simulation's message
  while not os.path.exists(planner_wait_identifier) or \
      not os.path.exists(control_wait_identifier):
    time.sleep(0.1)
  RunCommand("rm " + planner_wait_identifier, True)
  RunCommand("rm " + control_wait_identifier, True)
  simulator_process = subprocess.Popen(simulator_cmd)

  # Message to return
  msg = "iteration #%d log #%d: init pose solver failed to find a pose\n" % (rom_iter_idx, log_idx)

  if get_init_file:
    return ([planner_process, controller_process, simulator_process],
            [path_init_pose_success, msg],
            get_init_file,
            thread_idx)
  else:
    return ([simulator_process, planner_process, controller_process, logger_process],
            [path_init_pose_success, msg],
            get_init_file,
            thread_idx)


# sim_end_time is used to check if the simulation ended early
def EvalCost(sim_end_time, rom_iter_idx, log_idx, multithread=False):
  eval_cost_cmd = [
    'bazel-bin/examples/goldilocks_models/eval_single_sim_performance',
    LcmlogFilePath(rom_iter_idx, log_idx),
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


# trajopt_sample_indices for visualization
def CollectAllTrajoptSampleIndices(task, task_tolerance):
  # We pick iter 0 since we assume we are using grid tasks (index will be the same across different model iteration)
  # TODO: change this if not using grid task anymore
  rom_iter = 0

  n_sample_trajopt = int(np.loadtxt(model_dir + "n_sample.csv"))
  trajopt_sample_indices_for_viz = []
  for j in range(n_sample_trajopt):
    path = model_dir + "%d_%d_task.csv" % (rom_iter, j)
    if os.path.exists(path):
      trajopt_task = np.loadtxt(path)
      task_diff = trajopt_task - task
      task_diff[varying_task_element_idx] = 0
      if np.linalg.norm(task_diff) < task_tolerance:
        trajopt_sample_indices_for_viz.append(j)

  if len(trajopt_sample_indices_for_viz) == 0:
    print("ERROR: no such task in trajopt. Will not plot nominal cost")

  return trajopt_sample_indices_for_viz


# trajopt_sample_indices for planner (find the most similar tasks)
def ConstructTrajoptSampleIndicesGivenModelAndTask(model_indices, task_list):
  trajopt_sample_indices = np.zeros((len(model_indices), len(task_list)),
    dtype=np.dtype(int))
  for i in range(len(model_indices)):
    for j in range(len(task_list)):
      trajopt_sample_indices[i, j] = GetTrajoptSampleIndexGivenTask(model_indices[i],
        task_list[j])
  return trajopt_sample_indices


# Get trajopt sample idx with the most similar task
def GetTrajoptSampleIndexGivenTask(rom_iter, task):
  n_sample_trajopt = int(np.loadtxt(model_dir + "n_sample.csv"))
  dist_list = []
  for j in range(n_sample_trajopt):
    path = model_dir + "%d_%d_task.csv" % (rom_iter, j)
    # print("try " + path)
    if os.path.exists(path):
      trajopt_task = np.loadtxt(path)
      dist_list.append(np.linalg.norm(trajopt_task - task))
  # print("dist_list = ")
  # print(dist_list)
  if len(dist_list) == 0:
    raise ValueError("ERROR: This path doesn't exist: " + path)
  trajopt_sample_idx = np.argmin(np.array(dist_list))

  return trajopt_sample_idx


def SaveLogCorrespondence():
  msg = "log #%d to #%d: %s ranges from %.3f to %.3f\n" % (
    log_indices[0], log_indices[-1], task_names[varying_task_element_idx],
    task_list[0, varying_task_element_idx],
    task_list[-1, varying_task_element_idx])
  print(msg)
  f = open(eval_dir + "task_log_correspondence.txt", "a")
  f.write(msg)
  f.close()


def EndSim(working_threads, idx, recycle_idx=True):
  # Once we reach the code here, it means one simulation has ended

  # Log sim status
  if InitPoseSolverFailed(working_threads[idx][1][0], True):
    print(working_threads[idx][1][1])
    f = open(eval_dir + "sim_status.txt", "a")
    f.write(working_threads[idx][1][1])
    f.close()

  # Kill the rest of processes (necessary)
  for i in range(1, len(working_threads[idx][0])):
    working_threads[idx][0][i].kill()

  # add back available thread idx
  if recycle_idx:
    thread_idx_set.add(working_threads[idx][3])

  del working_threads[idx]


def BlockAndDeleteTheLatestThread(working_threads):
  # It is always the last one in the list because we just appended it.
  while working_threads[-1][0][0].poll() is None and \
      not InitPoseSolverFailed(working_threads[-1][1][0]):
    time.sleep(0.1)
  EndSim(working_threads, -1, False)


def CheckSimThreadAndBlockWhenNecessary(working_threads, n_max_thread,
                                        finish_up=False):
  # Wait for threads to finish once is more than n_max_thread
  while (not finish_up and (len(working_threads) >= n_max_thread)) or \
      (finish_up and (len(working_threads) > 0)):
    for j in range(len(working_threads)):
      if working_threads[j][0][0].poll() is None:
        time.sleep(0.1)
      else:
        EndSim(working_threads, j)
        break


def RunSimAndEvalCostInMultithread(model_indices, log_indices, task_list,
    do_eval_cost=False):
  # TODO: I wonder if we will break any thing if we run ipopt in parallel for the initial pose? (currently we are not doing this, so we are fine)

  ### Channel names
  global ch
  ch = ChannelNames()

  ### Logging
  LogSimCostStudySetting()
  SaveLogCorrespondence()

  ### Build files just in case forgetting
  BuildFiles('examples/goldilocks_models/...')
  BuildFiles('examples/Cassie:multibody_sim_w_ground_incline')

  ### Construct sample indices from the task list for simulation
  # `trajopt_sample_idx` is for planner's initial guess and cost regularization term
  trajopt_sample_indices = ConstructTrajoptSampleIndicesGivenModelAndTask(model_indices,
    task_list)
  print("trajopt_sample_indices = \n" + str(trajopt_sample_indices))

  ### multithreading
  working_threads = []
  n_max_thread = 3

  global thread_idx_set
  thread_idx_set = set()
  for i in range(n_max_thread):
    thread_idx_set.add(i)

  ### Start simulation
  n_total_sim = len(model_indices) * len(task_list)
  counter = -1
  for i in range(len(model_indices)):
    for j in range(len(task_list)):
      counter += 1

      rom_iter = model_indices[i]
      task = task_list[j]
      trajopt_sample_idx = trajopt_sample_indices[i][j]
      log_idx = log_indices[j]

      print("\n===========\n")
      print("progress %.1f%%" % (float(counter) / n_total_sim * 100))
      print("run sim for model %d and task %.3f" % \
            (rom_iter, task[varying_task_element_idx]))

      path = eval_dir + '%d_%d_success.csv' % (rom_iter, log_idx)
      if not os.path.exists(path):
        # Get the initial traj
        thread_idx = thread_idx_set.pop()
        working_threads.append(
            RunSimAndController(thread_idx, sim_end_time,
                                task[varying_task_element_idx], log_idx,
                                rom_iter, trajopt_sample_idx, True))
        BlockAndDeleteTheLatestThread(working_threads)

        # Run the simulation
        working_threads.append(
            RunSimAndController(thread_idx, sim_end_time,
                                task[varying_task_element_idx], log_idx,
                                rom_iter, trajopt_sample_idx, False))
        CheckSimThreadAndBlockWhenNecessary(working_threads, n_max_thread)

        # Evaluate the cost (not multithreaded. Will block the main thread)
        if do_eval_cost:
          EvalCost(sim_end_time, rom_iter, log_idx)

        # Delete the lcmlog
        # os.remove(LcmlogFilePath(rom_iter_idx, log_idx))

  CheckSimThreadAndBlockWhenNecessary(working_threads, n_max_thread, True)

  print("Finished evaluating. Current time = " + str(datetime.now()))


def CheckThreadAndBlockWhenNecessary(working_threads, n_max_thread,
    finish_up=False):
  # Wait for threads to finish once is more than n_max_thread
  while (not finish_up and (len(working_threads) >= n_max_thread)) or \
      (finish_up and (len(working_threads) > 0)):
    for j in range(len(working_threads)):
      if working_threads[j].poll() is None:  # subprocess is alive
        time.sleep(0.1)
      else:
        del working_threads[j]
        break


# This function assumes that simulation has been run and there exist lcm logs
def EvalCostInMultithread(model_indices, log_indices):
  working_threads = []
  n_max_thread = 12

  ### Build files just in case forgetting
  BuildFiles('examples/goldilocks_models:eval_single_sim_performance')

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
        working_threads.append(EvalCost(sim_end_time, rom_iter, idx, True))
      counter += 1

      # Wait for threads to finish once is more than n_max_thread
      CheckThreadAndBlockWhenNecessary(working_threads, n_max_thread)

  print("Wait for all threads to join")
  CheckThreadAndBlockWhenNecessary(working_threads, n_max_thread, True)
  print("Finished evaluating. Current time = " + str(datetime.now()))


def DeleteMostLogs(model_indices, log_indices):
  if log_indices[0] != 0:
    raise ValueError("log index should start from 0")

  model_len = len(model_indices)
  if model_indices[0] == 1:
    sampled_model_indices = list(
      {model_indices[0], model_indices[int(model_len / 2)], model_indices[-1]})
  else:
    sampled_model_indices = list(
      {model_indices[int(model_len / 2) - 1], model_indices[-1]})
  sampled_model_indices.sort()
  print("sampled_model_indices = ", sampled_model_indices)

  # Create a temp folder
  Path(eval_dir + "temp").mkdir(parents=True, exist_ok=True)

  file_saving_command_list = []
  for model_idx in sampled_model_indices:
    # Get log indices to save
    # parameter
    mid_idx1_target = len(log_indices) / 2
    mid_idx2_target = 50

    min_success_log_idx = max(log_indices)
    max_success_log_idx = 0
    mid_success_log_idx1 = 0
    mid_success_log_idx2 = 0
    for log_idx in log_indices:
      path = eval_dir + '%d_%d_success.csv' % (model_idx, log_idx)
      if os.path.exists(path):
        if min_success_log_idx > log_idx:
          min_success_log_idx = log_idx
        if max_success_log_idx < log_idx:
          max_success_log_idx = log_idx
        if ((mid_success_log_idx1 < log_idx) and (
            log_idx < mid_idx1_target)) or mid_success_log_idx1 == 0:
          mid_success_log_idx1 = log_idx
        if ((mid_success_log_idx2 < log_idx) and (
            log_idx < mid_idx2_target)) or mid_success_log_idx2 == 0:
          mid_success_log_idx2 = log_idx

    if min_success_log_idx > max_success_log_idx:
      raise ValueError("No successful simulation for model idx %d" % model_idx)

    print("model idx %d: min, mid1, mid2, max log idx = %d, %d, %d, %d)" % (
      model_idx, min_success_log_idx, mid_success_log_idx1,
      mid_success_log_idx2, max_success_log_idx))

    # Save log indices
    file_saving_command_list.append(
      ['cp', LcmlogFilePath(model_idx, min_success_log_idx),
       LcmlogFilePath(model_idx, min_success_log_idx, "temp/")])
    file_saving_command_list.append(
      ['cp', LcmlogFilePath(model_idx, max_success_log_idx),
       LcmlogFilePath(model_idx, max_success_log_idx, "temp/")])
    file_saving_command_list.append(
      ['cp', LcmlogFilePath(model_idx, mid_success_log_idx1),
       LcmlogFilePath(model_idx, mid_success_log_idx1, "temp/")])
    file_saving_command_list.append(
      ['cp', LcmlogFilePath(model_idx, mid_success_log_idx2),
       LcmlogFilePath(model_idx, mid_success_log_idx2, "temp/")])

  # Check if all lcmlogs exist
  for command in file_saving_command_list:
    if not os.path.exists(command[1]):
      raise ValueError(
        "%s doesn't exist. `model_indices` was probably set incorrectly" %
        command[1])

  # Save logs
  for command in file_saving_command_list:
    RunCommand(command)

  input("WARNING: Going to delete lcmlog files! (type anything to continue)")

  # Delete the rest of the file
  RunCommand('rm ' + eval_dir + 'lcmlog-idx_*', True)
  # Copy back the successful files
  RunCommand('cp ' + eval_dir + 'temp/lcmlog-idx_* ' + eval_dir, True)
  # Delete temp folder
  RunCommand(['rm', '-rf', eval_dir + 'temp/'])


def PlotNominalCost(model_indices, trajopt_sample_indices_for_viz):
  filename = '_' + str(trajopt_sample_indices_for_viz) + '_trajopt_settings_and_cost_breakdown.txt'

  costs = np.zeros((0, 1))
  for rom_iter_idx in model_indices:
    with open(model_dir + str(rom_iter_idx) + filename, 'rt') as f:
      contents = f.read()
    cost_x = FindVarValueInString(contents, "cost_x =")
    cost_u = FindVarValueInString(contents, "cost_u =")
    cost_accel = FindVarValueInString(contents, "cost_joint_acceleration =")
    total_cost = cost_x + cost_u + cost_accel
    costs = np.vstack([costs, total_cost])

  # figname = "Nominal cost over model iterations"
  # plt.figure(figname, figsize=(6.4, 4.8))
  # plt.plot(model_indices, costs)
  # plt.ylabel('cost')
  # plt.xlabel('model iterations')
  # plt.legend(["total_cost"])
  # plt.show()
  return costs


def GetSamplesToPlot(model_indices, log_indices):
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
        # if (cost[-1] > 2.25) & (task < 0.3):
        #   continue
        # print('Add (iter,idx) = (%d,%d)' % (rom_iter, idx))
        mtcl = np.vstack([mtcl, current_mtcl])
        ### For debugging
        # if (cost[-1] > 2.25) & (task < 0.3):
        #   print("(iter, log) = (%.0f, %.0f) has cost %.3f (outlier)" %
        #         (current_mtcl[0, 0], current_mtcl[0, 3], current_mtcl[0, 2]))
  print("mtcl.shape = " + str(mtcl.shape))

  ### Testing -- find the log idx with high cost
  cost_threshold = 3
  for mem in mtcl:
    if mem[2] > cost_threshold:
      print("(iter, log) = (%.0f, %.0f) has high cost %.3f" %
            (mem[0], mem[3], mem[2]))

  return mtcl


def GetNominalSamplesToPlot(model_indices):
  ### Get all samples from trajopt for nominal cost
  trajopt_sample_indices_for_viz = CollectAllTrajoptSampleIndices(task_list[0],
    task_tolerance) if plot_nominal else []
  print("sample_indices (trajopt) for nominal cost visualization = \n" + str(trajopt_sample_indices_for_viz))

  ### Get the samples to plot
  # nominal_mtc stores model index, task value, and cost from trajopt
  nominal_mtc = np.zeros((0, 3))
  for rom_iter in model_indices:
    for i in range(len(trajopt_sample_indices_for_viz)):
      sub_mtc = np.zeros((1, 3))
      ### Read cost
      cost = PlotNominalCost([rom_iter], trajopt_sample_indices_for_viz[i])[0][0]
      sub_mtc[0, 2] = cost
      if cost.item() > max_cost_to_ignore:
        continue
      ### Read nominal task
      path = model_dir + "%d_%d_task.csv" % (rom_iter, trajopt_sample_indices_for_viz[i])
      if os.path.exists(path):
        task = np.loadtxt(path)[varying_task_element_idx]
        sub_mtc[0, 1] = task
      else:
        continue
      if (task > max_sl) or (task < min_sl):
        continue
      ### Read model iteration
      sub_mtc[0, 0] = rom_iter
      ### Assign values
      nominal_mtc = np.vstack([nominal_mtc, sub_mtc])
  print("nominal_mtc.shape = " + str(nominal_mtc.shape))

  return nominal_mtc

def Generate3dPlots(model_indices, mtcl, nominal_mtc):
  app = "_w_nom" if plot_nominal else ""
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


def Generate2dPlots(model_indices, mtcl, nominal_mtc):
  app = "_w_nom" if plot_nominal else ""
  ### 2D plot (cost vs iteration)
  # The line along which we evaluate the cost (using interpolation)
  n_model_iter = model_indices[-1] - model_indices[0]
  x = np.linspace(0, n_model_iter, n_model_iter + 1)

  plt.figure(figsize=(6.4, 4.8))
  plt.rcParams.update({'font.size': 14})
  triang = mtri.Triangulation(mtcl[:, 0], mtcl[:, 1])
  interpolator = mtri.LinearTriInterpolator(triang, mtcl[:, 2])

  for task_slice_value in task_slice_value_list:
    y = task_slice_value * np.ones(n_model_iter + 1)
    z = interpolator(x, y)
    # plt.plot(x, z, linewidth=3, label='stride length ' + str(task_slice_value) + " m (Drake sim)")
    plt.plot(x, z, linewidth=3, label='stride length ' + str(task_slice_value) + " m")
    # plt.plot(x, z, 'k-', linewidth=3, label="Drake simulation")

  if plot_nominal:
    plt.gca().set_prop_cycle(None)  # reset color cycle
    triang = mtri.Triangulation(nominal_mtc[:, 0], nominal_mtc[:, 1])
    interpolator = mtri.LinearTriInterpolator(triang, nominal_mtc[:, 2])
    for task_slice_value in task_slice_value_list:
      y = task_slice_value * np.ones(n_model_iter + 1)
      z = interpolator(x, y)
      plt.plot(x, z, '--', linewidth=3, label='stride length ' + str(task_slice_value) + " m (traj opt)")
      # plt.plot(x, z, '--', linewidth=3, label='stride length ' + str(task_slice_value) + " m")
      # plt.plot(x, z, 'k--', linewidth=3, label="trajectory optimization")

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
    plt.savefig("%scost_vs_model_iter%s.png" % (eval_dir, app))

  ### 2D plot (cost vs tasks)
  plt.figure(figsize=(6.4, 4.8))
  plt.rcParams.update({'font.size': 14})
  for i in range(len(model_slices)):
    model_iter = model_slices[i]
    # The line along which we evaluate the cost (using interpolation)
    x = model_iter * np.ones(500)
    y = np.linspace(-0.8, 0.8, 500)

    triang = mtri.Triangulation(mtcl[:, 0], mtcl[:, 1])
    interpolator = mtri.LinearTriInterpolator(triang, mtcl[:, 2])
    z = interpolator(x, y)
    plt.plot(y, z, '-',  # color=color_names[i],
             linewidth=3, label="iter " + str(model_iter))
    # if plot_nominal:
    #   triang = mtri.Triangulation(nominal_mtc[:, 0], nominal_mtc[:, 1])
    #   interpolator = mtri.LinearTriInterpolator(triang, nominal_mtc[:, 2])
    #   z = interpolator(x, y)
    #   plt.plot(x, z, 'k--', linewidth=3, label="trajectory optimization")

  plt.xlabel('stride length (m)')
  plt.ylabel('total cost')
  plt.legend()
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%scost_vs_task.png" % eval_dir)

  ### 2D plot (iter vs tasks)
  data_list = [mtcl, nominal_mtc]
  title_list = ["(Drake sim)", "(traj opt)"]
  app_list = ["", "_nom"]
  for i in range(2 if plot_nominal else 1):
    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()

    data = data_list[i]
    n_levels = 50
    levels = list(set(
      np.linspace(min(data[:, 2]), max(data[:, 2]), n_levels).round(
        decimals=2)))  # set() is used to get rid of duplicates
    levels.sort()
    levels[0] -= 0.01
    levels[-1] += 0.01
    # levels = list(set(np.linspace(0.4, 3, n_levels)))
    # levels.sort()
    surf = ax.tricontourf(data[:, 0], data[:, 1], data[:, 2], levels=levels, cmap='coolwarm')
    fig.colorbar(surf, shrink=0.9, aspect=15)

    # plt.xlim([0, 135])
    plt.xlabel('model iterations')
    plt.ylabel('stride length (m)')
    plt.title('Cost landscape ' + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape_iter%s.png" % (eval_dir, app_list[i]))


def ComputeExpectedCostOverTask(mtcl, stride_length_range_to_average):
  if len(stride_length_range_to_average) == 0:
    return
  elif len(stride_length_range_to_average) != 2:
    raise ValueError("the range list has to be 2 dimensional")
  elif stride_length_range_to_average[0] > stride_length_range_to_average[1]:
    raise ValueError("first element should be the lower bound of the range")

  # Correct the range so that it's within the achieveable task space for all model iter
  viable_min = -math.inf
  viable_max = math.inf
  effective_length = len(model_indices)
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      viable_min = max(viable_min, min(mtcl[mtcl[:, 0] == model_iter, 1]))
      viable_max = min(viable_max, max(mtcl[mtcl[:, 0] == model_iter, 1]))
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
    model_iter = model_indices[i]
    # The line along which we evaluate the cost (using interpolation)
    x = model_iter * np.ones(n_sample)
    y = np.linspace(stride_length_range_to_average[0], stride_length_range_to_average[1], n_sample)

    triang = mtri.Triangulation(mtcl[:, 0], mtcl[:, 1])
    interpolator = mtri.LinearTriInterpolator(triang, mtcl[:, 2])
    z = interpolator(x, y)

    # Make sure the averaged range is within the achievable task space
    # Method 1
    # if min(mtcl[mtcl[:, 0] == model_iter, 1]) > stride_length_range_to_average[0]:
    #   raise ValueError("iter %d: the range we average over is bigger than the achievable task space. Increase the range's lower bound" % model_iter)
    # elif max(mtcl[mtcl[:, 0] == model_iter, 1]) < stride_length_range_to_average[1]:
    #   raise ValueError("the range we average over is bigger than the achievable task space. Decrease the range's upper bound" % model_iter)
    # Method 2
    if z.mask.sum() > 0:
      viable_min = min(mtcl[mtcl[:, 0] == model_iter, 1])
      viable_max = max(mtcl[mtcl[:, 0] == model_iter, 1])
      raise ValueError("iter %d: the range we average over is larger than the achievable task space. "
                       "We should either increase the range's lower bound or decrease the range's upper bound. "
                       "Viable (min, max) is (%f, %f)" % (model_iter, viable_min, viable_max))

    averaged_cost[i] = z.sum() / n_sample

  plt.figure(figsize=(6.4, 4.8))
  plt.plot(model_indices[:effective_length], averaged_cost, 'k-', linewidth=3)
  plt.xlabel('model iteration')
  plt.ylabel('averaged cost')
  plt.title("Cost averaged over stride length [" + str(stride_length_range_to_average[0]) + ", " + str(stride_length_range_to_average[1]) + "] m")
  plt.gcf().subplots_adjust(bottom=0.15)
  plt.gcf().subplots_adjust(left=0.15)
  if save_fig:
    plt.savefig("%saveraged_cost_vs_model_iter.png" % eval_dir)


def ComputeAchievableTaskRangeOverIter(mtcl):
  ### 2D plot (task range vs iteration)
  task_range = np.zeros(len(model_indices))
  for i in range(len(model_indices)):
    model_iter = model_indices[i]
    try:
      tasks = mtcl[mtcl[:, 0] == model_iter, 1]
      task_range[i] = max(tasks) - min(tasks)
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
    plt.savefig("%stask_space_vs_model_iter.png" % eval_dir)


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
  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  model_dir = parsed_yaml_file.get('dir_model')
  data_dir = parsed_yaml_file.get('dir_data')

  eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval/"
  # eval_dir = "/home/yuming/Desktop/temp/"
  # eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval_2/"
  # eval_dir = "/home/yuming/Desktop/temp/3/sim_cost_eval_20210507/sim_cost_eval/"

  ### global parameters
  sim_end_time = 10.0
  spring_model = False
  # Parameters that are modified often
  target_realtime_rate = 0.1  # 0.04
  foot_step_from_planner = True
  init_sim_vel = True

  ### parameters for model, task, and log indices
  # Model iteration list
  model_iter_idx_start = 1  # 0
  model_iter_idx_end = 100
  idx_spacing = 5

  # Task list
  n_task = 30
  stride_length = np.linspace(-0.6, 0.6, n_task)
  # stride_length = np.linspace(0, 0.1, n_task)
  # stride_length = np.linspace(-0.2, -0.1, n_task)
  # stride_length = np.linspace(-0.3, 0, n_task, endpoint=False)
  # stride_length = np.linspace(0.4, 0.5, n_task)
  # stride_length = np.linspace(0, 0, n_task)
  # stride_length = np.hstack([np.linspace(-0.6, -0.4, n_task, endpoint=False),
  #                            -np.linspace(-0.6, -0.4, n_task, endpoint=False)])
  ground_incline = 0.0
  duration = -1.0  # assign later; this shouldn't be a task for sim evaluation
  turning_rate = 0.0
  pelvis_height = 0.8  # not used in simulation; only used in CollectAllTrajoptSampleIndices

  # log indices
  log_idx_offset = 0  # 0

  ### Parameters for plotting
  log_indices_for_plot = list(range(log_idx_offset + n_task))
  # log_indices_for_plot = list(range(240))
  save_fig = True
  plot_nominal = True
  task_tolerance = 0.05  # 0.01  # if tasks are not on the grid points exactly

  # 2D plot (cost vs model)
  task_slice_value_list = [-0.16, 0, 0.16]
  # task_slice_value_list = [-0.2, -0.1, 0, 0.1, 0.2]
  # task_slice_value_list = [-0.4, -0.2, 0, 0.2, 0.4]

  # 2D plot (cost vs task)
  model_slices = [1, 50, 100, 150]
  # model_slices = list(range(1, 55, 5))
  # color_names = ["darkblue", "maroon"]
  # color_names = ["k", "maroon"]

  # Expected (averaged) cost over a task range
  stride_length_range_to_average = [-0.2, 0.1897]

  ### Set up environment
  # Create folder if not exist
  if len(sys.argv) > 1 and sys.argv[1] == "fresh":
    input("WARNING: Going to delete lcmlog files! (type anything to continue)")
    os.system("rm -rf " + eval_dir)
  Path(eval_dir).mkdir(parents=True, exist_ok=True)
  Path(data_dir).mkdir(parents=True, exist_ok=True)  # for MPC's init file

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
  # Make sure the order is correct
  if not ((task_names[0] == "stride_length") &
          (task_names[1] == "ground_incline") &
          (task_names[2] == "duration") &
          (task_names[3] == "turning_rate")):
    raise ValueError("ERROR: unexpected task name or task order")
  # Get duration from model optimization file
  path_1_0_task = model_dir + "1_0_task.csv"
  if os.path.exists(path_1_0_task):
    duration = np.loadtxt(path_1_0_task)[2]
  else:
    raise ValueError("%s doesn't exist" % path_1_0_task)

  task_list = np.zeros((n_task, 5))
  task_list[:, 0] = stride_length
  task_list[:, 1] = ground_incline
  task_list[:, 2] = duration
  task_list[:, 3] = turning_rate
  task_list[:, 4] = pelvis_height
  print("task_list = \n" + str(task_list))

  # index of task vector where we sweep through
  varying_task_element_idx = GetVaryingTaskElementIdx(task_list)
  if varying_task_element_idx != 0:
    raise ValueError("Currently, the code assume only stride length is varying")


  # Some other checks
  # duration in sim doesn't have to be the same as trajopt's, but I added a check here as a reminder.
  if not math.isclose(
      parsed_yaml_file.get('left_support_duration') + parsed_yaml_file.get(
        'double_support_duration'), duration):
    # raise ValueError("Reminder: you are setting a different duration in sim than in trajopt")
    print("Warning: duration in sim is different from in trajopt")
    input("type anything to confirm and continue")

  ### Construct log indices
  log_indices = list(range(log_idx_offset, log_idx_offset + len(task_list)))
  print("log_indices = \n" + str(log_indices))

  ### Toggle the functions here to run simulation or evaluate cost
  # Simulation
  #RunSimAndEvalCostInMultithread(model_indices, log_indices, task_list)

  # Cost evaluate only
  #EvalCostInMultithread(model_indices, log_indices)

  # Delete all logs but a few successful ones (for analysis later)
  #DeleteMostLogs(model_indices, log_indices)

  ### Plotting
  print("Nominal cost is from: " + model_dir)
  print("Simulation cost is from: " + eval_dir)

  # Parameters for visualization
  max_cost_to_ignore = 100  # 2
  # mean_sl = 0.2
  # delta_sl = 0.1  # 0.1 #0.005
  # min_sl = mean_sl - delta_sl
  # max_sl = mean_sl + delta_sl
  min_sl = -100
  max_sl = 100

  # Manual overwrite log_indices for plotting
  if len(log_indices_for_plot) != 0:
    log_indices = log_indices_for_plot
  print("log_indices for plotting = " + str(log_indices))

  # Get samples to plot
  # mtcl is a list of (model index, task value, cost, and log index)
  mtcl = GetSamplesToPlot(model_indices, log_indices)
  nominal_mtc = GetNominalSamplesToPlot(model_indices)
  if len(nominal_mtc) == 0:
    plot_nominal = False

  # Plot
  Generate3dPlots(model_indices, mtcl, nominal_mtc)
  Generate2dPlots(model_indices, mtcl, nominal_mtc)

  ### Compute expected (averaged) cost
  ComputeExpectedCostOverTask(mtcl, stride_length_range_to_average)

  ### Compute task range over iteration
  ComputeAchievableTaskRangeOverIter(mtcl)

  plt.show()
