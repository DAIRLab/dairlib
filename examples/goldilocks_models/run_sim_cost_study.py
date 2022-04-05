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
from scipy.interpolate import LinearNDInterpolator
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

def EnforceSlashEnding(dir):
  if len(dir) > 0 and dir[-1] != "/":
    raise ValueError("Directory path name should end with slash")

def LogSimCostStudySetting():
  f = open(eval_dir + "sim_cost_study_log.txt", "a")
  f.write("\n\n*************************************************************\n")
  f.write("Current time : %s\n" % str(datetime.now()))
  f.write("model_dir = %s\n" % model_dir)
  f.write("spring_model = %s\n" % spring_model)
  f.write("target_realtime_rate = %s\n" % target_realtime_rate)
  f.write("foot_step_from_planner = %s\n" % foot_step_from_planner)
  f.write("stance_hip_angles_from_planner = %s\n" % stance_hip_angles_from_planner)
  f.write("init_sim_vel = %s\n" % init_sim_vel)
  f.write("use_nominal_traj_pool = %s\n" % use_nominal_traj_pool)
  f.write("set_sim_init_state_from_trajopt = %s\n" % set_sim_init_state_from_trajopt)
  f.write("completely_use_trajs_from_model_opt_as_target = %s\n" % completely_use_trajs_from_model_opt_as_target)

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
def RunSimAndController(thread_idx, sim_end_time, task, log_idx, rom_iter_idx,
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

  # Extract tasks
  task_sl = task[tasks.GetDimIdxByName("stride_length")]

  dir_and_prefix_FOM_reg = "" if len(FOM_model_dir) == 0 else "%s0_%d_" % (FOM_model_dir, trajopt_sample_idx)
  path_init_state = "%s%d_%d_x_samples0.csv" % (model_dir, rom_iter_idx, trajopt_sample_idx) if set_sim_init_state_from_trajopt else ""

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
    '--stride_length=%.3f' % task_sl,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--time_limit=%.3f' % time_limit,
    '--realtime_rate_for_time_limit=%.3f' % realtime_rate_for_time_limit,
    '--init_file=%s' % planner_init_file,
    '--use_ipopt=%s' % ("true" if use_ipopt else str(get_init_file).lower()),
    '--log_data=%s' % str(get_init_file).lower(),
    '--run_one_loop_to_get_init_file=%s' % str(get_init_file).lower(),
    '--spring_model=%s' % str(spring_model).lower(),
    '--dir_and_prefix_FOM=%s' % dir_and_prefix_FOM_reg,
    '--dir_data=%s' % data_dir_this_thread,
    '--path_wait_identifier=%s' % planner_wait_identifier,
    '--print_level=0',
    '--completely_use_trajs_from_model_opt_as_target=%s' % str(completely_use_trajs_from_model_opt_as_target).lower(),
    ]
  controller_cmd = [
    'bazel-bin/examples/goldilocks_models/run_cassie_rom_controller',
    '--channel_x=%s' % ch.channel_x,
    '--channel_fsm_t=%s' % ch.channel_fsm_t,
    '--channel_y=%s' % ch.channel_y,
    '--channel_u=%s' % ch.channel_u,
    '--lcm_url_port=%d' % port_idx,
    '--close_sim_gap=true',
    '--stride_length=%.3f' % task_sl,
    '--stride_length_scaling=%.3f' % stride_length_scaling,
    '--iter=%d' % rom_iter_idx,
    '--init_traj_file_path=%s' % init_traj_file_path,
    '--spring_model=%s' % str(spring_model).lower(),
    '--get_swing_foot_from_planner=%s' % str(foot_step_from_planner).lower(),
    '--get_stance_hip_angles_from_planner=%s' % str(stance_hip_angles_from_planner).lower(),
    '--get_swing_hip_angle_from_planner=%s' % str(swing_hip_angle_from_planner).lower(),
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
    '--pelvis_x_vel=%.3f' % ((init_x_vel_reduction_ratio * task_sl / duration) if init_sim_vel else 0),
    '--target_realtime_rate=%.3f' % target_realtime_rate,
    '--spring_model=%s' % str(spring_model).lower(),
    '--path_init_state=%s' % path_init_state,
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
def CollectAllTrajoptSampleIndices():
  # We pick iter 0 since we assume we are using grid tasks (index will be the same across different model iteration)
  # TODO: change this if not using grid task anymore
  rom_iter = 0

  # Collect all samples
  n_sample_trajopt = int(np.loadtxt(model_dir + "n_sample.csv"))
  trajopt_sample_indices_for_viz = []

  for j in range(n_sample_trajopt):
    path = model_dir + "%d_%d_task.csv" % (rom_iter, j)
    if os.path.exists(path):
      # trajopt_task = np.loadtxt(path)
      trajopt_sample_indices_for_viz.append(j)

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
  dir = model_dir if len(FOM_model_dir) == 0 else FOM_model_dir

  n_sample_trajopt = int(np.loadtxt(dir + "n_sample.csv"))
  dist_list = []
  path = ""
  for j in range(n_sample_trajopt):
    path = dir + "%d_%d_task.csv" % (rom_iter, j)
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
  msg = "log #%d to #%d: \n" % (log_indices[0], log_indices[-1])
  msg += tasks.tasks_info()
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
  for i in range(0, len(working_threads[idx][0])):
    working_threads[idx][0][i].kill()

  # Add back available thread idx.
  # We don't add back avaible thread idx when the sim is used to initialize the
  # MPC first solution, because we need the same thread index (used in folder
  # name) to store the files
  if recycle_idx:
    thread_idx_set.add(working_threads[idx][3])

  del working_threads[idx]


def BlockAndDeleteTheLatestThread(working_threads):
  # It is always the last one in the list because we just appended it.
  # Question (20220329): why did I have a condition `not InitPoseSolverFailed` (see below)? I found today this was a bug to me, because it ended the process too early and no init_file was created for the planner.
    # while working_threads[-1][0][0].poll() is None and \
    #     not InitPoseSolverFailed(working_threads[-1][1][0]):
  while working_threads[-1][0][0].poll() is None:
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
        # I paused a second in sim script to give lcm-logger time to finish
        # logging, so I don't need to pause here again. That is, no need to run
        # `time.sleep(1)` here.
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
  n_max_thread = min(int(psutil.cpu_count() / 3) - 1, len(task_list))  # TODO: check if the min is necessary
  n_max_thread = min(int(psutil.cpu_count() / 2), len(task_list)) if target_realtime_rate == 0.1 else n_max_thread

  global thread_idx_set
  thread_idx_set = set()
  for i in range(n_max_thread):
    thread_idx_set.add(i)

  ### Start simulation
  n_total_sim = len(model_indices) * len(task_list)
  counter = -1
  # skip_this_iter = True
  for i in range(len(model_indices)):
    for j in range(len(task_list)):
      counter += 1

      # # Testing
      # if i == 1 and j == 45:  #42
      #   skip_this_iter = False
      # if skip_this_iter:
      #   continue

      rom_iter = model_indices[i]
      task = task_list[j]
      trajopt_sample_idx = trajopt_sample_indices[i][j]
      log_idx = log_indices[j]

      print("\n===========\n")
      print("progress %.1f%%" % (float(counter) / n_total_sim * 100))
      print("run sim for model %d and task %d" % (rom_iter, j))

      path = eval_dir + '%d_%d_success.csv' % (rom_iter, log_idx)
      if not os.path.exists(path):
        # Get the initial traj
        # print("1 thread_idx_set = " + str(thread_idx_set))
        # print("len(working_threads) = " + str(len(working_threads)))
        thread_idx = thread_idx_set.pop()
        working_threads.append(
          RunSimAndController(thread_idx, sim_end_time, task, log_idx,
                              rom_iter, trajopt_sample_idx, True))
        # print("2 thread_idx_set = " + str(thread_idx_set))
        # print("len(working_threads) = " + str(len(working_threads)))
        # print("BlockAndDeleteTheLatestThread")
        BlockAndDeleteTheLatestThread(working_threads)
        # print("BlockAndDeleteTheLatestThread")

        # Run the simulation
        # print("3 thread_idx_set = " + str(thread_idx_set))
        # print("len(working_threads) = " + str(len(working_threads)))
        working_threads.append(
          RunSimAndController(thread_idx, sim_end_time, task, log_idx,
                              rom_iter, trajopt_sample_idx, False))
        # print("4 thread_idx_set = " + str(thread_idx_set))
        # print("len(working_threads) = " + str(len(working_threads)))
        # print("CheckSimThreadAndBlockWhenNecessary")
        CheckSimThreadAndBlockWhenNecessary(working_threads, n_max_thread)
        # print("CheckSimThreadAndBlockWhenNecessary")
        # print("5 thread_idx_set = " + str(thread_idx_set))
        # print("len(working_threads) = " + str(len(working_threads)))

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
  n_max_thread = psutil.cpu_count()

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
  # cmt_l stores cost, model index, task value, and log index
  cmt = np.zeros((0, 2 + len(varying_task_element_indices)))
  log = np.zeros((0, 1))
  for rom_iter in model_indices:
    for idx in log_indices:
      path0 = eval_dir + '%d_%d_success.csv' % (rom_iter, idx)
      path1 = eval_dir + '%d_%d_cost_values.csv' % (rom_iter, idx)
      if os.path.exists(path0):
        current_cmt = np.zeros((1, 2 + len(varying_task_element_indices)))
        ### Read cost
        cost = np.loadtxt(path1, delimiter=',')[idx_sim_cost_element]
        current_cmt[0, 0] = cost
        if cost > max_cost_to_ignore:
          continue
        ### model iteration
        current_cmt[0, 1] = rom_iter
        ### Read actual task
        add_this_element = True
        col = 2
        for key in varying_task_element_indices:
          path_task = eval_dir + '%d_%d_ave_%s.csv' % (rom_iter, idx, key)
          task = np.loadtxt(path_task, delimiter=',').item()  # 0-dim scalar
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


def GetNominalSamplesToPlot(model_indices):
  ### Get all samples from trajopt for nominal cost
  trajopt_sample_indices_for_viz = CollectAllTrajoptSampleIndices() if plot_nominal else []
  print("sample_indices (trajopt) for nominal cost visualization = \n" + str(trajopt_sample_indices_for_viz))

  ### Get the samples to plot
  # nominal_cmt stores cost from trajopt, model index, and task value
  nominal_cmt = np.zeros((0, 2 + len(varying_task_element_indices)))
  for rom_iter in model_indices:
    for i in range(len(trajopt_sample_indices_for_viz)):
      sub_cmt = np.zeros((1, 2 + len(varying_task_element_indices)))
      ### Read cost
      cost = PlotNominalCost([rom_iter], trajopt_sample_indices_for_viz[i])[0][0]
      sub_cmt[0, 0] = cost
      if cost.item() > max_cost_to_ignore:
        continue
      ### Read model iteration
      sub_cmt[0, 1] = rom_iter
      ### Read nominal task
      col = 2
      add_this_sample = True
      for key in varying_task_element_indices:
        path = model_dir + "%d_%d_task.csv" % (rom_iter, trajopt_sample_indices_for_viz[i])
        if os.path.exists(path):
          task = np.loadtxt(path)[varying_task_element_indices[key]]
          sub_cmt[0, col] = task
          if (task < min_max_task_filter_for_viz[key][0]) or (task > min_max_task_filter_for_viz[key][1]):
            add_this_sample = False
        else:
          add_this_sample = False
        col += 1
      if not add_this_sample:
        continue
      ### Assign values
      nominal_cmt = np.vstack([nominal_cmt, sub_cmt])
  print("nominal_cmt.shape = " + str(nominal_cmt.shape))

  return nominal_cmt

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


def Generate4dPlots(cmt, nominal_cmt, plot_nominal):
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
    plt.savefig("%smodel_ter_vs_task1_4Dscatterplot.png" % (eval_dir))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    plt.savefig("%stask2_vs_task1_4Dscatterplot.png" % (eval_dir))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    plt.savefig("%stask2_vs_model_iter_4Dscatterplot.png" % (eval_dir))
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration


def Generate3dPlots(cmt, nominal_cmt, plot_nominal):
  plot_nominal = False  # nominal cost plot not implemented yet
  print("WARNING: currently Generate3dPlots() has not been generalized to > 1D task space for the nominal data")

  cmt = copy.deepcopy(cmt)
  nominal_cmt = copy.deepcopy(nominal_cmt)

  # Project tasks to the specified walking height and get the corresponding cost
  interpolator = LinearNDInterpolator(cmt[:, 1:], cmt[:, 0])
  z = interpolator(np.vstack((cmt[:, 1], cmt[:, 2], task_slice_value_ph * np.ones(len(cmt[:, 2])))).T)
  # Remove the rows corresponding to nan cost (from interpolation outside the region)
  cmt = cmt[~np.isnan(z), :]
  z = z[~np.isnan(z)]
  # Assign interpolated cost
  cmt[:, 0] = z
  print("interpolated cmt for 3D viz = " + str(cmt.shape))

  app = "_w_nom" if plot_nominal else ""
  ### scatter plot
  print("\nPlotting 3D scatter plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = plt.axes(projection="3d")
  ax.scatter3D(cmt[:, 1], cmt[:, 2], cmt[:, 0], color="green")
  if plot_nominal:
    ax.scatter3D(nominal_cmt[:, 1], nominal_cmt[:, 2], nominal_cmt[:, 0], "b")
  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('total cost')
  # plt.title("")
  if save_fig:
    ax.view_init(90, -90)  # look from +z axis. model iter vs task
    plt.savefig("%smodel_ter_vs_task_scatterplot%s_ph%.2f.png" % (eval_dir, app, task_slice_value_ph))
    ax.view_init(0, 0)  # look from x axis. cost vs task
    plt.savefig("%scost_vs_task_scatterplot%s_ph%.2f.png" % (eval_dir, app, task_slice_value_ph))
    ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
    plt.savefig("%scost_vs_model_iter_scatterplot%s_ph%.2f.png" % (eval_dir, app, task_slice_value_ph))
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration

  ### level set plot
  print("\nPlotting 3D level set plots...")

  fig = plt.figure(figsize=(10, 7))
  ax = plt.axes(projection="3d")
  if plot_nominal:
    # tcf = ax.tricontour(nominal_cmt[:, 0], nominal_cmt[:, 1],
    #   nominal_cmt[:, 2], zdir='y', cmap=cm.coolwarm)
    ax.scatter3D(nominal_cmt[:, 1], nominal_cmt[:, 2], nominal_cmt[:, 0], "b")
    # tcf = ax.plot_trisurf(nominal_cmt[:, 0], nominal_cmt[:, 1],
    #   nominal_cmt[:, 2], cmap=cm.coolwarm)
    pass
  print("If a deprecation warning comes up here, it's from within tricontour()")
  tcf = ax.tricontour(cmt[:, 1], cmt[:, 2], cmt[:, 0], zdir='y',
                      cmap=cm.coolwarm)
  fig.colorbar(tcf)
  ax.set_xlabel('model iterations')
  ax.set_ylabel('stride length (m)')
  ax.set_zlabel('total cost')
  ax.view_init(0, -90)  # look from -y axis. cost vs model iteration
  if save_fig:
    plt.savefig("%scost_vs_model_iter_contour%s_ph%.2f.png" % (eval_dir, app, task_slice_value_ph))


def Generate2dPlots(model_indices, cmt, nominal_cmt, plot_nominal):
  app = "_w_nom" if plot_nominal else ""

  if cmt.shape[1] != 4:
    raise ValueError("The code assumes cmt is 4D (two dimensinoal task)")

  # Some set up for plotting nominal costs
  num_task_dim = 2
  if plot_nominal:
    if np.sum(nominal_cmt[:, -1] != nominal_cmt[0, -1]) == 0:  # pelvis task is fixed in nominal traj
      if np.sum(np.array(task_slice_value_list)[:, 1] != nominal_cmt[0, -1]) == 0:  # The task slice has to be the same as the trajopt task
        num_task_dim = 1
      else:
        plot_nominal = False

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

    # Log the improvement percentage into a file
    masked_z = z[~np.isnan(z)]
    message = "Max cost improvement for task (sl, ph) = (%.2f, %.2f) m is %.1f %%\n" % (task_slice_value[0], task_slice_value[1], float((masked_z[0] - min(masked_z)) / masked_z[0] * 100))
    print(message)
    f = open(eval_dir + "costs_info.txt", "a")
    f.write(message)
    f.close()

  if plot_nominal:
    plt.gca().set_prop_cycle(None)  # reset color cycle

    interpolator = LinearNDInterpolator(nominal_cmt[:, 1:2+num_task_dim], nominal_cmt[:, 0])
    for task_slice_value in task_slice_value_list:
      t = np.array(task_slice_value[:num_task_dim]).reshape(num_task_dim, 1) * np.ones(n_model_iter + 1)
      z = interpolator(np.vstack((m, t)).T)
      plt.plot(m, z, '--', linewidth=3, label='(sl, ph) = (%.2f, %.2f) m (trajopt)' % tuple(task_slice_value))
      # plt.plot(m, z, '--', linewidth=3, label='stride length ' + str(task_slice_value) + " m")
      # plt.plot(m, z, 'k--', linewidth=3, label="trajectory optimization")

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
    plt.savefig("%scost_vs_model_iter%s_ph%.2f.png" % (eval_dir, app, task_slice_value_ph))

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
    plt.savefig("%scost_vs_task_ph%.2f.png" % (eval_dir, task_slice_value_ph))

  ### 2D plot (iter vs tasks; cost visualized in contours)
  print("\nPlotting iterations vs task...")

  data_list = [cmt, nominal_cmt]
  title_list = ["(Drake sim)", "(traj opt)"]
  app_list = ["", "_nom"]
  for i in range(2 if plot_nominal else 1):
    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()

    data = copy.deepcopy(data_list[i])

    # Interpolate to get the cost at specific pelvis height
    interpolator = LinearNDInterpolator(data[:, 1:], data[:, 0]) if i == 0 else LinearNDInterpolator(data[:, 1:2+num_task_dim], data[:, 0])
    z = interpolator(np.vstack((data[:, 1], data[:, 2], task_slice_value_ph * np.ones(len(data[:, 2])))).T) if i == 0 else interpolator(np.vstack((data[:, 1], data[:, 2])).T)

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
      plt.savefig("%scost_landscape_iter%s_ph%.2f.png" % (eval_dir, app_list[i], task_slice_value_ph))

  ### 2D plot; cost landscape (task1 vs task2; cost visualized in contours)
  print("\nPlotting 2D cost landscape (task1 vs task2)...")

  data_list = [cmt, nominal_cmt]
  title_list = ["(Drake sim)", "(traj opt)"]
  app_list = ["", "_nom"]
  for i in range(2 if plot_nominal else 1):
    plt.rcParams.update({'font.size': 14})
    fig, ax = plt.subplots()

    data = copy.deepcopy(data_list[i])

    # Interpolate to get the cost at specific pelvis height
    interpolator = LinearNDInterpolator(data[:, 1:], data[:, 0]) if i == 0 else LinearNDInterpolator(data[:, 1:2+num_task_dim], data[:, 0])
    z = interpolator(np.vstack((model_slice_value_cost_landsacpe * np.ones(len(data[:, 1])), data[:, 2], data[:, 3])).T) if i == 0 else interpolator(np.vstack((data[:, 1], data[:, 2])).T)

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
    surf = ax.tricontourf(data[:, 2], data[:, 3], z, levels=levels, cmap='coolwarm')
    fig.colorbar(surf, shrink=0.9, aspect=15)

    # plt.xlim([0, 135])
    plt.xlabel('stride length (m)')
    plt.ylabel('pelvis height (m)')
    plt.title('Cost landscape at iteration %d ' % model_slice_value_cost_landsacpe + title_list[i])
    plt.gcf().subplots_adjust(bottom=0.15)
    plt.gcf().subplots_adjust(left=0.15)
    if save_fig:
      plt.savefig("%scost_landscape%s_model_iter_%d.png" % (eval_dir, app_list[i], model_slice_value_cost_landsacpe))


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
    plt.savefig("%save_cost_vs_model_iter_range_%.2fto%.2f_ph%.2f.png" % (eval_dir, stride_length_range_to_average[0], stride_length_range_to_average[1], task_slice_value_ph))

  # Log the improvement percentage into a file
  message = "Max average cost improvement = %.1f %% over stride length from %.2f to %.2f\n" % (float((averaged_cost[0] - min(averaged_cost)) / averaged_cost[0] * 100), stride_length_range_to_average[0], stride_length_range_to_average[1])
  print(message)
  f = open(eval_dir + "costs_info.txt", "a")
  f.write(message)
  f.close()


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
    plt.savefig("%stask_space_vs_model_iter_ph%.2f.png" % (eval_dir, task_slice_value_ph))

  # Log the improvement percentage into a file
  message = "Max achievable task space improvement = %.1f %%\n" % float((max(task_range) - task_range[0]) / task_range[0] * 100)
  print(message)
  f = open(eval_dir + "costs_info.txt", "a")
  f.write(message)
  f.close()


def GetVaryingTaskElementIdx(tasks, nominal_task_names):
  indices = {}
  # names = tasks.GetVaryingTaskElementName()
  names = ["stride_length", "pelvis_height"]
  for name in names:
    indices[name] = nominal_task_names.index(name)
  return indices


# Grid tasks
class Tasks:
  # Constructor and builders
  def __init__(self):
    self.constructed = False
    self.task_data = {}

  def AddTaskDim(self, array, name, overwrite_existing=False):
    if self.constructed:
      raise ValueError("Cannot call this function after building the task obj")
    if not isinstance(array, (list, np.ndarray)):
      raise TypeError("array should be a list or numpy array")
    if not isinstance(name, str):
      raise TypeError("name should be a string")
    if not overwrite_existing:
      if name in self.task_data:
        raise ValueError("%s is already a key in task_data" % name)
    self.task_data[name] = np.array(array)

  def CreateTasklistViaDfs(self, level, indices_tuple):
    if level == self.n_dim:
      task = []
      # print("indices_tuple = " + str(indices_tuple))
      for i_dim in range(self.n_dim):
        task_idx = indices_tuple[i_dim]
        task.append(self.task_data[self.names[i_dim]][task_idx])
      self.task_list.append(task)
    else:
      for _ in self.task_data[self.names[level]]:
        # print("before " + str(indices_tuple))
        self.CreateTasklistViaDfs(level + 1,  copy.deepcopy(indices_tuple))
        indices_tuple[level] += 1
        # print("after " + str(indices_tuple))

  def Construct(self):
    self.constructed = True

    self.n_dim = len(self.task_data)

    # Compute n_task
    self.n_task = 1
    for key in self.task_data:
      self.n_task *= len(self.task_data[key])

    # Create ordered names
    self.names = []
    for key in self.task_data:
      self.names.append(key)

    # Create task list
    # self.task_list = np.zeros((self.n_task, self.n_dim))
    self.task_list = []
    level = 0
    indices_tuple = [0] * self.n_dim
    self.CreateTasklistViaDfs(level, copy.deepcopy(indices_tuple))
    self.task_arr = np.array(self.task_list)

    print("task_list = \n" + str(self.task_arr))

    if self.task_arr.shape != (self.n_task, self.n_dim):
      raise ValueError("self.task_list.shape = " + str(self.task_arr.shape) + ", but we expect (" + str(self.n_task) + ", " + str(self.n_dim) + ")")


  # Getters
  def tasks_info(self):
    output = ""
    for name in self.names:
      output += "%s ranges from %.3f to %.3f\n" % (name, self.task_data[name][0], self.task_data[name][-1])
    return output
  def get_task_dim(self):
    return self.n_dim
  def get_n_task(self):
    return self.n_task
  def GetDimIdxByName(self, name):
    if not (name in self.names):
      raise ValueError("%s doesn't exist in the tasks" % name)
    return self.names.index(name)
  def GetTask(self, task_idx):
    return self.task_arr[task_idx]
  def GetTaskList(self):
    return self.task_arr
  def GetVaryingTaskElementName(self):
    name_list = []
    for key in self.task_data:
      if len(self.task_data[key]) > 1:
        name_list.append(key)
    return name_list



if __name__ == "__main__":
  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)
  model_dir = parsed_yaml_file.get('dir_model')
  data_dir = parsed_yaml_file.get('dir_data')

  FOM_model_dir = ""

  eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval/"
  # eval_dir = "/media/yuming/data/dairlib_data/sim_cost_eval/"
  # eval_dir = "/home/yuming/Desktop/temp/test_sim_eval/"
  # eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval_2/"
  # eval_dir = "/home/yuming/Desktop/temp/3/sim_cost_eval_20210507/sim_cost_eval/"

  ### global parameters
  sim_end_time = 10.0
  spring_model = False
  # Parameters that are modified often
  target_realtime_rate = 0.1  # 0.04
  foot_step_from_planner = True
  stance_hip_angles_from_planner = True
  swing_hip_angle_from_planner = False
  init_sim_vel = True
  use_nominal_traj_pool = True
  set_sim_init_state_from_trajopt = True
  completely_use_trajs_from_model_opt_as_target = True

  ### parameters for model, task, and log indices
  # Model iteration list
  model_iter_idx_start = 1  # 0
  model_iter_idx_end = 100
  idx_spacing = 5

  # Task list
  n_task_sl = 30
  n_task_ph = 3
  tasks = Tasks()
  # tasks.AddTaskDim(np.linspace(-0.6, 0.6, n_task_sl), "stride_length")
  tasks.AddTaskDim(np.linspace(-0.42, 0.42, n_task_sl), "stride_length")
  # tasks.AddTaskDim(np.linspace(0, 0.2, n_task_sl), "stride_length")
  # tasks.AddTaskDim(np.linspace(0, 0, n_task_sl), "stride_length")
  # stride_length = np.linspace(-0.2, -0.1, n_task)
  # stride_length = np.linspace(-0.3, 0, n_task, endpoint=False)
  # stride_length = np.linspace(0.4, 0.5, n_task)
  # stride_length = np.hstack([np.linspace(-0.6, -0.4, n_task, endpoint=False),
  #                            -np.linspace(-0.6, -0.4, n_task, endpoint=False)])
  tasks.AddTaskDim([0.0], "ground_incline")
  tasks.AddTaskDim([-1.0], "duration")  # assign later; this shouldn't be a task for sim evaluation
  tasks.AddTaskDim([0.0], "turning_rate")
  # pelvis_heights used in both simulation and in CollectAllTrajoptSampleIndices
  tasks.AddTaskDim(np.linspace(0.85, 1.05, n_task_ph), "pelvis_height")
  # tasks.AddTaskDim([0.95], "pelvis_height")
  tasks.AddTaskDim([0.03], "swing_margin")  # This is not being used.

  # log indices
  log_idx_offset = 0  # 0

  ### Parameters for plotting
  log_indices_for_plot = []
  # log_indices_for_plot = list(range(log_idx_offset + tasks.get_n_task()))
  # log_indices_for_plot = list(range(240))
  save_fig = True
  plot_nominal = False
  task_tolerance = 0.05  # 0.01  # if tasks are not on the grid points exactly
  plot_main_cost = True  # main cost is the cost of which we take gradient during model optimization

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
  model_slice_value_cost_landsacpe = 1

  # Expected (averaged) cost over a task range
  stride_length_range_to_average = [-0.4, 0.4]
  # stride_length_range_to_average = [0.2, 0.4]
  final_iter_ave_cost = model_iter_idx_end
  # final_iter_ave_cost = 30

  ### Set up environment

  # Check directory names
  EnforceSlashEnding(model_dir)
  EnforceSlashEnding(data_dir)
  EnforceSlashEnding(FOM_model_dir)
  EnforceSlashEnding(eval_dir)

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
  # model_indices = [1]  # Overwrite
  print("model_indices = \n" + str(np.array(model_indices)))

  ### Create task list
  nominal_task_names = np.loadtxt(model_dir + "task_names.csv", dtype=str, delimiter=',')
  # Make sure the order is correct
  if not ((nominal_task_names[0] == "stride_length") &
          (nominal_task_names[1] == "ground_incline") &
          (nominal_task_names[2] == "duration") &
          (nominal_task_names[3] == "turning_rate") &
          (nominal_task_names[4] == "pelvis_height")):
    raise ValueError("ERROR: unexpected task name or task order")
  # Get duration from model optimization file
  path_1_0_task = model_dir + "1_0_task.csv"
  if os.path.exists(path_1_0_task):
    duration = np.loadtxt(path_1_0_task)[2]
  else:
    raise ValueError("%s doesn't exist" % path_1_0_task)
  tasks.AddTaskDim([duration], "duration", True)
  # Construct task object
  tasks.Construct()
  task_list = tasks.GetTaskList()

  # Make sure the dimension is correct
  if len(nominal_task_names) != tasks.get_task_dim():
    raise ValueError("sim eval task dimension is different from trajopt dim. "
                     "We want them to be the same becasue we use the same code "
                     "to plot sim cost and trajopt cost")

  # Index of task vector where we sweep through
  varying_task_element_indices = GetVaryingTaskElementIdx(tasks, list(nominal_task_names))
  print("varying_task_element_indices = " + str(varying_task_element_indices))

  # Plotting setups
  idx_sim_cost_element = -2 if plot_main_cost else -1

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
  # RunSimAndEvalCostInMultithread(model_indices, log_indices, task_list)

  # Cost evaluate only
  # EvalCostInMultithread(model_indices, log_indices)

  # Delete all logs but a few successful ones (for analysis later)
  # DeleteMostLogs(model_indices, log_indices)

  ### Plotting
  print("Nominal cost is from: " + model_dir)
  print("Simulation cost is from: " + eval_dir)
  RunCommand("rm " + eval_dir + "costs_info.txt", True)

  # Parameters for visualization
  max_cost_to_ignore = 100  # 2
  # mean_sl = 0.2
  # delta_sl = 0.1  # 0.1 #0.005
  # min_sl = mean_sl - delta_sl
  # max_sl = mean_sl + delta_sl
  min_sl = -100
  max_sl = 100
  min_max_task_filter_for_viz = {}
  min_max_task_filter_for_viz['stride_length'] = (min_sl, max_sl)
  min_max_task_filter_for_viz['pelvis_height'] = (0, 2)

  # Manual overwrite log_indices for plotting
  if len(log_indices_for_plot) != 0:
    log_indices = log_indices_for_plot
  print("log_indices for plotting = " + str(log_indices) + "\n")

  # Get samples to plot
  # cmt is a list of (model index, task value, and cost)
  cmt = GetSamplesToPlot(model_indices, log_indices)
  nominal_cmt = GetNominalSamplesToPlot(model_indices)
  if len(nominal_cmt) == 0:
    plot_nominal = False

  # Adjust slices value (for 2D plots)
  model_slices = AdjustSlices(model_slices)

  # Plot
  Generate4dPlots(cmt, nominal_cmt, plot_nominal)
  Generate3dPlots(cmt, nominal_cmt, plot_nominal)
  Generate2dPlots(model_indices, cmt, nominal_cmt, plot_nominal)

  ### Compute expected (averaged) cost
  ComputeExpectedCostOverTask(model_indices, cmt, stride_length_range_to_average)

  ### Compute task range over iteration
  ComputeAchievableTaskRangeOverIter(cmt)

  plt.show()
