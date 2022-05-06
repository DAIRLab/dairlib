import sys
import os

import yaml
import lcm
import scipy
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from pydairlib.analysis_scripts import process_lcm_log
import pathlib
from pathlib import Path
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
import pydrake.common as mut

from py_utils import FindVarValueInString


def PrintAndLogStatus(msg):
  print(msg)
  f = open(path_status_log, "a")
  f.write(msg)
  f.close()


def IsSimLogGood(x, t_x, desried_sim_end_time):
  # Check if the simulation ended early
  sim_time_tolerance = 2  # Sometimes the script stopped the logger too early
  if desried_sim_end_time > 0:
    if abs(t_x[-1] - desried_sim_end_time) > sim_time_tolerance:
      msg = msg_first_column + ": sim end time (" + str(
        t_x[-1]) + " s) is too different from the desired sim time (" + str(
        desried_sim_end_time) + " s)\n"
      print(msg)
      f = open(path_status_log, "a")
      f.write(msg)
      f.close()
      return False

  # Check that the pelvis didn't fall below a certain height
  min_height = 0.4
  for idx in range(x.shape[0]):
    if x[idx, 6] < min_height:
      msg = msg_first_column + ": pelvis fell below " + str(
        min_height) + " at time " + str(t_x[idx]) + "\n"
      print(msg)
      f = open(path_status_log, "a")
      f.write(msg)
      f.close()
      return False

  return True


### Check if it's close to steady state
def CheckSteadyStateAndSaveTasks(x, t_x, td_times, Print=True,
    separate_left_right_leg=False):
  is_steady_state = True

  t_x_touchdown_indices = []
  for time in td_times:
    t_x_touchdown_indices.append(np.argwhere(np.abs(t_x - time) < 2e-3)[0][0])

  # 1. stride length
  max_step_diff = 0.0
  pelvis_x_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_x_at_td[i] = x[t_x_touchdown_indices[i], 4]
  pelvis_x_at_td_list = [pelvis_x_at_td[0::2], pelvis_x_at_td[1::2]] \
    if separate_left_right_leg else [pelvis_x_at_td]
  for i in range(len(pelvis_x_at_td_list)):
    step_lengths = np.diff(pelvis_x_at_td_list[i])
    min_step_length = min(step_lengths)
    max_step_length = max(step_lengths)
    max_step_diff = max(max_step_diff, abs(max_step_length - min_step_length))
    if abs(max_step_length - min_step_length) > step_length_variation_tol:
      is_steady_state = False
      if Print:
        msg = msg_first_column + \
              ": not close to steady state. min and max stride length are " + \
              str(min_step_length) + ", " + str(max_step_length) + \
              "tolerance is " + str(step_length_variation_tol) + "\n"
        PrintAndLogStatus(msg)

  # 2. pelvis height
  pelvis_z_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_z_at_td[i] = x[t_x_touchdown_indices[i], 6]
  min_pelvis_height = min(pelvis_z_at_td)
  max_pelvis_height = max(pelvis_z_at_td)
  max_pelvis_height_diff = abs(max_pelvis_height - min_pelvis_height)
  if abs(max_pelvis_height - min_pelvis_height) > pelvis_height_variation_tol:
    is_steady_state = False
    if Print:
      msg = msg_first_column + \
            ": not close to steady state. min and max pelvis height are " + \
            str(min_pelvis_height) + ", " + str(max_pelvis_height) + \
            "tolerance is " + str(pelvis_height_variation_tol) + "\n"
      PrintAndLogStatus(msg)

  # Save average tasks
  ave_stride_length = np.average(np.diff(pelvis_x_at_td))
  ave_pelvis_height = np.average(pelvis_z_at_td)
  ave_tasks = {"ave_stride_length": ave_stride_length,
               "ave_pelvis_height": ave_pelvis_height}

  return is_steady_state, max_step_diff + max_pelvis_height_diff, ave_tasks


def GetSteadyStateWindows(x, t_x, u, t_u, fsm, t_osc_debug, pick_single_best_window=False):
  list_of_start_time_end_time_pair = []
  list_of_ave_tasks = []

  # Override t_start and t_end here
  if is_hardware:
    left_support = parsed_yaml.get('left_support')
    right_support = parsed_yaml.get('right_support')
    post_left_double_support = parsed_yaml.get('post_left_double_support')
    post_right_double_support = parsed_yaml.get('post_right_double_support')

    start_end_time_list = []
    max_diff_list = []
    ave_tasks_list = []
    prev_fsm_state = -1
    for i in range(len(fsm)):
      if t_osc_debug[i] + n_step * stride_period > t_shutoff:
        break

      fsm_state = fsm[i]
      # At the start of the double support state
      if ((prev_fsm_state == left_support) and
          (fsm_state == post_left_double_support)) or \
          ((prev_fsm_state == right_support) and
           (fsm_state == post_right_double_support)):
        t_fsm_start = t_osc_debug[i]

        if t_fsm_start + n_step * stride_period <= t_x[-1]:
          # Create a list of times at touchdown
          td_times = [t_fsm_start]
          for _ in range(n_step):
            td_times.append(td_times[-1] + stride_period)

          sub_window_is_ss, max_diff, ave_tasks = CheckSteadyStateAndSaveTasks(x, t_x, td_times, False)
          if sub_window_is_ss:
            start_end_time_list.append((td_times[0], td_times[-1]))
            max_diff_list.append(max_diff)
            ave_tasks_list.append(ave_tasks)
      prev_fsm_state = fsm_state

    # start_end_time_list would be non-empty when there is a steady state window
    is_steady_state = len(start_end_time_list) > 0
    if is_steady_state:
      if pick_single_best_window:
        # We pick the window that has the smallest steady state error
        idx = np.argmin(max_diff_list).item()
        list_of_start_time_end_time_pair.append(start_end_time_list[idx])
        list_of_ave_tasks.append(ave_tasks_list[idx])

        print("max_diff_list = ", max_diff_list)
        print("idx = ", idx)
        max_diff_list.sort()
        print("sorted = ", max_diff_list)
      else:
        list_of_start_time_end_time_pair = start_end_time_list
        list_of_ave_tasks = ave_tasks_list
    else:
      msg = msg_first_column + ": not close to steady state."
      PrintAndLogStatus(msg)

  else:
    t_start = t_u[10]
    t_end = t_u[-10]

    step_idx_start = int(t_end / stride_period) - n_step
    step_idx_end = int(t_end / stride_period)
    # step_idx_start = 14
    # step_idx_end = step_idx_start + n_step

    t_start = stride_period * step_idx_start
    t_end = stride_period * step_idx_end

    # Create a list of times at touchdown
    td_times = []
    for idx in range(step_idx_start, step_idx_end + 1):
      td_times.append(stride_period * idx)
    is_steady_state, _, ave_tasks = CheckSteadyStateAndSaveTasks(x, t_x, td_times)

    if is_steady_state:
      list_of_start_time_end_time_pair.append((t_start, t_end))
      list_of_ave_tasks.append(ave_tasks)

  return list_of_start_time_end_time_pair, list_of_ave_tasks



# cutoff_freq is in Hz
def ApplyLowPassFilter(x, t, cutoff_freq):
  dt = np.diff(t)
  x_filtered = x[0]
  for i in range(len(dt)):
    alpha = 2 * np.pi * dt[i] * cutoff_freq / (
        2 * np.pi * dt[i] * cutoff_freq + 1)
    x_filtered = alpha * x[i + 1] + (1 - alpha) * x_filtered
    x[i + 1] = x_filtered
  return x

def GetCostWeight(nq, nv, nu):
  filename = '1_0_trajopt_settings_and_cost_breakdown.txt'
  with open(parsed_yaml.get('dir_model') + filename, 'rt') as f:
    contents = f.read()

  w_Q = FindVarValueInString(contents, "w_Q =")
  w_Q_vy = FindVarValueInString(contents, "w_Q_vy =")
  w_Q_vz = FindVarValueInString(contents, "w_Q_vz =")
  w_Q_v_swing_hip_roll = w_Q * 1
  w_Q_v_swing_toe = FindVarValueInString(contents, "w_Q_v_swing_toe =")
  w_R = FindVarValueInString(contents, "w_R =")
  w_R_swing_toe = FindVarValueInString(contents, "w_R_swing_toe =")
  w_lambda = FindVarValueInString(contents, "w_lambda =")
  w_lambda_diff = FindVarValueInString(contents, "w_lambda_diff =")
  w_q_diff = FindVarValueInString(contents, "w_q_diff =")
  w_q_diff_swing_toe = FindVarValueInString(contents, "w_q_diff_swing_toe =")
  w_v_diff = FindVarValueInString(contents, "w_v_diff =")
  w_v_diff_swing_leg = FindVarValueInString(contents, "w_v_diff_swing_leg =")
  w_u_diff = FindVarValueInString(contents, "w_u_diff =")
  w_q_hip_roll = FindVarValueInString(contents, "w_q_hip_roll =")
  # w_q_hip_roll = 0.1
  w_q_hip_yaw = FindVarValueInString(contents, "w_q_hip_yaw =")
  w_q_quat = FindVarValueInString(contents, "w_q_quat =")
  w_joint_accel = FindVarValueInString(contents, "w_joint_accel =")
  w_reg = FindVarValueInString(contents, "w_reg =")

  W_Q_ls = w_Q * np.identity(nv)
  W_Q_ls[4, 4] = w_Q_vy
  W_Q_ls[5, 5] = w_Q_vz
  W_Q_ls[7, 7] = w_Q_v_swing_hip_roll
  W_Q_ls[vel_map["toe_rightdot"], vel_map["toe_rightdot"]] = w_Q_v_swing_toe
  W_Q_rs = w_Q * np.identity(nv)
  W_Q_rs[4, 4] = w_Q_vy
  W_Q_rs[5, 5] = w_Q_vz
  W_Q_rs[6, 6] = w_Q_v_swing_hip_roll
  W_Q_rs[vel_map["toe_leftdot"], vel_map["toe_leftdot"]] = w_Q_v_swing_toe

  W_R_ls = w_R * np.identity(nu)
  W_R_ls[act_map["toe_right_motor"], act_map["toe_right_motor"]] = w_R_swing_toe
  W_R_rs = w_R * np.identity(nu)
  W_R_rs[act_map["toe_left_motor"], act_map["toe_left_motor"]] = w_R_swing_toe

  W_joint_accel_ls = w_joint_accel * W_Q_ls
  W_joint_accel_rs = w_joint_accel * W_Q_rs

  W_q_diff_ls = w_q_diff * np.identity(nq)
  W_q_diff_ls[pos_map["toe_right"], pos_map["toe_right"]] = w_q_diff_swing_toe
  W_q_diff_rs = w_q_diff * np.identity(nq)
  W_q_diff_rs[pos_map["toe_left"], pos_map["toe_left"]] = w_q_diff_swing_toe

  W_v_diff_ls = w_v_diff * np.identity(nv)
  W_v_diff_ls[vel_map["toe_rightdot"], vel_map["toe_rightdot"]] = w_v_diff_swing_leg
  W_v_diff_rs = w_v_diff * np.identity(nv)
  W_v_diff_rs[vel_map["toe_leftdot"], vel_map["toe_leftdot"]] = w_v_diff_swing_leg

  w_lambda = w_lambda ** 2

  # Creat cost dictionary
  weight_dict = {"W_Q_ls": W_Q_ls,
                 "W_Q_rs": W_Q_rs,
                 "W_R_ls": W_R_ls,
                 "W_R_rs": W_R_rs,
                 "W_joint_accel_ls": W_joint_accel_ls,
                 "W_joint_accel_rs": W_joint_accel_rs,
                 "w_lambda": w_lambda,
                 "w_lambda_diff": w_lambda_diff,
                 "W_q_diff_ls": W_q_diff_ls,
                 "W_q_diff_rs": W_q_diff_rs,
                 "W_v_diff_ls": W_v_diff_ls,
                 "W_v_diff_rs": W_v_diff_rs,
                 "w_u_diff": w_u_diff,
                 "w_q_hip_roll": w_q_hip_roll,
                 "w_q_hip_yaw": w_q_hip_yaw,
                 "w_q_quat": w_q_quat,
                 "w_reg": w_reg}

  # Printing
  # for key in weight_dict:
  #   print(key, " = \n", weight_dict[key])

  return weight_dict


def CalcCost(t_start, t_end, n_x_data, n_u_data, dt_x, dt_u, x_extracted,
    u_extracted, vdot_numerical, fsm_tx_extracted, fsm_tu_extracted, weight_dict):
  cost_dict = {}

  # Get indices at virtual nodes
  # TODO: n_node could be extracted automatically. I think the right approach is to integrate every reg cost over time in trajopt, so it's easier for us here.
  n_node = 20
  x_data_knot_idx = np.linspace(0, n_x_data - 2, n_node * n_step)  # use -2 for vdot
  u_data_knot_idx = np.linspace(0, n_u_data - 1, n_node * n_step)

  # Create left right stance array
  # is left stance
  ls_tx = (fsm_tx_extracted == 0) + (fsm_tx_extracted == 3)
  # is right stance
  rs_tx = (fsm_tx_extracted == 1) + (fsm_tx_extracted == 4)
  if np.sum(ls_tx) + np.sum(rs_tx) != n_x_data:
    raise ValueError("Left/right stance identification has error")
  ls_tu = (fsm_tu_extracted == 0) + (fsm_tu_extracted == 3)

  cost_x = 0.0
  for i in range(n_x_data - 1):
    v_i = x_extracted[i, nq:]
    if ls_tx[i]:
      cost_x += (v_i.T @ weight_dict["W_Q_ls"] @ v_i) * dt_x[i]
    else:
      cost_x += (v_i.T @ weight_dict["W_Q_rs"] @ v_i) * dt_x[i]
  cost_dict["cost_x"] = cost_x

  cost_u = 0.0
  for i in range(n_u_data - 1):
    u_i = u_extracted[i, :]
    if ls_tu[i]:
      cost_u += (u_i.T @ weight_dict["W_R_ls"] @ u_i) * dt_u[i]
    else:
      cost_u += (u_i.T @ weight_dict["W_R_rs"] @ u_i) * dt_u[i]
  cost_dict["cost_u"] = cost_u

  cost_accel = 0.0
  for k in range(len(x_data_knot_idx)):
    # Testing code
    # if k == 0 or k == 1:
    #   continue  # dt = 0
    
    i = int(x_data_knot_idx[k])
    vdot_i = vdot_numerical[i, :]
    if ls_tx[i]:
      cost_accel += (vdot_i.T @ weight_dict["W_joint_accel_ls"] @ vdot_i)
    else:
      cost_accel += (vdot_i.T @ weight_dict["W_joint_accel_rs"] @ vdot_i)
  cost_dict["cost_accel"] = cost_accel

  cost_pos_diff = 0.0
  if np.linalg.norm(weight_dict["W_q_diff_ls"]) != 0:
    for k in range(len(x_data_knot_idx) - 1):
      i = int(x_data_knot_idx[k])
      j = int(x_data_knot_idx[k+1])
      delta_q = x_extracted[j, :nq] - x_extracted[i, :nq]
      if ls_tx[i] != ls_tx[j]:  # Rough approximation. We don't need to count between steps because we added one more node each mode
        continue
      if ls_tx[i]:  # TODO: this is a very rough estimate, you can improve this. (but maybe it doesn't matter too much)
        cost_pos_diff += (delta_q.T @ weight_dict["W_q_diff_ls"] @ delta_q)
      else:
        cost_pos_diff += (delta_q.T @ weight_dict["W_q_diff_rs"] @ delta_q)
  cost_dict["cost_pos_diff"] = cost_pos_diff

  cost_vel_diff = 0.0
  if np.linalg.norm(weight_dict["W_v_diff_ls"]) != 0:
    for k in range(len(x_data_knot_idx) - 1):
      i = int(x_data_knot_idx[k])
      j = int(x_data_knot_idx[k+1])
      delta_v = x_extracted[j, nq:] - x_extracted[i, nq:]
      if ls_tx[i] != ls_tx[j]:  # Rough approximation. We don't need to count between steps because we added one more node each mode
        continue
      if ls_tx[i]:  # TODO: this is a very rough estimate, you can improve this. (but maybe it doesn't matter too much)
        cost_vel_diff += (delta_v.T @ weight_dict["W_v_diff_ls"] @ delta_v)
      else:
        cost_vel_diff += (delta_v.T @ weight_dict["W_v_diff_rs"] @ delta_v)
  cost_dict["cost_vel_diff"] = cost_vel_diff

  cost_u_diff = 0.0
  if np.linalg.norm(weight_dict["w_u_diff"]) != 0:
    for k in range(len(u_data_knot_idx) - 1):
      i = int(u_data_knot_idx[k])
      j = int(u_data_knot_idx[k+1])
      if ls_tu[i] != ls_tu[j]:  # Rough approximation. We don't need to count between steps because we added one more node each mode
        continue
      delta_u = u_extracted[j, :] - u_extracted[i, :]
      cost_u_diff += (delta_u.T @ delta_u)
    cost_u_diff *= weight_dict["w_u_diff"]
  cost_dict["cost_u_diff"] = cost_u_diff

  cost_q_hip_roll = 0.0
  cost_q_hip_yaw = 0.0
  for k in range(len(x_data_knot_idx)):
    i = int(x_data_knot_idx[k])
    q_hip_roll = x_extracted[i, 7:9]
    q_hip_yaw = x_extracted[i, 9:11]
    cost_q_hip_roll += (q_hip_roll.T @ q_hip_roll)
    cost_q_hip_yaw += (q_hip_yaw.T @ q_hip_yaw)
  cost_q_hip_roll *= weight_dict["w_q_hip_roll"]
  cost_q_hip_yaw *= weight_dict["w_q_hip_yaw"]
  cost_dict["cost_q_hip_roll"] = cost_q_hip_roll
  cost_dict["cost_q_hip_yaw"] = cost_q_hip_yaw

  # TODO: improve this
  print("WARNING: currently we can only compute cost for zero turning rate")
  cost_q_quat_xyz = 0.0
  unit_quat = np.array([1, 0, 0, 0])
  for k in range(len(x_data_knot_idx)):
    i = int(x_data_knot_idx[k])
    q_quat = x_extracted[i, :4]
    quat_error = unit_quat - q_quat
    cost_q_quat_xyz += (quat_error.T @ quat_error)
  cost_q_quat_xyz *= weight_dict["w_q_quat"]
  cost_dict["cost_q_quat_xyz"] = cost_q_quat_xyz

  # TODO: currently the regularzation cost doesn't include force.
  cost_regularization = 0.0
  for frac_i in x_data_knot_idx:
    i = int(frac_i)
    x_i = x_extracted[i, :]
    cost_regularization += (x_i.T @ x_i)
  for frac_i in u_data_knot_idx:
    i = int(frac_i)
    u_i = u_extracted[i, :]
    cost_regularization += (u_i.T @ u_i)
  cost_regularization *= weight_dict["w_reg"]
  cost_dict["cost_regularization"] = cost_regularization

  # TODO: finish implementing this part
  cost_lambda_x_diff = 0.0
  cost_lambda_diff = 0.0
  cost_lambda = 0.0
  cost_collocation_lambda = 0.0
  cost_tau = 0.0

  # Get total cost
  total_cost = 0.0
  for key in cost_dict:
    total_cost += cost_dict[key]
  cost_dict["total_cost"] = total_cost

  # Get main total cost (excluding regularization cost)
  total_main_cost = cost_x + cost_u + cost_accel
  cost_dict["total_main_cost"] = total_main_cost

  # Get total reg cost
  total_reg_cost = total_cost - total_main_cost
  cost_dict["total_reg_cost"] = total_reg_cost

  # Divide every cost by n_step at the end
  for key in cost_dict:
    cost_dict[key] /= n_step

  # Printing
  # for key in cost_dict:
  #   print(key, " = ", cost_dict[key])

  return cost_dict


def EnforceSlashEnding(dir):
  if len(dir) > 0 and dir[-1] != "/":
    raise ValueError("Directory path name should end with slash")


# TODO: maybe we should use pelvis height wrt stance foot in CheckSteadyState()

# TODO: we also need to add horizontal steady state check (currently it has to be close to 0)

# TODO: also need to make the stride length local to pelvis heading

def main():
  # Script input arguments
  global is_hardware
  rom_iter_idx = -1
  log_idx = -1
  desried_sim_end_time = -1.0
  spring_model = True

  file_path = sys.argv[1]
  controller_channel = sys.argv[2]
  is_hardware = (sys.argv[3].lower() == "true")
  if is_hardware:
    spring_model = True
  else:
    rom_iter_idx = int(sys.argv[3])  # doesn't affect cost calculation
    log_idx = int(sys.argv[4])       # doesn't affect cost calculation
    desried_sim_end_time = float(sys.argv[5])
    spring_model = (sys.argv[6].lower() == "true")

  global eval_dir
  if is_hardware:
    if len(sys.argv) >= 5:
      eval_dir = sys.argv[4]
    else:
      eval_dir = "../dairlib_data/goldilocks_models/hardware_cost_eval/"
  else:
    if len(sys.argv) >= 8:
      eval_dir = sys.argv[7]
    else:
      eval_dir = "../dairlib_data/goldilocks_models/sim_cost_eval/"

  if is_hardware:
    if len(sys.argv) >= 6:
      rom_iter_idx = int(sys.argv[5])

  # Parameters
  global n_step
  n_step = 4  #1  # steps to average over

  # Steady state parameters
  global step_length_variation_tol, pelvis_height_variation_tol
  step_length_variation_tol = 0.05 if is_hardware else 0.02
  pelvis_height_variation_tol = 0.05 if is_hardware else 0.05

  # Some parameters
  low_pass_filter = True

  # Read the controller parameters
  global parsed_yaml, stride_period
  # tip: change full_load() to safe_load() if there is a yaml version issue
  parsed_yaml = yaml.safe_load(open(
    "examples/goldilocks_models/rom_walking_gains.yaml"))
  stride_length_x = parsed_yaml.get('constant_step_length_x')
  left_support_duration = parsed_yaml.get('left_support_duration')
  double_support_duration = parsed_yaml.get('double_support_duration')
  stride_period = left_support_duration + double_support_duration
  const_walking_speed_x = stride_length_x / stride_period

  # File setting
  global path_status_log
  EnforceSlashEnding(eval_dir)
  Path(eval_dir).mkdir(parents=True, exist_ok=True)
  path_status_log = eval_dir + (
    "hardware_status.txt" if is_hardware else "sim_status.txt")
  filename = file_path.split("/")[-1]
  print("eval_dir = ", eval_dir)
  print("path_status_log = ", path_status_log)

  # Message first column
  global msg_first_column
  msg_first_column = "hardware_" + filename if is_hardware else \
    "iteration #" + str(rom_iter_idx) + "log #" + str(log_idx)

  # File prefix
  file_prefix = filename if is_hardware else "%d_%d" % (rom_iter_idx, log_idx)
  print("file_prefix = ", file_prefix)
  starting_log_idx = 0
  if is_hardware:
    while os.path.exists(eval_dir + file_prefix + "_%d_%d" % (rom_iter_idx, starting_log_idx) + "_cost_values.csv"):
      starting_log_idx += 1

  # Build plant
  mut.set_log_level("err")  # ignore warnings about joint limits
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  urdf_path = "examples/Cassie/urdf/cassie_v2.urdf" if spring_model else \
    "examples/Cassie/urdf/cassie_fixed_springs.urdf"
  Parser(plant).AddModelFromFile(FindResourceOrThrow(urdf_path))
  plant.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  global nq, nv, nx
  global pos_map, vel_map, act_map

  # relevant MBP parameters
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  context = plant.CreateDefaultContext()

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)

  log = lcm.EventLog(file_path, "r")

  matplotlib.rcParams["savefig.directory"] = pathlib.Path(file_path).parent

  # Read the log file
  x, u_meas, imu_aceel, t_x, u, t_u, contact_switch, t_contact_switch, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, t_osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  u_dispatcher, t_u_dispatcher, \
  osc_output, input_supervisor_status, t_input_supervisor, full_log = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

  n_msgs = len(cassie_out)
  knee_pos = np.zeros(n_msgs)
  t_cassie_out = np.zeros(n_msgs)
  estop_signal = np.zeros(n_msgs)
  motor_torques = np.zeros(n_msgs)
  for i in range(n_msgs):
    knee_pos[i] = cassie_out[i].leftLeg.kneeDrive.velocity
    t_cassie_out[i] = cassie_out[i].utime / 1e6
    motor_torques[i] = cassie_out[i].rightLeg.kneeDrive.torque
    estop_signal[i] = cassie_out[i].pelvis.radio.channel[8]

  print("Finished parsing the log")

  global t_shutoff
  if is_hardware:
    t_shutoff = t_cassie_out[-1]
    for i in reversed(range(len(cassie_out))):
      if cassie_out[i].pelvis.radio.channel[-1] < 0:  # soft e-stop triggered
        t_shutoff = t_cassie_out[i]
      if cassie_out[i].pelvis.radio.channel[8] < 0:  # hard e-stop triggered
        t_shutoff = t_cassie_out[i]
    print("t_shutoff = ", t_shutoff)

  # Check if the log data is ok for simulation
  if not is_hardware:
    if not IsSimLogGood(x, t_x, desried_sim_end_time):
      return

  # Pick the start and end time
  list_of_start_time_end_time_pair, list_of_ave_tasks = GetSteadyStateWindows(x, t_x, u, t_u, fsm, t_osc_debug)

  # Set start and end time manually
  # list_of_start_time_end_time_pair = [(0.001, 0.35)]

  if len(list_of_start_time_end_time_pair) == 0:
    return

  # Some checks -- currently we don't support many windows for simulation log,
  # because of file_prefix
  if not is_hardware:
    if len(list_of_start_time_end_time_pair) > 1:
      raise ValueError("We currently don't support multiple windows for sim log")

  # Weight used in model optimization
  weight_dict = GetCostWeight(nq, nv, nu)

  for i in range(len(list_of_start_time_end_time_pair)):
    t_start, t_end = list_of_start_time_end_time_pair[i]
    ave_tasks = list_of_ave_tasks[i]

    cost_dict = ProcessDataGivenStartTimeAndEndTime(t_start, t_end, weight_dict,
      is_hardware, spring_model, low_pass_filter, n_step,
      x, u, fsm, t_x, t_u, t_osc_debug, nq, nu, nv, vel_map)

    # Get a file_prefix name
    file_prefix_this_loop = file_prefix
    if is_hardware:
      file_prefix_this_loop += "_%d_%d" % (rom_iter_idx, starting_log_idx)
      starting_log_idx += 1

    SaveData(cost_dict, file_prefix_this_loop, ave_tasks)


def ProcessDataGivenStartTimeAndEndTime(t_start, t_end, weight_dict, is_hardware, spring_model,
    low_pass_filter, n_step,
    x, u, fsm, t_x, t_u, t_osc_debug, nq, nu, nv, vel_map):
  ### Get indices from time
  global t_slice, t_u_slice
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  t_start_idx = np.argwhere(np.abs(t_u - t_start) < 3e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_u - t_end) < 3e-3)[0][0]
  t_u_slice = slice(t_start_idx, t_end_idx)
  t_start_idx = np.argwhere(np.abs(t_osc_debug - t_start) < 3e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_osc_debug - t_end) < 3e-3)[0][0]
  t_fsm_slice = slice(t_start_idx, t_end_idx)

  # Extract the trajectories that we use to calculate the cost
  t_x_extracted = t_x[t_slice]
  t_u_extracted = t_u[t_u_slice]
  x_extracted = x[t_slice, :]
  u_extracted = u[t_u_slice, :]
  n_x_data = x_extracted.shape[0]
  n_u_data = u_extracted.shape[0]

  f = interp1d(t_osc_debug, fsm, kind='zero', fill_value="extrapolate")
  fsm_tx_extracted = f(t_x_extracted)
  fsm_tu_extracted = f(t_u_extracted)

  # ave_dt_x = (t_end - t_start) / n_x_data
  # ave_dt_u = (t_end - t_start) / n_u_data
  dt_x = np.diff(t_x_extracted)
  dt_u = np.diff(t_u_extracted)

  # Get rid of spring joints
  if spring_model:
    x_extracted[:, nq + vel_map["knee_joint_leftdot"]] = 0
    x_extracted[:, nq + vel_map["ankle_spring_joint_leftdot"]] = 0
    x_extracted[:, nq + vel_map["knee_joint_rightdot"]] = 0
    x_extracted[:, nq + vel_map["ankle_spring_joint_rightdot"]] = 0

  # Apply low pass filter to vel
  cutoff_freq = 100 #100
  if is_hardware:
    if low_pass_filter:
      x_extracted[:, nq:] = ApplyLowPassFilter(x_extracted[:, nq:], t_x_extracted,
        cutoff_freq)

  # Get joint acceleration
  dx = np.diff(x_extracted, axis=0)
  vdot_numerical = dx[:, nq:]
  for i in range(len(dt_x)):
    vdot_numerical[i, :] /= dt_x[i]

  # Remove the spikes in the acceleration (IMPORTANT: we assume the velocity is not filtered)
  if not is_hardware:
    row_wise_maximum = np.amax(vdot_numerical, axis=1)
    big_value_occurances = row_wise_maximum > 500
    for i in range(len(big_value_occurances)):
      if (big_value_occurances[i]):
        if i == 0:
          vdot_numerical[i,:] = 0
        else:
          vdot_numerical[i,:] = vdot_numerical[i-1,:]

  if low_pass_filter:
    vdot_numerical = ApplyLowPassFilter(vdot_numerical, t_x_extracted[1:], cutoff_freq)
    # vdot_numerical[:, vel_map["ankle_joint_rightdot"]] = ApplyLowPassFilter(vdot_numerical[:, vel_map["ankle_joint_rightdot"]], t_x_extracted[1:], cutoff_freq)
    # vdot_numerical[:, vel_map["ankle_joint_leftdot"]] = ApplyLowPassFilter(vdot_numerical[:, vel_map["ankle_joint_leftdot"]], t_x_extracted[1:], cutoff_freq)
    # vdot_numerical[:, vel_map["toe_rightdot"]] = ApplyLowPassFilter(vdot_numerical[:, vel_map["toe_rightdot"]], t_x_extracted[1:], cutoff_freq)
    # vdot_numerical[:, vel_map["toe_leftdot"]] = ApplyLowPassFilter(vdot_numerical[:, vel_map["toe_leftdot"]], t_x_extracted[1:], cutoff_freq)

  # import pdb; pdb.set_trace()
  # Testing -- set the toe vel to 0
  # x_extracted[:, nq + vel_map["toe_leftdot"]] = 0
  # x_extracted[:, nq + vel_map["toe_rightdot"]] = 0
  # Testing -- set the toe acceleration to 0
  # vdot_numerical[:, vel_map["toe_leftdot"]] = 0
  # vdot_numerical[:, vel_map["toe_rightdot"]] = 0

  # Testing (hacks) -- cap the acceleration within 500 to avoid contact spikes
  # max_accel = 750
  # vdot_numerical = np.clip(vdot_numerical, -max_accel, max_accel)

  # Compute all costs
  cost_dict = CalcCost(t_start, t_end, n_x_data, n_u_data, dt_x, dt_u, x_extracted,
    u_extracted, vdot_numerical, fsm_tx_extracted, fsm_tu_extracted, weight_dict)

  # # Testing ankle and toe accleration
  # vdot_numerical_copy1 = np.copy(vdot_numerical)
  # vdot_numerical_copy1[:, :nv-4] = 0
  # cost_accel_toe_ankle = 0.0
  # for i in range(n_x_data - 1):
  #   vdot_i = vdot_numerical_copy1[i, :]
  #   cost_accel_toe_ankle += (vdot_i.T @ vdot_i) * dt_x[i]
  # cost_accel_toe_ankle *= (w_accel / n_step)
  # print("cost_accel_toe_ankle = " + str(cost_accel_toe_ankle))
  #
  # # Testing ankle and toe accleration
  # vdot_numerical_copy2 = np.copy(vdot_numerical)
  # vdot_numerical_copy2[:, nv-4:] = 0
  # cost_accel_except_toe_ankle = 0.0
  # for i in range(n_x_data - 1):
  #   vdot_i = vdot_numerical_copy2[i, :]
  #   cost_accel_except_toe_ankle += (vdot_i.T @ vdot_i) * dt_x[i]
  # cost_accel_except_toe_ankle *= (w_accel / n_step)
  # print("cost_accel_except_toe_ankle = " + str(cost_accel_except_toe_ankle))

  return cost_dict


def SaveData(cost_dict, file_prefix, ave_tasks):
  # Store into files
  names = ['cost_x',
           'cost_u',
           'cost_accel',
           'cost_pos_diff',
           'cost_vel_diff',
           'cost_u_diff',
           'cost_q_hip_roll',
           'cost_q_hip_yaw',
           'cost_q_quat_xyz',
           'cost_regularization',
           'total_reg_cost',
           'total_main_cost',
           'total_cost']
  values = [str(cost_dict[name]) for name in names]

  values = ', '.join(values)

  path = eval_dir + "cost_names.csv"
  # print("writing to " + path)
  f = open(path, "w")
  f.write(', '.join(names))
  f.close()

  path = eval_dir + "%s_cost_values.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write(values)
  f.close()

  path = eval_dir + "%s_ave_stride_length.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write(str(ave_tasks["ave_stride_length"]))
  f.close()

  path = eval_dir + "%s_ave_pelvis_height.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write(str(ave_tasks["ave_pelvis_height"]))
  f.close()

  path = eval_dir + "%s_success.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write("1")
  f.close()


if __name__ == "__main__":
  main()

