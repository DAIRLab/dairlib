import sys

import yaml
import lcm
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
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
import pydrake.common as mut

def IsSimLogGood(x, t_x, desried_sim_end_time):
  # Check if the simulation ended early
  sim_time_tolerance = 0.1
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
def CheckSteadyState(x, t_x, time_list_at_td, separate_left_right_leg = False):
  t_x_touchdown_indices = []
  for time in time_list_at_td:
    t_x_touchdown_indices.append(np.argwhere(np.abs(t_x - time) < 1e-3)[0][0])

  # 1. stride length
  pelvis_x_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_x_at_td[i] = x[t_x_touchdown_indices[i], 4]
  pelvis_x_at_td_list = [pelvis_x_at_td[0::2], pelvis_x_at_td[1::2]] \
    if separate_left_right_leg else [pelvis_x_at_td]
  for i in range(len(pelvis_x_at_td_list)):
    stride_lengths = np.diff(pelvis_x_at_td_list[i])
    min_stride_length = min(stride_lengths)
    max_stride_length = max(stride_lengths)
    if abs(max_stride_length - min_stride_length) > stride_length_variation_tol:
      msg = msg_first_column + \
            ": not close to steady state. min and max stride length are " + \
            str(min_stride_length) + ", " + str(max_stride_length) + \
            "tolerance is " + str(stride_length_variation_tol) + "\n"
      print(msg)
      f = open(path_status_log, "a")
      f.write(msg)
      f.close()
      return False

  # 2. pelvis height
  pelvis_z_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_z_at_td[i] = x[t_x_touchdown_indices[i], 6]
  min_pelvis_height = min(pelvis_z_at_td)
  max_pelvis_height = max(pelvis_z_at_td)
  if abs(max_pelvis_height - min_pelvis_height) > pelvis_height_variation_tol:
    msg = msg_first_column + \
          ": not close to steady state. min and max pelvis height are " + \
          str(min_pelvis_height) + ", " + str(max_pelvis_height) + \
          "tolerance is " + str(pelvis_height_variation_tol) + "\n"
    print(msg)
    f = open(path_status_log, "a")
    f.write(msg)
    f.close()
    return False
  return True

def GetStartTimeAndEndTime(x, t_x, u, t_u, fsm, t_osc_debug):
  is_steady_state = False

  # Default time window values, can override
  t_start = t_u[10]
  t_end = t_u[-10]

  # Override t_start and t_end here
  if is_hardware:
    left_support = parsed_yaml.get('left_support')
    right_support = parsed_yaml.get('right_support')
    post_left_double_support = parsed_yaml.get('post_left_double_support')
    post_right_double_support = parsed_yaml.get('post_right_double_support')

    # Find the start time fo the double support state
    prev_state = -1
    for i in range(len(fsm)):
      state = fsm[i]
      if ((prev_state == left_support) and
          (state == post_left_double_support)) or \
          ((prev_state == right_support) and
           (state == post_right_double_support)):
        # import pdb;pdb.set_trace()
        t_fsm_start = t_osc_debug[i]

        if t_fsm_start + n_step * stride_period <= t_x[-1]:
          # Create time_list_at_td
          time_list_at_td = [t_fsm_start]
          for i in range(n_step):
            time_list_at_td.append(time_list_at_td[-1] + stride_period)

          is_steady_state = CheckSteadyState(x, t_x, time_list_at_td)

          # Stop moving the window to the future once we found one steady state window
          if is_steady_state:
            t_start = t_fsm_start
            t_end = t_fsm_start + n_step * stride_period
            break
      prev_state = state
  else:
    step_idx_start = int(t_end / stride_period) - n_step
    step_idx_end = int(t_end / stride_period)
    # step_idx_start = 14
    # step_idx_end = step_idx_start + n_step
    t_start = stride_period * step_idx_start
    t_end = stride_period * step_idx_end

    time_list_at_td = []
    for idx in range(step_idx_start, step_idx_end + 1):
      time_list_at_td.append(stride_period * idx)
    is_steady_state = CheckSteadyState(x, t_x, time_list_at_td)

  if is_steady_state:
    return t_start, t_end
  else:
    return -1, -1


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
    rom_iter_idx = int(sys.argv[3])
    log_idx = int(sys.argv[4])
    desried_sim_end_time = float(sys.argv[5])
    spring_model = (sys.argv[6].lower() == "true")

  # Parameters
  global n_step
  n_step = 4  # steps to average over

  # Weight used in model optimization
  w_Q = 0.005  # Assume all joints have the same cost weight, though not what trajopt is using. TODO: improve this
  w_R = 0.0002
  w_accel = 0.002 * w_Q

  # Steady state parameters
  global stride_length_variation_tol, pelvis_height_variation_tol
  stride_length_variation_tol = 0.05 if is_hardware else 0.02
  pelvis_height_variation_tol = 0.05 if is_hardware else 0.05

  # Read the controller parameters
  global parsed_yaml, stride_period
  parsed_yaml = yaml.full_load(open(
    "examples/goldilocks_models/rom_walking_gains.yaml"))
  stride_length_x = parsed_yaml.get('constant_step_length_x')
  left_support_duration = parsed_yaml.get('left_support_duration')
  double_support_duration = parsed_yaml.get('double_support_duration')
  stride_period = left_support_duration + double_support_duration
  const_walking_speed_x = stride_length_x / stride_period

  # File setting
  global directory, path_status_log
  directory = "../dairlib_data/goldilocks_models/hardware_cost_eval/" \
    if is_hardware else "../dairlib_data/goldilocks_models/sim_cost_eval/"
  Path(directory).mkdir(parents=True, exist_ok=True)
  path_status_log = directory + "hardware_status.txt" if is_hardware else "sim_status.txt"
  filename = file_path.split("/")[-1]

  # Message first column
  global msg_first_column
  msg_first_column = "hardware_" + filename if is_hardware else \
    "iteration #" + str(rom_iter_idx) + "log #" + str(log_idx)
  file_prefix = filename if is_hardware else "%d_%d" % (rom_iter_idx, log_idx)

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
  x, u_meas, t_x, u, t_u, contact_switch, t_contact_switch, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, t_osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log = process_lcm_log.process_log(log, pos_map, vel_map,
    act_map, controller_channel)

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

  # Check if the log data is ok for simulation
  if not is_hardware:
    if not IsSimLogGood(x, t_x, desried_sim_end_time):
      return

  # Pick the start and end time
  t_start, t_end = GetStartTimeAndEndTime(x, t_x, u, t_u, fsm, t_osc_debug)
  if t_start < 0:
    return

  ### Get indices from time
  global t_slice, t_u_slice
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 3e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 3e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  # Extract the trajectories that we use to calculate the cost
  t_x_extracted = t_x[t_slice]
  t_u_extracted = t_u[t_u_slice]
  x_extracted = x[t_slice, :]
  u_extracted = u[t_u_slice, :]
  n_x_data = x_extracted.shape[0]
  n_u_data = u_extracted.shape[0]

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

  # Get joint acceleration
  dx = np.diff(x_extracted, axis=0)
  vdot_numerical = dx[:, nq:]
  for i in range(len(dt_x)):
    vdot_numerical[i, :] /= dt_x[i]
  # Testing -- set the toe acceleration to 0
  # vdot_numerical[:, vel_map["toe_leftdot"]] = 0
  # vdot_numerical[:, vel_map["toe_rightdot"]] = 0

  # Testing (hacks) -- cap the acceleration within 500 to avoid contact spikes
  # max_accel = 500
  # vdot_numerical = np.clip(vdot_numerical, -max_accel, max_accel)

  cost_x = 0.0
  for i in range(n_x_data - 1):
    v_i = x_extracted[i, nq:]
    cost_x += (v_i.T @ v_i) * dt_x[i]
  cost_x *= (w_Q / n_step)

  cost_u = 0.0
  for i in range(n_u_data - 1):
    u_i = u_extracted[i, :]
    cost_u += (u_i.T @ u_i) * dt_u[i]
  cost_u *= (w_R / n_step)

  cost_accel = 0.0
  for i in range(n_x_data - 1):
    vdot_i = vdot_numerical[i, :]
    cost_accel += (vdot_i.T @ vdot_i) * dt_x[i]
  cost_accel *= (w_accel / n_step)

  total_cost = cost_x + cost_u + cost_accel
  print("t_start = " + str(t_start))
  print("t_end = " + str(t_end))
  print("n_x_data = " + str(n_x_data))
  print("n_u_data = " + str(n_u_data))
  print("cost_x = " + str(cost_x))
  print("cost_u = " + str(cost_u))
  print("cost_accel = " + str(cost_accel))
  print("total_cost = " + str(total_cost))
  # import pdb; pdb.set_trace()

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

  # Store into files
  names = ['cost_x',
           'cost_u',
           'cost_accel',
           'total_cost']
  names = ', '.join(names)
  values = [str(cost_x),
            str(cost_u),
            str(cost_accel),
            str(total_cost)]
  values = ', '.join(values)

  path = directory + "cost_names.csv"
  # print("writing to " + path)
  f = open(path, "w")
  f.write(names)
  f.close()

  path = directory + "%s_cost_values.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write(values)
  f.close()

  path = directory + "%s_ave_stride_length.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write(str((x_extracted[-1, 4] - x_extracted[0, 4]) / n_step))
  f.close()

  path = directory + "%s_success.csv" % file_prefix
  # print("writing to " + path)
  f = open(path, "w")
  f.write("1")
  f.close()


if __name__ == "__main__":
  main()
