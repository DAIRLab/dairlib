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


def main():
  # Parameters
  n_step = 4  # steps to average over

  # Weight used in model optimization
  w_Q = 0.005  # Assume all joints have the same cost weight, though not what trajopt is using
  w_R = 0.0002
  w_accel = 0.002 * w_Q

  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)

  stride_length_x = parsed_yaml_file.get('constant_step_length_x')
  left_support_duration = parsed_yaml_file.get('left_support_duration')
  double_support_duration = parsed_yaml_file.get('double_support_duration')

  stride_period = left_support_duration + double_support_duration
  const_walking_speed_x = stride_length_x / stride_period

  # File setting
  directory = "../dairlib_data/goldilocks_models/sim_cost_eval/"
  Path(directory).mkdir(parents=True, exist_ok=True)

  # Script input arguments
  rom_iter_idx = int(sys.argv[3])
  log_idx = int(sys.argv[4])
  desried_sim_end_time = float(sys.argv[5])
  spring_model = (sys.argv[6].lower() == "true")

  global t_start
  global t_end
  global t_slice
  global t_u_slice
  global filename
  global nq
  global nv
  global nx
  global pos_map
  global vel_map
  global act_map

  # Build plant
  mut.set_log_level("err")  # ignore warnings about joint limits
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  urdf_path = "examples/Cassie/urdf/cassie_v2.urdf" if spring_model else "examples/Cassie/urdf/cassie_fixed_springs.urdf"
  Parser(plant).AddModelFromFile(FindResourceOrThrow(urdf_path))
  plant.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant.Finalize()

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

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  matplotlib.rcParams["savefig.directory"] = path

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

  # Default time window values, can override
  t_start = t_u[10]
  t_end = t_u[-10]
  # Override here #
  # t_start = 5
  # t_end = 6
  step_idx_start = int(t_end / stride_period) - n_step
  step_idx_end = int(t_end / stride_period)
  # step_idx_start = 14
  # step_idx_end = step_idx_start + n_step
  t_start = stride_period * step_idx_start
  t_end = stride_period * step_idx_end
  ### Convert times to indices
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 3e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 3e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  ### All analysis scripts here

  # Check if the simulation ended early
  sim_time_tolerance = 0.1
  if desried_sim_end_time > 0:
    if abs(t_x[-1] - desried_sim_end_time) > sim_time_tolerance:
      msg = "iteration #" + str(rom_iter_idx) + "log #" + str(
        log_idx) + ": sim end time (" + str(
        t_x[-1]) + " s) is too different from the desired sim time (" + str(
        desried_sim_end_time) + " s)\n"
      print(msg)
      f = open(directory + "sim_status.txt", "a")
      f.write(msg)
      f.close()
      return

  # Check that the pelvis didn't fall below a certain height
  min_height = 0.4
  for idx in range(x.shape[0]):
    if x[idx, 6] < min_height:
      msg = "iteration #" + str(rom_iter_idx) + "log #" + str(
        log_idx) + ": pelvis fell below " + str(
        min_height) + " at time " + str(t_x[idx]) + "\n"
      print(msg)
      f = open(directory + "sim_status.txt", "a")
      f.write(msg)
      f.close()
      return

  # Check if it's close to steady state
  t_x_touchdown_indices = []
  for idx in range(step_idx_start, step_idx_end + 1):
    time = stride_period * idx
    t_x_touchdown_indices.append(np.argwhere(np.abs(t_x - time) < 1e-3)[0][0])
  # 1. stride length
  stride_length_variation_tol = 0.02
  pelvis_x_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_x_at_td[i] = x[t_x_touchdown_indices[i], 4]
  stride_lengths = np.diff(pelvis_x_at_td)
  min_stride_length = min(stride_lengths)
  max_stride_length = max(stride_lengths)
  if abs(max_stride_length - min_stride_length) > stride_length_variation_tol:
    msg = "iteration #" + str(rom_iter_idx) + "log #" + str(log_idx) + \
          ": not close to steady state. min and max stride length are " + \
          str(min_stride_length) + ", " + str(max_stride_length) + \
          "tolerance is " + str(stride_length_variation_tol) + "\n"
    print(msg)
    f = open(directory + "sim_status.txt", "a")
    f.write(msg)
    f.close()
    return
  # 2. pelvis height
  pelvis_height_variation_tol = 0.05
  pelvis_z_at_td = np.zeros(len(t_x_touchdown_indices))
  for i in range(len(t_x_touchdown_indices)):
    pelvis_z_at_td[i] = x[t_x_touchdown_indices[i], 6]
  min_pelvis_height = min(pelvis_z_at_td)
  max_pelvis_height = max(pelvis_z_at_td)
  if abs(max_pelvis_height - min_pelvis_height) > pelvis_height_variation_tol:
    msg = "iteration #" + str(rom_iter_idx) + "log #" + str(log_idx) + \
          ": not close to steady state. min and max pelvis height are " + \
          str(min_pelvis_height) + ", " + str(max_pelvis_height) + \
          "tolerance is " + str(pelvis_height_variation_tol) + "\n"
    print(msg)
    f = open(directory + "sim_status.txt", "a")
    f.write(msg)
    f.close()
    return

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
  #vdot_numerical[:, vel_map["toe_leftdot"]] = 0
  #vdot_numerical[:, vel_map["toe_rightdot"]] = 0

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
  print("step_idx_start = " + str(step_idx_start))
  print("step_idx_end = " + str(step_idx_end))
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

  path = directory + "%d_%d_cost_values.csv" % (rom_iter_idx, log_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write(values)
  f.close()

  path = directory + "%d_%d_ave_stride_length.csv" % (rom_iter_idx, log_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write(str((x_extracted[-1, 4] - x_extracted[0, 4]) / n_step))
  f.close()

  path = directory + "%d_%d_success.csv" % (rom_iter_idx, log_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write("1")
  f.close()


if __name__ == "__main__":
  main()
