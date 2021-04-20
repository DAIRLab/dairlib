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
  w_Q = 0.1
  w_R = 0.0002

  # Read the controller parameters
  a_yaml_file = open(
    "examples/goldilocks_models/controller/osc_rom_walking_gains.yaml")
  parsed_yaml_file = yaml.load(a_yaml_file)

  stride_length = parsed_yaml_file.get('stride_length')
  left_support_duration = parsed_yaml_file.get('left_support_duration')
  double_support_duration = parsed_yaml_file.get('double_support_duration')

  stride_period = left_support_duration + double_support_duration
  const_walking_speed_x = stride_length / stride_period

  # File setting
  directory = "../dairlib_data/goldilocks_models/sim_cost_eval/"
  Path(directory).mkdir(parents=True, exist_ok=True)

  # Script input arguments
  rom_iter_idx = int(sys.argv[3])
  sample_idx = int(sys.argv[4])
  desried_sim_end_time = float(sys.argv[5])

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
  plant_w_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant_w_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  Parser(plant_wo_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  plant_w_spr.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant_w_spr.Finalize()

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  l_toe_frame = plant_w_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_w_spr.GetBodyByName("toe_right").body_frame()
  world = plant_w_spr.world_frame()
  context = plant_w_spr.CreateDefaultContext()

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant_w_spr)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant_w_spr)

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  matplotlib.rcParams["savefig.directory"] = path

  # Read the log file
  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
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
  if desried_sim_end_time > 0:
    if abs(t_x[-1] - desried_sim_end_time) > 0.1:
      msg = "iteration #" + str(rom_iter_idx) + "sample #" + str(
        sample_idx) + ": sim end time (" + str(
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
      msg = "iteration #" + str(rom_iter_idx) + "sample #" + str(
        sample_idx) + ": pelvis fell below " + str(
        min_height) + " at time " + str(t_x[idx]) + "\n"
      print(msg)
      f = open(directory + "sim_status.txt", "a")
      f.write(msg)
      f.close()
      return

  x_extracted = x[t_slice, :]
  u_extracted = u[t_u_slice, :]
  n_x_data = x_extracted.shape[0]
  n_u_data = u_extracted.shape[0]

  # TODO: should use the real dt not the average dt
  dt_x = (t_end - t_start) / n_x_data
  dt_u = (t_end - t_start) / n_u_data

  # Get rid of spring joints
  x_extracted[:, nq + vel_map["knee_joint_leftdot"]] = 0
  x_extracted[:, nq + vel_map["ankle_spring_joint_leftdot"]] = 0
  x_extracted[:, nq + vel_map["knee_joint_rightdot"]] = 0
  x_extracted[:, nq + vel_map["ankle_spring_joint_rightdot"]] = 0

  cost_x = 0.0
  for i in range(n_x_data):
    v_i = x_extracted[i, nq:]
    cost_x += v_i.T @ v_i
  cost_x *= (w_Q * dt_x / n_step)

  cost_u = 0.0
  for i in range(n_u_data):
    u_i = u_extracted[i, :]
    cost_u += u_i.T @ u_i
  cost_u *= (w_R * dt_u / n_step)

  total_cost = cost_x + cost_u
  print("step_idx_start = " + str(step_idx_start))
  print("step_idx_end = " + str(step_idx_end))
  print("t_start = " + str(t_start))
  print("t_end = " + str(t_end))
  print("n_x_data = " + str(n_x_data))
  print("n_u_data = " + str(n_u_data))
  print("cost_x = " + str(cost_x))
  print("cost_u = " + str(cost_u))
  print("total_cost = " + str(total_cost))
  # import pdb; pdb.set_trace()

  # Store into files
  names = ['cost_x',
           'cost_u',
           'total_cost']
  names = ', '.join(names)
  values = [str(cost_x),
            str(cost_u),
            str(total_cost)]
  values = ', '.join(values)

  path = directory + "cost_names.csv"
  # print("writing to " + path)
  f = open(path, "w")
  f.write(names)
  f.close()

  path = directory + "%d_%d_cost_values.csv" % (rom_iter_idx, sample_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write(values)
  f.close()

  path = directory + "%d_%d_ave_stride_length.csv" % (rom_iter_idx, sample_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write(str((x_extracted[-1, 4] - x_extracted[0, 4]) / n_step))
  f.close()

  path = directory + "%d_%d_success.csv" % (rom_iter_idx, sample_idx)
  # print("writing to " + path)
  f = open(path, "w")
  f.write("1")
  f.close()


if __name__ == "__main__":
  main()
