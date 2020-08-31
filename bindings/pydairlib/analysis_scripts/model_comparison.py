import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import process_lcm_log
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow


def main():
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
  global pos_map_spr_to_wo_spr
  global vel_map_spr_to_wo_spr
  global world
  global plant_w_spr
  global context_w_spr

  builder = DiagramBuilder()
  plant_w_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant_w_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  Parser(plant_wo_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf"))
  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  l_toe_frame = plant_w_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_w_spr.GetBodyByName("toe_right").body_frame()
  world = plant_w_spr.world_frame()
  context_w_spr = plant_w_spr.CreateDefaultContext()
  context_wo_spr = plant_wo_spr.CreateDefaultContext()

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  pos_map_spr_to_wo_spr = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
  vel_map_spr_to_wo_spr = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant_w_spr)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant_w_spr)

  x_datatypes_wo_spr = pydairlib.multibody.createStateNameVectorFromMap(plant_wo_spr)
  u_datatypes_wo_spr = pydairlib.multibody.createActuatorNameVectorFromMap(plant_wo_spr)

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  matplotlib.rcParams["savefig.directory"] = path

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

  if ("CASSIE_STATE_DISPATCHER" in full_log and "CASSIE_STATE_SIMULATION" in full_log):
    compare_ekf(full_log, pos_map, vel_map)

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
  # t_start = 205
  # t_end = 208
  ### Convert times to indices
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  x_wo_spr = np.zeros((t_x.shape[0], plant_wo_spr.num_positions() + plant_wo_spr.num_velocities()))
  for i in range(t_x.shape[0]):
    x_wo_spr[i, :] = np.hstack((pos_map_spr_to_wo_spr @ x[i, :nq], vel_map_spr_to_wo_spr @ x[i, -nv:]))

  # yddot_w_spr = get_yddot(plant_w_spr, context_w_spr, t_x, x, l_toe_frame)
  # yddot_wo_spr = get_yddot(plant_wo_spr, context_wo_spr, t_x, x_wo_spr, l_toe_frame)

  # plt.plot(t_x, yddot_w_spr[:, 0])
  # plt.plot(t_x, yddot_wo_spr[:, 0])
  # plt.plot(t_u[t_u_slice], 500*fsm[t_u_slice])
  # plt.legend(['0', '1', '2', '3', '4', '5', 'fsm'])
  # plt.show()
  ### All plotting scripts here
  # plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes)

  if False:
    plot_contact_est(full_log)
    # plot_feet_positions(plant_w_spr, context, x, l_toe_frame,
    #                     front_contact_disp,
    #                     world, t_x, t_slice, "left_", "_front")
    # plot_feet_positions(plant_w_spr, context, x, r_toe_frame,
    #                     front_contact_disp,
    #                     world, t_x, t_slice, "right_", "_front")
    # plot_feet_positions(plant_w_spr, context, x, l_toe_frame,
    #                     rear_contact_disp,
    #                     world, t_x, t_slice, "left_", "_rear")
    plot_feet_positions(plant_w_spr, context_w_spr, x, r_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "right_", "_rear")

    # plot_feet_positions(plant_wo_spr, context_wo_spr, x, l_toe_frame,
    #                     front_contact_disp,
    #                     world, t_x, t_slice, "left_", "_front")
    # plot_feet_positions(plant_wo_spr, context_wo_spr, x, r_toe_frame,
    #                     front_contact_disp,
    #                     world, t_x, t_slice, "right_", "_front")
    # plot_feet_positions(plant_wo_spr, context_wo_spr, x, l_toe_frame,
    #                     rear_contact_disp,
    #                     world, t_x, t_slice, "left_", "_rear")
    plot_feet_positions(plant_wo_spr, context_wo_spr, x, r_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "right_", "_rear")

  # plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
  yddot_w_spr, JdotV_w_spr, Jvdot_w_spr = get_yddot(plant_w_spr, context_w_spr, t_x, x, r_toe_frame)
  # yddot_wo_spr, JdotV_wo_spr, Jvdot_wo_spr = get_yddot(plant_wo_spr, context_wo_spr, t_x, x_wo_spr, r_toe_frame)
  plt.figure()
  plt.plot(t_x, yddot_w_spr[:, 2],label="yddot")
  plt.plot(t_x, JdotV_w_spr[:, 2],label="jdotv")
  plt.plot(t_x, Jvdot_w_spr[:, 2],label="jvdot")
  plt.legend()
  # plt.plot(t_x, yddot_wo_spr[:, 2])
  plt.show()


def plot_contact_est(log):
  t_contact = []
  t_filtered_contact = []
  t_gm_contact = []
  contact = []
  contact_filtered = []
  gm_contact = []
  for i in range(len(log["CASSIE_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_CONTACT_DISPATCHER"][i]
    t_contact.append(msg.utime / 1e6)
    contact.append(list(msg.contact))
  for i in range(len(log["CASSIE_FILTERED_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_FILTERED_CONTACT_DISPATCHER"][i]
    t_filtered_contact.append(msg.utime / 1e6)
    contact_filtered.append(list(msg.contact))
  for i in range(len(log["CASSIE_GM_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_GM_CONTACT_DISPATCHER"][i]
    t_gm_contact.append(msg.utime / 1e6)
    gm_contact.append(list(msg.contact))
  t_contact = np.array(t_contact)
  t_filtered_contact = np.array(t_filtered_contact)
  t_gm_contact = np.array(t_gm_contact)
  contact = np.array(contact)
  contact_filtered = np.array(contact_filtered)
  gm_contact = np.array(gm_contact)

  plt.figure("Contact estimation")
  # plt.plot(t_contact[t_slice], contact[t_slice], '-')
  plt.plot(t_filtered_contact[t_slice], contact_filtered[t_slice, 0], '-')
  plt.plot(t_gm_contact[t_slice], gm_contact[t_slice, 0], '-')
  plt.legend(["l_contact", "r_contact", "l_contact_filt", "r_contact_filt",
              "l_gm_contact", "r_gm_contact"])


def get_yddot(plant, context, t, x, toe_frame):
  dim = 3
  vdot = np.diff(x[:, -plant.num_velocities():], axis=0)
  dt = np.diff(t, axis=0)
  for i in range(plant.num_velocities()):
    vdot[:, i] = vdot[:, i] / dt

  yddot = np.zeros((t.shape[0], dim))
  JdotV = np.zeros((t.shape[0], dim))
  Jvdot = np.zeros((t.shape[0], dim))
  for i in range(dt.shape[0]):
    plant.SetPositionsAndVelocities(context, x[i, :])
    J = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, np.zeros(3), world, world)
    JdotV_i = plant.CalcBiasTranslationalAcceleration(context, JacobianWrtVariable.kV, toe_frame, np.zeros(3),
                                                           world, world)
    yddot[i, :] = J@vdot[i, :] + JdotV_i[:, 0]
    JdotV[i, :] = JdotV_i[:, 0]  
    Jvdot[i, :] = J@vdot[i, :]  
  filter = 100
  idx = int(filter / 2)
  for i in range(idx, dt.shape[0] - idx):
    yddot[i, :] = np.average(yddot[i - idx:i+idx, :])
    JdotV[i, :] = np.average(JdotV[i - idx:i+idx, :])
    Jvdot[i, :] = np.average(Jvdot[i - idx:i+idx, :])
  return yddot, JdotV, Jvdot

def plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output):
  input_cost = np.zeros(t_u.shape[0])
  acceleration_cost = np.zeros(t_u.shape[0])
  soft_constraint_cost = np.zeros(t_u.shape[0])
  tracking_cost = np.zeros((t_u.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  num_tracking_cost = 0

  for i in range(t_u.shape[0] - 1 - 2):
    # import pdb;
    # pdb.set_trace()
    input_cost[i] = osc_output[i].input_cost
    acceleration_cost[i] = osc_output[i].acceleration_cost
    soft_constraint_cost[i] = osc_output[i].soft_constraint_cost
    for j in range(len(osc_output[i].tracking_data_names)):
      name = osc_output[i].tracking_data_names[j]
      if osc_output[i].tracking_data_names[j] not in tracking_cost_map:
        tracking_cost_map[name] = num_tracking_cost
        num_tracking_cost += 1
      tracking_cost[i, tracking_cost_map[name]] = osc_output[i].tracking_cost[j]

  for name in tracking_cost_map.keys():
    print(name)
    print(tracking_cost_map[name])

  plt.figure("costs")
  plt.plot(t_u[t_u_slice], input_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], acceleration_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))
  osc_traj0 = "swing_ft_traj"
  osc_traj1 = "lipm_traj"
  osc_traj2 = "pelvis_balance_traj"
  osc_traj3 = "swing_hip_yaw_traj"

  #
  # plot_osc(osc_debug, osc_traj0, 0, "pos")
  # plot_osc(osc_debug, osc_traj0, 1, "pos")
  # plot_osc(osc_debug, osc_traj0, 2, "pos")
  # #
  # plot_osc(osc_debug, osc_traj0, 0, "vel")
  # plot_osc(osc_debug, osc_traj0, 1, "vel")
  # plot_osc(osc_debug, osc_traj0, 2, "vel")
  # #
  plot_osc(osc_debug, osc_traj0, 2, "accel")
  plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  # plot_osc(osc_debug, osc_traj0, 1, "accel")
  # plot_osc(osc_debug, osc_traj0, 2, "accel")
  #
  # plot_osc(osc_debug, osc_traj3, 0, "accel")
  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  # plot_osc(osc_debug, osc_traj3, 0, "vel")


def plot_osc(osc_debug, osc_traj, dim, derivative):
  fig = plt.figure(osc_traj + " " + derivative + " tracking " + str(dim))
  if (derivative == "pos"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_y[t_u_slice, dim])
    plt.legend(["y_des", "y", "error_y"])
    # plt.legend(["y_des", "y"])
  elif (derivative == "vel"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim])
    plt.legend(["ydot_des", "ydot", "error_ydot"])
  elif (derivative == "accel"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command_sol[t_u_slice, dim])
    plt.legend(["yddot_des", "yddot_command", "yddot_command_sol"])


def plot_feet_positions(plant, context, x, toe_frame, contact_point, world,
                        t_x, t_x_slice, foot_type, contact_type):
  foot_x = np.zeros((6, t_x.size))
  for i in range(t_x.size):
    x_i = x[i, :]
    if (plant.num_positions() != nq):
      x_i = np.hstack((pos_map_spr_to_wo_spr @ x[i, :nq], vel_map_spr_to_wo_spr @ x[i, -nv:]))
      # plant is fixed spring
    plant.SetPositionsAndVelocities(context, x_i)
    foot_x[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                 contact_point, world)
    foot_x[3:6, i] = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, contact_point,
      world,
      world) @ x_i[-plant.num_velocities():]
  fig = plt.figure('foot pos: ' + filename)
  # state_indices = slice(0, 3)
  state_indices = slice(5, 6)
  # state_indices = slice(5, 6)
  state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
  state_names = [foot_type + name for name in state_names]
  state_names = [name + contact_type for name in state_names]
  plt.plot(t_x[t_x_slice], foot_x.T[t_x_slice, state_indices],
           label=state_names[state_indices])
  plt.legend()


def compare_ekf(log, pos_map, vel_map):
  t_x = []
  t_x_est = []
  q = []
  v = []
  imu = []
  q_est = []
  v_est = []
  for i in range(len(log["CASSIE_STATE_SIMULATION"])):
    msg = log["CASSIE_STATE_SIMULATION"][i]
    q_temp = [[] for i in range(len(msg.position))]
    v_temp = [[] for i in range(len(msg.velocity))]
    for i in range(len(q_temp)):
      q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
    for i in range(len(v_temp)):
      v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
    q.append(q_temp)
    v.append(v_temp)
    imu.append(msg.imu_accel)
    t_x.append(msg.utime / 1e6)
  for i in range(len(log["CASSIE_STATE_DISPATCHER"])):
    msg = log["CASSIE_STATE_DISPATCHER"][i]
    q_temp = [[] for i in range(len(msg.position))]
    v_temp = [[] for i in range(len(msg.velocity))]
    for i in range(len(q_temp)):
      q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
    for i in range(len(v_temp)):
      v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
    q_est.append(q_temp)
    v_est.append(v_temp)
    t_x_est.append(msg.utime / 1e6)
  t_x = np.array(t_x)
  t_x_est = np.array(t_x_est)
  q = np.array(q)
  v = np.array(v)
  imu = np.array(imu)
  q_est = np.array(q_est)
  v_est = np.array(v_est)

  pos_indices = slice(4, 7)
  vel_indices = slice(4, 5)
  plt.figure("EKF positions: " + filename)
  plt.plot(t_x, q[:, pos_indices], '-')
  plt.plot(t_x_est, q_est[:, pos_indices], '--')
  plt.figure("EKF velocities: " + filename)
  plt.plot(t_x, v[:, vel_indices], '-')
  plt.plot(t_x_est, v_est[:, vel_indices], '--')
  plt.figure("IMU: " + filename)
  plt.plot(t_x, imu, 'k-')


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes):
  pos_indices = slice(0 + 7, 23, 2)
  vel_indices = slice(23 + 6, 45, 2)
  u_indices = slice(6, 8)
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)

  plt.figure("positions: " + filename)
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["knee_joint_right"]])
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["ankle_spring_joint_right"]])
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts: " + filename)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.figure("efforts meas: " + filename)
  # plt.figure("Delay characterization")
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  # plt.legend(u_datatypes[u_indices])


if __name__ == "__main__":
  main()
