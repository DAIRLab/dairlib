import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
import process_lcm_log
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.cassie.cassie_utils import *

from pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.parameter_studies.plot_styler import PlotStyler


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
  global controller_channel
  global td_time
  global ps
  global pos_map_spr_to_wo_spr
  global vel_map_spr_to_wo_spr

  load_maps()

  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  builder = DiagramBuilder()
  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_wo_spr, scene_graph_wo_spr, True,
                                                   "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)
  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  # Reference trajectory
  delay_time = 2.0
  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  jumping_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)
  input_traj = jumping_traj.ReconstructInputTrajectory()
  input_traj.shiftRight(delay_time)

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  l_toe_frame = plant_w_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_w_spr.GetBodyByName("toe_right").body_frame()
  world = plant_w_spr.world_frame()
  context = plant_w_spr.CreateDefaultContext()
  context_wo_spr = plant_wo_spr.CreateDefaultContext()

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
  matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]
  font = {'size'   : 18}
  matplotlib.rc('font', **font)
  matplotlib.rcParams['lines.linewidth'] = 4

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log, t_lcmlog_u = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

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

  # Nominal touchdown time
  td_time = jumping_traj.GetStateBreaks(2)[0] + delay_time
  # Default time window values, can override
  t_start = t_u[10]
  t_end = t_u[-10]
  # Override here #
  t_start = 30.635
  t_end = 30.68
  ### Convert times to indices
  t_slice = slice(np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0])
  t_u_slice = slice(np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0])

  # t_lcmlog_u -= t_lcmlog_u[0] - t_u[0]
  # plt.plot(t_u[:-1], np.diff(t_u))
  # plt.plot(t_x[:-1], np.diff(t_x))
  # plt.plot(t_u[:-1], np.diff(t_lcmlog_u))
  # plt.figure()
  # plt.plot(t_u, t_lcmlog_u - t_u)
  # plt.show()

  ### All plotting scripts here


  # plt.figure("Impact_Event")
  # t_u_slice = slice(np.argwhere(np.abs(osc_debug['com_traj'].t - (td_time)) < 1e-3)[0][0], np.argwhere(np.abs(t_u - (td_time + 0.2)) < 1e-3)[0][0])
  # plot_osc(osc_debug, 'com_traj', 2, "vel")
  # t_u_slice = slice(np.argwhere(np.abs(osc_debug['left_ft_traj'].t - (td_time - 0.2)) < 1e-3)[0][0], np.argwhere(np.abs(t_u - (td_time)) < 1e-3)[0][0])
  # plot_osc(osc_debug, 'left_ft_traj', 2, "vel")

  # plot_status(full_log)
  plot_ii_projection(t_x, x, plant_w_spr, context)
  # plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas)

  # u_nominal = np.zeros((t_u[t_u_slice].shape[0], nu))
  # for t in range(t_u[t_u_slice].shape[0]):
  #   u_nominal[t] = input_traj.value(t_u[t_u_slice][t])[:,0]
  # plt.plot(t_u[t_u_slice], np.sum(u_nominal[:, 6:8], axis=1))

  # plot_contact_est(full_log)\
  # plt.figure("Impact Event")
  # plt.plot(t_contact_info[t_slice], (contact_info[0, t_slice, s2] + contact_info[3, t_slice, 2]), 'r-')
  # plt.plot([td_time, td_time], [0, 10])
  # plt.legend(['left foot', 'right foot'])
  # plt.plot(t_u[t_u_slice], 100 * fsm[t_u_slice], 'k')

  if False:
    plot_feet_positions(plant_w_spr, context, x, l_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "left_", "_front")
    plot_feet_positions(plant_w_spr, context, x, r_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "right_", "_front")
    plot_feet_positions(plant_w_spr, context, x, l_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "left_", "_rear")
    plot_feet_positions(plant_w_spr, context, x, r_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "right_", "_rear")
    # plt.plot(t_u[t_u_slice], 0.025*fsm[t_u_slice])

  # plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
  # plot_id_debug(t_u, osc_debug, osc_output)
  plt.show()

def plot_contact_est(log):
  t_contact = []
  contact = []
  t_filtered_contact = []
  contact_filtered = []
  t_gm_contact = []
  gm_contact_l = []
  gm_contact_r = []
  t_fsm_contact = []
  fsm_contact = np.zeros((len(log["CASSIE_CONTACT_FOR_FSM_DISPATCHER"]), 2))
  for i in range(len(log["CASSIE_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_CONTACT_DISPATCHER"][i]
    t_contact.append(msg.utime / 1e6)
    contact.append(list(msg.contact))
  for i in range(len(log["CASSIE_FILTERED_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_FILTERED_CONTACT_DISPATCHER"][i]
    t_filtered_contact.append(msg.utime / 1e6)
    contact_filtered.append(list(msg.contact))
  for i in range(len(log["CASSIE_CONTACT_FOR_FSM_DISPATCHER"])):
    msg = log["CASSIE_CONTACT_FOR_FSM_DISPATCHER"][i]
    t_fsm_contact.append(msg.timestamp / 1e6)
    for j in range(msg.num_point_pair_contacts):
      fsm_contact[i][j] = msg.point_pair_contact_info[j].contact_force[2]

  t_contact = np.array(t_contact)
  contact = np.array(contact)
  t_filtered_contact = np.array(t_filtered_contact)
  contact_filtered = np.array(contact_filtered)
  t_gm_contact = np.array(t_gm_contact)
  gm_contact_l = np.array(gm_contact_l)
  gm_contact_r = np.array(gm_contact_r)
  plt.figure("Contact estimation")
  # plt.plot(t_contact[t_slice], contact[t_slice], '-')
  # plt.plot(t_filtered_contact[t_slice], contact_filtered[t_slice, 0], '-')
  plt.plot(t_gm_contact[t_slice], gm_contact_l[t_slice, 2], 'b--')
  plt.plot(t_gm_contact[t_slice], gm_contact_r[t_slice, 2], 'r--')
  # plt.plot(t_fsm_contact, fsm_contact, 'k-')
  plt.legend(["l_contact", "r_contact", "l_contact_filt", "r_contact_filt"])


def plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output):
  input_cost = np.zeros(t_u.shape[0])
  acceleration_cost = np.zeros(t_u.shape[0])
  soft_constraint_cost = np.zeros(t_u.shape[0])
  tracking_cost = np.zeros((t_u.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  qp_solve_time = np.zeros(t_u.shape[0])
  num_tracking_cost = 0
  for i in range(t_u.shape[0] - 10):
    input_cost[i] = osc_output[i].input_cost
    acceleration_cost[i] = osc_output[i].acceleration_cost
    soft_constraint_cost[i] = osc_output[i].soft_constraint_cost
    qp_solve_time[i] = osc_output[i].qp_output.solve_time
    for j in range(len(osc_output[i].tracking_data_names)):
      name = osc_output[i].tracking_data_names[j]
      if osc_output[i].tracking_data_names[j] not in tracking_cost_map:
        tracking_cost_map[name] = num_tracking_cost
        num_tracking_cost += 1
      tracking_cost[i, tracking_cost_map[name]] = osc_output[i].tracking_cost[j]

  for name in tracking_cost_map.keys():
    print(name)
    print(tracking_cost_map[name])

  plt.figure("qp solve time")
  plt.plot(t_u[t_u_slice], qp_solve_time[t_u_slice])

  plt.figure("costs")
  plt.plot(t_u[t_u_slice], input_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], acceleration_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))

  if(controller_channel == 'OSC_JUMPING'):
    # For Jumping
    osc_traj0 = "com_traj"
    osc_traj1 = "left_ft_traj"
    osc_traj2 = "right_ft_traj"
    osc_traj3 = "left_toe_angle_traj"
    osc_traj4 = "right_toe_angle_traj"
    osc_traj5 = "swing_hip_yaw_left_traj"
  else:
    # For Walking
    osc_traj0 = "lipm_traj"
    osc_traj3 = "left_toe_angle_traj"
    osc_traj4 = "right_toe_angle_traj"
    if 'right_foot_traj' in osc_debug:
      osc_traj1 = "left_foot_traj"
      osc_traj2 = "right_foot_traj"
      osc_traj5 = "pelvis_balance_traj"
    else:
      osc_traj1 = "swing_ft_traj"
      osc_traj2 = "swing_ft_traj"
      osc_traj5 = "pelvis_balance_traj"


  #
  # plot_osc(osc_debug, osc_traj0, 0, "pos")
  # plot_osc(osc_debug, osc_traj0, 1, "pos")
  # plot_osc(osc_debug, osc_traj0, 2, "pos")

  #
  # plot_osc(osc_debug, osc_traj0, 0, "vel")
  # plot_osc(osc_debug, osc_traj0, 1, "vel")
  plot_osc(osc_debug, osc_traj0, 2, "vel")

  #
  # plot_osc(osc_debug, osc_traj0, 0, "accel")
  # plot_osc(osc_debug, osc_traj0, 1, "accel")
  # plot_osc(osc_debug, osc_traj0, 2, "accel")

  # plot_osc(osc_debug, osc_traj1, 0, "pos")
  # plot_osc(osc_debug, osc_traj1, 1, "pos")
  # plot_osc(osc_debug, osc_traj1, 2, "pos")
  plot_osc(osc_debug, osc_traj1, 2, "vel")
  # plot_osc(osc_debug, osc_traj2, 0, "pos")
  # plot_osc(osc_debug, osc_traj2, 1, "pos")
  # plot_osc(osc_debug, osc_traj2, 2, "pos")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  # plot_osc(osc_debug, osc_traj2, 0, "vel")
  # plot_osc(osc_debug, osc_traj2, 1, "vel")
  plot_osc(osc_debug, osc_traj2, 2, "vel")

  # plot_osc(osc_debug, osc_traj1, 0, "accel")
  # plot_osc(osc_debug, osc_traj1, 1, "accel")
  # plot_osc(osc_debug, osc_traj1, 2, "accel")

  # plot_osc(osc_debug, osc_traj2, 0, "accel")
  # plot_osc(osc_debug, osc_traj2, 1, "accel")
  # plot_osc(osc_debug, osc_traj2, 2, "accel")

  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])
  # plot_osc(osc_debug, osc_traj4, 0, "pos")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plot_osc(osc_debug, osc_traj5, 0, "vel")
  # plot_osc(osc_debug, osc_traj5, 0, "accel")
  # plot_osc(osc_debug, osc_traj5, 1, "accel")
  # plot_osc(osc_debug, osc_traj5, 2, "accel")
  # plot_osc(osc_debug, osc_traj5, 0, "vel")


  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

def plot_id_debug(t_u, osc_debug, osc_output):
  input_cost = np.zeros(t_u.shape[0])
  acceleration_cost = np.zeros(t_u.shape[0])
  soft_constraint_cost = np.zeros(t_u.shape[0])
  tracking_cost = np.zeros((t_u.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  qp_solve_time = np.zeros(t_u.shape[0])
  num_tracking_cost = 0
  for i in range(t_u.shape[0] - 10):
    input_cost[i] = osc_output[i].input_cost
    acceleration_cost[i] = osc_output[i].acceleration_cost
    soft_constraint_cost[i] = osc_output[i].soft_constraint_cost
    qp_solve_time[i] = osc_output[i].qp_output.solve_time
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

  osc_traj0 = 'hip_roll_left_traj'
  osc_traj1 = 'hip_roll_right_traj'
  osc_traj2 = 'hip_yaw_left_traj'
  osc_traj3 = 'hip_yaw_right_traj'
  osc_traj4 = 'hip_pitch_left_traj'
  osc_traj5 = 'hip_pitch_right_traj'
  osc_traj6 = 'knee_left_traj'
  osc_traj7 = 'knee_right_traj'
  osc_traj8 = 'toe_left_traj'
  osc_traj9 = 'toe_right_traj'

  # plot_osc(osc_debug, osc_traj2, 0, "pos")
  # plot_osc(osc_debug, osc_traj2, 1, "pos")
  # plot_osc(osc_debug, osc_traj2, 2, "pos")
  #
  # plot_osc(osc_debug, osc_traj0, 0, "pos")
  # plot_osc(osc_debug, osc_traj1, 0, "pos")
  # plot_osc(osc_debug, osc_traj2, 0, "pos")
  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plot_osc(osc_debug, osc_traj4, 0, "pos")
  # plot_osc(osc_debug, osc_traj5, 0, "pos")
  # plot_osc(osc_debug, osc_traj6, 0, "pos")
  # plot_osc(osc_debug, osc_traj7, 0, "pos")
  # plot_osc(osc_debug, osc_traj8, 0, "pos")
  # plot_osc(osc_debug, osc_traj9, 0, "pos")

  #
  plot_osc(osc_debug, osc_traj0, 0, "vel")
  plot_osc(osc_debug, osc_traj1, 0, "vel")
  plot_osc(osc_debug, osc_traj2, 0, "vel")
  plot_osc(osc_debug, osc_traj3, 0, "vel")
  plot_osc(osc_debug, osc_traj4, 0, "vel")
  plot_osc(osc_debug, osc_traj5, 0, "vel")
  plot_osc(osc_debug, osc_traj6, 0, "vel")
  plot_osc(osc_debug, osc_traj7, 0, "vel")
  plot_osc(osc_debug, osc_traj8, 0, "vel")
  plot_osc(osc_debug, osc_traj9, 0, "vel")
  #


  # plot_osc(osc_debug, osc_traj0, 0, "acc")
  # plot_osc(osc_debug, osc_traj1, 0, "acc")
  # plot_osc(osc_debug, osc_traj2, 0, "acc")
  # plot_osc(osc_debug, osc_traj3, 0, "acc")
  # plot_osc(osc_debug, osc_traj4, 0, "acc")
  # plot_osc(osc_debug, osc_traj5, 0, "acc")
  # plot_osc(osc_debug, osc_traj6, 0, "acc")
  # plot_osc(osc_debug, osc_traj7, 0, "acc")
  # plot_osc(osc_debug, osc_traj8, 0, "acc")
  # plot_osc(osc_debug, osc_traj9, 0, "acc")


def plot_osc(osc_debug, osc_traj, dim, derivative):
  fig = plt.figure(osc_traj + " " + derivative + " tracking " + str(dim))
  if (derivative == "pos"):
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y_des[t_u_slice, dim])
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y[t_u_slice, dim])
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_y[t_u_slice, dim])
    ps.add_legend(["y_des", "y", "error_y"])
    # plt.legend(["y_des", "y"])
  elif (derivative == "vel"):
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim], linestyle=ps.blue)
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot[t_u_slice, dim], linestyle=ps.red)
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim] - osc_debug[osc_traj].ydot[t_u_slice, dim], linestyle=ps.yellow)
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim], linestyle=ps.grey)
    ps.add_legend(["ydot_des", "ydot", "error_ydot", "projected_error_ydot"])
    # plt.legend(["error_ydot", "corrected_error"])
  elif (derivative == "accel"):
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_des[t_u_slice, dim])
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command[t_u_slice, dim])
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command_sol[t_u_slice, dim])
    ps.add_legend(["yddot_des", "yddot_command", "yddot_command_sol"])


def plot_feet_positions(plant, context, x, toe_frame, contact_point, world,
                        t_x, t_x_slice, foot_type, contact_type):
  foot_x = np.zeros((6, t_x.size))
  for i in range(t_x.size):
    plant.SetPositionsAndVelocities(context, x[i, :])
    foot_x[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                 contact_point, world)
    foot_x[3:6, i] = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, contact_point,
      world,
      world) @ x[i, -nv:]
  # fig = plt.figure('foot pos: ' + filename)
  # plt.figure("Impact Event")

  # state_indices = slice(4, 5)
  # state_indices = slice(2, 3)
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
  plt.plot(t_x_est, q_est[:, pos_indices])
  plt.figure("EKF velocities: " + filename)
  plt.plot(t_x, v[:, vel_indices], '-')
  plt.plot(t_x_est, v_est[:, vel_indices])
  plt.figure("IMU: " + filename)
  plt.plot(t_x, imu, 'k-')


def calc_loop_closure_jacobian(plant, context, x_pre):
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  heel_disp = np.array((.11877, -.01, 0.0))
  left_thigh_disp = np.array((0.0, 0.0, 0.045))
  right_thigh_disp = np.array((0.0, 0.0, -0.045))
  world = plant.world_frame()

  l_thigh_frame = plant.GetBodyByName("thigh_left").body_frame()
  r_thigh_frame = plant.GetBodyByName("thigh_right").body_frame()
  l_heel_frame = plant.GetBodyByName("heel_spring_left").body_frame()
  r_heel_frame = plant.GetBodyByName("heel_spring_right").body_frame()

  left_heel = plant.CalcPointsPositions(context, l_heel_frame, heel_disp, world)
  left_thigh = plant.CalcPointsPositions(context, l_thigh_frame, left_thigh_disp, world)
  right_heel = plant.CalcPointsPositions(context, r_heel_frame, heel_disp, world)
  right_thigh = plant.CalcPointsPositions(context, r_thigh_frame, right_thigh_disp, world)
  left_rel_pos = left_heel - left_thigh
  right_rel_pos = right_heel - right_thigh

  J_l_heel = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_heel_frame, heel_disp, world, world)
  J_l_thigh = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_thigh_frame, left_thigh_disp, world, world)
  J_r_heel = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_heel_frame, heel_disp, world, world)
  J_r_thigh = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_thigh_frame, right_thigh_disp, world, world)
  J_l_loop = (left_rel_pos.transpose() @ (J_l_heel - J_l_thigh)) / np.linalg.norm(left_rel_pos)
  J_r_loop = (right_rel_pos.transpose() @ (J_r_heel - J_r_thigh)) / np.linalg.norm(right_rel_pos)

  return J_l_loop, J_r_loop

def plot_ii_projection(t_x, x, plant, context):

  t_pre = 30.557
  t_idx = np.argwhere(np.abs(t_x - t_pre) < 1e-3)[0][0]
  x_pre = x[t_idx]

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  world = plant.world_frame()

  # x_wo_spr = np.vstack((pos_map_spr_to_wo_spr @ x[:, :nq].T, vel_map_spr_to_wo_spr @ x[:, -nv:].T))
  plant.SetPositionsAndVelocities(context, x_pre)
  M = plant.CalcMassMatrixViaInverseDynamics(context)
  M_inv = np.linalg.inv(M)
  J_l_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
  J_l_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
  J_r_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)
  J_r_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
  J_l_loop, J_r_loop = calc_loop_closure_jacobian(plant, context, x_pre)
  J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r, J_l_loop, J_r_loop))
  # J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r))
  M_Jt = M_inv @ J.T
  proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
  P = linalg.null_space(M_Jt.T).T

  proj_vel = P @ x[t_slice, -nv:].T
  # proj_vel = P @ x[t_slice, -nv:].T

  plt.figure("joint velocities")
  ps.plot(t_x[t_slice], x[t_slice, -nv:])
  plt.ylim([-10, 10])
  plt.figure("projected velocities")
  ps.plot(t_x[t_slice], proj_vel.T)
  plt.ylim([-10, 10])

def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas):
  # pos_indices = slice(0 + 7, 23)
  vel_indices = slice(23 + 6, 45)
  pos_indices = slice(0, 7)
  # vel_indices = slice(23, 23 + 6)
  u_indices = slice(0, 6)
  # overwrite

  plt.figure("positions: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])

  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])

  u_indices = slice(6, 8)
  # plt.figure("Combined knee motor efforts: " + filename)
  plt.figure("Impact Event")
  plt.plot(t_u[t_u_slice], np.sum(u[t_u_slice, u_indices], axis=1))
  plt.legend(u_datatypes[u_indices])
  # u_indices = slice(8, 10)
  # plt.figure("efforts 8-9: " + filename)
  # plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  # plt.legend(u_datatypes[u_indices])


  # plt.ylim(-50, 300)

  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices], '--')
  # plt.legend(u_datatypes[u_indices])
  # plt.figure("efforts meas: " + filename)
  # plt.figure("Delay characterization")
  # plt.legend(u_datatypes[u_indices])


def plot_status(full_log):
  t_status = []
  status = []
  shutdown = []
  vel_limit = []
  act_limit = []
  act_delay = []
  for log in full_log['INPUT_SUPERVISOR_STATUS']:
    t_status.append(log.utime / 1e6)
    status.append(log.status)
    shutdown.append(log.shutdown)
    vel_limit.append(log.vel_limit)
    act_limit.append(log.act_limit)
    act_delay.append(log.act_delay)
  t_status = np.array(t_status)
  status = np.array(status)
  plt.figure("Input Supervisor Status")
  plt.plot(t_status, status)
  plt.plot(t_status, shutdown)
  plt.plot(t_status, vel_limit)
  plt.plot(t_status, act_limit)
  plt.plot(t_status, act_delay)
  plt.legend(['status',
              'shutdown',
              'vel_limit',
              'act_limit',
              'act_delay'])

def load_maps():
  global pos_map_spr_to_wo_spr
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
  global vel_map_spr_to_wo_spr
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

if __name__ == "__main__":
  main()
