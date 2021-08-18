import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.lcm import lcm_trajectory
from pydairlib.lcm import process_lcm_log
import pydairlib.multibody
from pydairlib.multibody.kinematic import DistanceEvaluator
from impact_invariant_scripts import plot_ii_projection
from pydairlib.cassie.cassie_utils import *

from pydairlib.common import FindResourceOrThrow
from pydairlib.common import plot_styler

def main():
  global filename
  global t_start
  global t_end
  global t_slice
  global t_u_slice
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
  global nominal_impact_time

  load_maps()

  # figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/data/'
  ps = plot_styler.PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  builder = DiagramBuilder()
  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   # "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)


  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_wo_spr, scene_graph_wo_spr, True,
                                                   "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)
  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  context_w_spr = plant_w_spr.CreateDefaultContext()
  context_wo_spr = plant_wo_spr.CreateDefaultContext()

  # Reference trajectory
  delay_time = 2.0
  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  jumping_traj = lcm_trajectory.DirconTrajectory(filename)
  state_traj = jumping_traj.ReconstructStateTrajectory()
  input_traj = jumping_traj.ReconstructInputTrajectory()
  input_traj.shiftRight(delay_time)

  nominal_impact_time = jumping_traj.GetStateBreaks(2)[0]
  import pdb; pdb.set_trace()

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  l_toe_frame = plant_w_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_w_spr.GetBodyByName("toe_right").body_frame()
  world = plant_w_spr.world_frame()

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
  log_num = filename.split('-')[-1]

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log, t_lcmlog_u = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

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
  # t_start = 30.645
  # t_end = t_start + 0.08
  ### Convert times to indices
  t_slice = slice(np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0])
  t_u_slice = slice(np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0])


  # All plotting scripts here
  # plot_status(full_log)
  # plot_ekf(full_log, pos_map, vel_map)
  # plot_control_rate(t_u, u)
  # plot_ii_projection(ps, t_x, x, plant_w_spr, context_w_spr, t_slice, pos_map_spr_to_wo_spr, vel_map_spr_to_wo_spr, '-', log_num, u_meas)
  plot_ii_projection(ps, t_x, x, plant_wo_spr, context_wo_spr, t_slice, pos_map_spr_to_wo_spr, vel_map_spr_to_wo_spr, '-', log_num, u_meas)
  # plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas)
  # plot_contact_est(full_log)

  if False:
    # front_contact_disp = np.zeros(3)
    plot_feet_positions(plant_w_spr, context_w_spr, x, l_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "left_", "_front")
    plot_feet_positions(plant_w_spr, context_w_spr, x, r_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "right_", "_front")
    plot_feet_positions(plant_w_spr, context_w_spr, x, l_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "left_", "_rear")
    plot_feet_positions(plant_w_spr, context_w_spr, x, r_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "right_", "_rear")

  # plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
  # plot_id_debug(t_u, osc_debug, osc_output)
  plt.show()

def plot_contact_est(log):
  t_contact = []
  contact = []
  hardware_impact = 30.0 + nominal_impact_time + 0.09

  contact = np.zeros((len(log["CASSIE_GM_CONTACT_DISPATCHER"]), 2))
  for i in range(len(log["CASSIE_GM_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_GM_CONTACT_DISPATCHER"][i]
    t_contact.append(msg.timestamp / 1e6)
    for j in range(msg.num_point_pair_contacts):
      contact[i][j] = msg.point_pair_contact_info[j].contact_force[2]

  t_contact = np.array(t_contact)
  contact = np.array(contact)
  plt.figure("Contact estimation: " + filename)
  ps.plot(t_contact[t_slice] - hardware_impact, contact[t_slice, 0], xlabel='Time Since Nominal Impact (s)', ylabel='Estimated Normal Contact Force (N)')
  ps.plot(t_contact[t_slice] - hardware_impact, contact[t_slice, 1])
  ps.add_legend(["Left Foot", "Right Foot"])
  # ps.save_fig('hardware_contact_est_' + filename + '.png')

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

  plt.figure("qp_solve_time:" + filename)
  ps.plot(t_u[t_u_slice], qp_solve_time[t_u_slice])
  # plt.hist(qp_solve_time[t_u_slice], range=[0, 0.003])

  plt.figure("costs")
  ps.plot(t_u[t_u_slice], input_cost[t_u_slice], color=ps.grey)
  ps.plot(t_u[t_u_slice], acceleration_cost[t_u_slice], color=ps.blue)
  ps.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice], color=ps.red)
  ps.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))

  if(controller_channel == 'OSC_JUMPING' or controller_channel == 'CASSIE_INPUT'):
    # For Jumping
    osc_traj0 = "com_traj"
    osc_traj1 = "left_ft_traj"
    osc_traj2 = "right_ft_traj"
    osc_traj3 = "left_toe_angle_traj"
    osc_traj4 = "right_toe_angle_traj"
    osc_traj5 = "swing_hip_yaw_left_traj"
    osc_traj6 = "swing_hip_yaw_right_traj"
    osc_traj7 = "pelvis_rot_tracking_data"
  elif(controller_channel == 'OSC_WALKING'):
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
  elif(controller_channel == 'OSC_RUNNING'):
    # For Walking
    osc_traj0 = "pelvis_trans_traj"
    osc_traj1 = "left_ft_traj"
    osc_traj2 = "right_ft_traj"
    osc_traj3 = "hip_pitch_left_traj"
    osc_traj4 = "hip_pitch_right_traj"
    osc_traj5 = "hip_roll_left_traj"
    osc_traj6 = "hip_roll_right_traj"
    osc_traj7 = "left_toe_angle_traj"
    osc_traj8 = "right_toe_angle_traj"
    osc_traj9 = "hip_yaw_left_traj"
    osc_traj10 = "hip_yaw_right_traj"
  elif(controller_channel == 'OSC_STANDING'):
    osc_traj0 = "com_traj"
    osc_traj1 = "pelvis_rot_traj"
    osc_traj2 = "hip_yaw_left_traj"
    osc_traj3 = "hip_yaw_right_traj"

  #
  plot_osc(osc_debug, osc_traj0, 0, "pos")
  plot_osc(osc_debug, osc_traj0, 1, "pos")
  plot_osc(osc_debug, osc_traj0, 2, "pos")


  #
  # plot_osc(osc_debug, osc_traj0, 0, "vel")
  # plot_osc(osc_debug, osc_traj0, 1, "vel")
  # plot_osc(osc_debug, osc_traj0, 2, "vel")

  #
  # plot_osc(osc_debug, osc_traj0, 0, "acc")
  # plot_osc(osc_debug, osc_traj0, 1, "acc")
  # plot_osc(osc_debug, osc_traj0, 2, "acc")

  # plot_osc(osc_debug, osc_traj1, 0, "pos")
  # plot_osc(osc_debug, osc_traj1, 1, "pos")
  # plot_osc(osc_debug, osc_traj1, 2, "pos")
  # #
  # plot_osc(osc_debug, osc_traj2, 0, "pos")
  # plot_osc(osc_debug, osc_traj2, 1, "pos")
  # plot_osc(osc_debug, osc_traj2, 2, "pos")
  # ps.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  # plot_osc(osc_debug, osc_traj2, 0, "vel")
  # plot_osc(osc_debug, osc_traj2, 1, "vel")
  # plot_osc(osc_debug, osc_traj2, 2, "vel")

  # plot_osc(osc_debug, osc_traj1, 0, "acc")
  # plot_osc(osc_debug, osc_traj1, 1, "acc")
  # plot_osc(osc_debug, osc_traj1, 2, "acc")

  # plot_osc(osc_debug, osc_traj2, 0, "acc")
  # plot_osc(osc_debug, osc_traj2, 1, "acc")
  # plot_osc(osc_debug, osc_traj2, 2, "acc")

  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # ps.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])
  # plot_osc(osc_debug, osc_traj4, 0, "pos")
  # ps.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plot_osc(osc_debug, osc_traj4, 0, "pos")
  # plot_osc(osc_debug, osc_traj5, 0, "pos")
  # plot_osc(osc_debug, osc_traj6, 0, "pos")
  # plot_osc(osc_debug, osc_traj5, 0, "vel")
  # plot_osc(osc_debug, osc_traj6, 0, "vel")
  # plot_osc(osc_debug, osc_traj5, 0, "acc")
  # plot_osc(osc_debug, osc_traj6, 0, "acc")
  # plot_osc(osc_debug, osc_traj7, 0, "pos")
  # plot_osc(osc_debug, osc_traj8, 0, "pos")
  # plot_osc(osc_debug, osc_traj9, 0, "pos")
  # plot_osc(osc_debug, osc_traj10, 0, "pos")


  # ps.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

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
  ps.plot(t_u[t_u_slice], input_cost[t_u_slice])
  ps.plot(t_u[t_u_slice], acceleration_cost[t_u_slice])
  ps.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice])
  ps.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
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
  plot_osc(osc_debug, osc_traj0, 0, "pos")
  plot_osc(osc_debug, osc_traj1, 0, "pos")
  plot_osc(osc_debug, osc_traj2, 0, "pos")
  plot_osc(osc_debug, osc_traj3, 0, "pos")
  plot_osc(osc_debug, osc_traj4, 0, "pos")
  plot_osc(osc_debug, osc_traj5, 0, "pos")
  plot_osc(osc_debug, osc_traj6, 0, "pos")
  plot_osc(osc_debug, osc_traj7, 0, "pos")
  plot_osc(osc_debug, osc_traj8, 0, "pos")
  plot_osc(osc_debug, osc_traj9, 0, "pos")
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
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim], color=ps.blue)
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot[t_u_slice, dim], color=ps.red)
    ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim] - osc_debug[osc_traj].ydot[t_u_slice, dim], color=ps.yellow)
    # ps.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim], color=ps.grey)
    ps.add_legend(["ydot_des", "ydot", "error_ydot", "projected_error_ydot"])
    # plt.legend(["error_ydot", "corrected_error"])
  elif (derivative == "acc"):
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
  fig = plt.figure('foot pos: ' + filename)

  # state_indices = slice(4, 5)
  # state_indices = slice(2, 3)
  state_indices = slice(5, 6)
  state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
  state_names = [foot_type + name for name in state_names]
  state_names = [name + contact_type for name in state_names]
  ps.plot(t_x[t_x_slice], foot_x.T[t_x_slice, state_indices],
           data_label=state_names[state_indices])
  plt.legend()


def plot_ekf(log, pos_map, vel_map):
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
    # imu.append(msg.imu_accel)
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
    imu.append(msg.imu_accel)
    t_x_est.append(msg.utime / 1e6)

  t_x = np.array(t_x)
  t_x_est = np.array(t_x_est)
  q = np.array(q)
  v = np.array(v)
  imu = np.array(imu)
  q_est = np.array(q_est)
  v_est = np.array(v_est)

  pos_indices = slice(4, 7)
  vel_indices = slice(3, 6)
  plt.figure("EKF positions: " + filename)
  ps.plot(t_x_est[t_slice], q[t_slice, pos_indices])
  ps.plot(t_x_est[t_slice], q_est[t_slice, pos_indices])
  ps.add_legend(['ground_truth', 'EKF'])
  plt.figure("EKF velocities: " + filename)
  ps.plot(t_x_est[t_slice], v[t_slice, vel_indices])
  ps.plot(t_x_est[t_slice], v_est[t_slice, vel_indices])
  ps.add_legend(['ground_truth', 'EKF'])
  plt.figure("IMU: " + filename)
  ps.plot(t_x_est[t_slice], imu[t_slice,:])

def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas):
  # non-floating base states
  pos_indices = slice(0 + 7, 11)
  vel_indices = slice(23 + 6, 45)
  # floating base states
  pos_indices = slice(0, 7)
  # vel_indices = slice(23, 23 + 6)
  # vel_indices = slice(23 + 6, 23 + 10)
  # all motor torques
  u_indices = slice(0, 10)

  plt.figure("positions: " + filename)
  ps.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])

  plt.figure("velocities: " + filename)
  ps.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  #
  plt.figure("efforts: " + filename)
  ps.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  # ps.plot(t_x[t_slice], u_meas[t_slice, u_indices], linestyle='--')
  plt.legend(u_datatypes[u_indices] + u_datatypes[u_indices])

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
  ps.plot(t_status, status)
  ps.plot(t_status, shutdown)
  ps.plot(t_status, vel_limit)
  ps.plot(t_status, act_limit)
  ps.plot(t_status, act_delay)
  plt.legend(['status',
              'shutdown',
              'vel_limit',
              'act_limit',
              'act_delay'])

def plot_control_rate(t_u, u):
  print(np.average(np.diff(t_u[t_u_slice])))
  plt.plot(t_u[t_u_slice], np.diff(t_u[t_u_slice], prepend=30), '.')

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
