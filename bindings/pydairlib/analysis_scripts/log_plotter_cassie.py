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
from pydairlib.cassie.cassie_utils import *

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

  # Default time window values, can override
  t_start = t_u[10]
  t_end = t_u[-1000]
  # import pdb; pdb.set_trace()
  # Override here #
  # t_start = 50
  # t_end = 60
  ### Convert times to indices
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)

  # t_lcmlog_u -= t_lcmlog_u[0] - t_u[0]
  # plt.plot(t_u[:-1], np.diff(t_u))
  # plt.plot(t_x[:-1], np.diff(t_x))
  # plt.plot(t_u[:-1], np.diff(t_lcmlog_u))
  # plt.figure()
  # plt.plot(t_u, t_lcmlog_u - t_u)
  # plt.show()

  ### All plotting scripts here
  # plot_status(full_log)
  plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas)
  # plot_contact_est(full_log)
  # plt.plot(t_contact_info, contact_info[0, :, 2], 'b-')
  # plt.plot(t_contact_info, contact_info[2, :, 2], 'r-')
  # plt.plot(t_u[t_u_slice], 100 * fsm[t_u_slice], 'k')

  plt.ylim([-100, 500])
  # plt.plot(t_u[t_u_slice], fsm[t_u_slice])

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

  plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
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
  for i in range(len(log["CASSIE_GM_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_GM_CONTACT_DISPATCHER"][i]
    t_gm_contact.append(msg.timestamp / 1e6)
    gm_contact_l.append(list(msg.point_pair_contact_info[0].contact_force))
    gm_contact_r.append(list(msg.point_pair_contact_info[1].contact_force))
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

  plt.figure("blend_amount")
  plt.plot(t_u[t_u_slice], qp_solve_time[t_u_slice])

  plt.figure("costs")
  plt.plot(t_u[t_u_slice], input_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], acceleration_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))

  # For Jumping
  # osc_traj0 = "com_traj"
  # osc_traj1 = "left_ft_traj"
  # osc_traj2 = "left_toe_angle_traj"

  # For Walking
  osc_traj0 = "lipm_traj"
  osc_traj1 = "swing_ft_traj"
  # osc_traj2 = "swing_hip_yaw_traj"
  # osc_traj1 = "l_foot_traj"
  osc_traj2 = "swing_hip_yaw_traj"
  osc_traj3 = "left_toe_angle_traj"
  osc_traj4 = "right_toe_angle_traj"

  #
  # plot_osc(osc_debug, osc_traj0, 0, "pos")
  # plot_osc(osc_debug, osc_traj0, 1, "pos")
  # plot_osc(osc_debug, osc_traj0, 2, "pos")
  #
  plot_osc(osc_debug, osc_traj0, 0, "vel")
  plot_osc(osc_debug, osc_traj0, 1, "vel")
  plot_osc(osc_debug, osc_traj0, 2, "vel")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  #
  # plot_osc(osc_debug, osc_traj0, 0, "accel")
  # plot_osc(osc_debug, osc_traj0, 1, "accel")
  # plot_osc(osc_debug, osc_traj0, 2, "accel")

  # plot_osc(osc_debug, osc_traj1, 0, "pos")
  # plot_osc(osc_debug, osc_traj1, 1, "pos")
  # plot_osc(osc_debug, osc_traj1, 2, "pos")

  plot_osc(osc_debug, osc_traj1, 0, "vel")
  plot_osc(osc_debug, osc_traj1, 1, "vel")
  plot_osc(osc_debug, osc_traj1, 2, "vel")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])


  # plot_osc(osc_debug, osc_traj2, 0, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  # plot_osc(osc_debug, osc_traj2, 1, "pos")
  # plot_osc(osc_debug, osc_traj2, 2, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  #
  # plot_osc(osc_debug, osc_traj2, 0, "vel")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

# plot_osc(osc_debug, osc_traj2, 1, "vel")
  # plot_osc(osc_debug, osc_traj2, 2, "vel")

  # plot_osc(osc_debug, osc_traj1, 0, "accel")
  # plot_osc(osc_debug, osc_traj1, 1, "accel")
  # plot_osc(osc_debug, osc_traj1, 2, "accel")

  # plot_osc(osc_debug, osc_traj2, 0, "accel")
  # plot_osc(osc_debug, osc_traj2, 1, "accel")
  # plot_osc(osc_debug, osc_traj2, 2, "accel")

  plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])
  plot_osc(osc_debug, osc_traj4, 0, "pos")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])

  # plot_osc(osc_debug, osc_traj3, 0, "vel")
  # plot_osc(osc_debug, osc_traj3, 0, "accel")
  # plt.plot(osc_debug[osc_traj0].t[t_u_slice], fsm[t_u_slice])


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
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim] - osc_debug[osc_traj].ydot[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim])
    plt.legend(["ydot_des", "ydot", "error_ydot", "corrected_error"])
    # plt.legend(["error_ydot", "corrected_error"])
  elif (derivative == "accel"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command_sol[t_u_slice, dim])
    plt.legend(["yddot_des", "yddot_command", "yddot_command_sol"])


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
  state_indices = slice(2, 3)
  # state_indices = slice(5, 6)
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


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, u_meas):
  # pos_indices = slice(0 + 7, 23)
  vel_indices = slice(23 + 6, 45)
  pos_indices = slice(0, 7)
  # vel_indices = slice(23, 23 + 6)
  u_indices = slice(0, 10)
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
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices], '--')
  plt.legend(u_datatypes[u_indices])
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

if __name__ == "__main__":
  main()
