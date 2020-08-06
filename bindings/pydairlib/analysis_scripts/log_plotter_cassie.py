import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import process_lcm_log
from numpy.linalg import norm
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial
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
  matplotlib.rcParams["savefig.directory"] = \
    "/home/yangwill/Documents/research/projects/cassie/walking/analysis" \
    "/figures/"

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  plant.mutable_gravity_field().set_gravity_vector(

    -9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # relevant MBP parameters
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)

  # for i in range(len(x_datatypes)):
  #   print(i)
  #   print(x_datatypes[i])


  n_modes = 3
  folder_path = "/home/yangwill/Documents/research/projects/cassie/walking/saved_trajs/"
  trajectory_name = "walking_0.5_processed"
  left_foot_mode_name = "left_foot_trajectory"
  right_foot_mode_name = "right_foot_trajectory"
  pelvis_rot_mode_name = "pelvis_rot_trajectory"
  com_mode_name = "center_of_mass_trajectory"
  # mode_name = "pelvis_rot_trajectory"
  # x_traj_nominal, x_hybrid_trajs_nominal, u_traj_nominal, \
  # u_hybrid_trajs_nominal, decision_vars, datatypes \
  #   = load_lcm_trajs(37, nu, n_modes, folder_path, trajectory_name, mode_name)

  filename = sys.argv[1]
  log = lcm.EventLog(filename, "r")

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, \
  switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log  = process_lcm_log.process_log(log, pos_map, vel_map,
                                                  act_map)


  compare_ekf(full_log, pos_map, vel_map)
  # import pdb; pdb.set_trace()
  n_msgs = len(cassie_out)
  knee_pos = np.zeros(n_msgs)
  t_cassie_out = np.zeros(n_msgs)
  estop_signal = np.zeros(n_msgs)
  # motor_torques = np.zeros((n_msgs, nu))
  motor_torques = np.zeros(n_msgs)
  for i in range(n_msgs):
    # knee_pos[i] = cassie_out[i].leftLeg.kneeDrive.velocity
    t_cassie_out[i] = cassie_out[i].utime / 1e6
    motor_torques[i] = cassie_out[i].rightLeg.kneeDrive.torque
    estop_signal[i] = cassie_out[i].pelvis.radio.channel[8]

  # plt.figure("Delay characterization")
  # import pdb; pdb.set_trace()
  # plt.plot(t_cassie_out, motor_torques, 'b')
  # plt.plot(t_u, u[t_u_slice, 5], 'r.')
  # plt.plot(t_u_pd, u_pd[:, 7], 'g')
  # plt.plot(t_controller_switch, switch_signal, '*')
  plt.legend(["Motor torque", "Commanded Torque PD"])
  # plt.plot(t_cassie_out / 1e6, knee_pos, '.')

  t_start = t_x[10]
  t_end = t_x[-50]
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u  - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u  - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)
  # t_slice = slice(0, t_x.shape[0])

  # plt.figure("Efforts difference")
  # plt.plot(t_u_pd, u_pd, 'b')
  # plt.plot(t_pd, kp)
  # plt.plot(t_pd, kd)
  plot_state(x, u_meas, t_x, u, t_u, x_datatypes, u_datatypes)
  plt.show()
  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  context = plant.CreateDefaultContext()

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  if False:
    plot_feet_positions(plant, context, x, l_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "left_", "_front")
    plot_feet_positions(plant, context, x, r_toe_frame,
                        front_contact_disp,
                        world, t_x, t_slice, "right_", "_front")
    plot_feet_positions(plant, context, x, l_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "left_", "_rear")
    plot_feet_positions(plant, context, x, r_toe_frame,
                        rear_contact_disp,
                        world, t_x, t_slice, "right_", "_rear")
  # plt.figure("Contact info")
  # plt.plot(t_contact_info, contact_info[0, :, 2])
  # plt.plot(t_contact_info, contact_info[1, :, 2])
  # plt.plot(t_contact_info, contact_info[2, :, 2])
  # plt.plot(t_contact_info, contact_info[3, :, 2])
  # plt.legend(["lfoot_rear", "lfoot_front", "rfoot_rear", "rfoot_front"])
  # plot_nominal_output_trajectories(t_points_com, com_trajectory, pelvis_rot_trajectory, l_foot_trajectory,
  #                                  r_foot_trajectory)
  plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
  plt.show()


def plot_nominal_output_trajectories(t_points, com_trajectory, pelvis_rot_trajectory, l_foot_trajectory,
                                     r_foot_trajectory):
  t_sampled = np.linspace(0, t_points[-1], 100)
  l_foot = []
  r_foot = []
  com = []
  pelvis_rot = []
  pelvis_rot_trajectory_dot = pelvis_rot_trajectory.derivative(1)
  for t in t_sampled:
    l_foot.append(l_foot_trajectory.value(t))
    r_foot.append(r_foot_trajectory.value(t))
    com.append(com_trajectory.value(t))
    # pelvis_rot.append(pelvis_rot_trajectory.value(t))
    pelvis_rot.append(pelvis_rot_trajectory_dot.value(t))
  l_foot = np.array(l_foot)
  r_foot = np.array(r_foot)
  com = np.array(com)
  pelvis_rot = np.array(pelvis_rot)

  plt.figure("Nominal Trajectories")
  plt.plot(t_sampled, l_foot[:, :, 0])
  plt.plot(t_sampled, r_foot[:, :, 0])
  plt.plot(t_sampled, com[:, :, 0])
  plt.plot(t_sampled, pelvis_rot[:, :, 0])


def plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output):


  input_cost = np.zeros(t_u.shape[0])
  acceleration_cost = np.zeros(t_u.shape[0])
  soft_constraint_cost = np.zeros(t_u.shape[0])
  tracking_cost = np.zeros((t_u.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  num_tracking_cost = 0

  for i in range(t_u.shape[0] - 1):
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
  plt.plot(t_u, input_cost)
  plt.plot(t_u, acceleration_cost)
  plt.plot(t_u, soft_constraint_cost)
  plt.plot(t_u, tracking_cost)
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))
  plt.show()
  # fig = plt.figure("OSC debug")
  # plt.plot(t_osc_debug, osc_debug[0].y_des)
  fig = plt.figure("COM_x errors: ")
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_des[t_u_slice, 0])
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command[t_u_slice, 0])
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command_sol[t_u_slice, 0])
  # import pdb; pdb.set_trace()
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_y[t_u_slice, 0])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_ydot[t_u_slice, 0])
  # plt.plot(t_cassie_out, estop_signal, 'k-')
  plt.legend(["error y", "error ydot", "estop signal"])
  # fig = plt.figure("COM accel y: ")
  fig = plt.figure("COM_y errors: ")
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_des[t_u_slice, 1])
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command[t_u_slice, 1])
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command_sol[t_u_slice, 1])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_y[t_u_slice, 1])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_ydot[t_u_slice, 1])



  plt.show() #######****





  plt.legend(["error y", "error ydot", "estop signal"])
  # plt.plot(t_cassie_out, estop_signal, 'k-')
  fig = plt.figure("COM accel z: ")
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_des[t_u_slice, 2])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command[t_u_slice, 2])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command_sol[t_u_slice, 2])
  # plt.plot(t_cassie_out, estop_signal, 'k-')
  plt.legend(["yddot des", "yddot command", "yddot command sol", "estop signal"])
  fig = plt.figure("COM positions")
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].y_des[t_u_slice])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].y[t_u_slice])
  plt.legend(["x_desired", "y_desired", "z_desired", "x_actual", "y_actual", "z_actual"])
  fig = plt.figure("COM velocities")
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].ydot_des[t_u_slice])
  plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].ydot[t_u_slice])
  plt.legend(["xdot_desired", "ydot_desired", "zdot_desired", "xdot_actual", "ydot_actual", "zdot_actual"])
  fig = plt.figure("Pelvis Tracking")
  plt.plot(osc_debug["pelvis_rot_traj"].t[t_u_slice], osc_debug["pelvis_rot_traj"].ydot_des[t_u_slice])
  plt.plot(osc_debug["pelvis_rot_traj"].t[t_u_slice], osc_debug["pelvis_rot_traj"].ydot[t_u_slice])
  fig = plt.figure("Pelvis Tracking Quaternoin")
  plt.plot(osc_debug["pelvis_rot_traj"].t[t_u_slice], osc_debug["pelvis_rot_traj"].y_des[t_u_slice])
  plt.plot(osc_debug["pelvis_rot_traj"].t[t_u_slice], osc_debug["pelvis_rot_traj"].y[t_u_slice])

  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_y)
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].error_y)
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_des)
  # plt.plot(osc_debug["com_traj"].t[t_u_slice], osc_debug["com_traj"].yddot_command_sol)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].y_des)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].error_y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].error_ydot)
  # plt.plot(osc_debug["pelvis_rot_tracking_data"].t, osc_debug["pelvis_rot_tracking_data"].error_y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].yddot_des)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].yddot_command_sol)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].y_des)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].y)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].error_y)
  # plt.plot(osc_debug["r_foot_traj"].t , osc_debug["r_foot_traj"].ydot_des)
  # plt.plot(osc_debug["r_foot_traj"].t , osc_debug["r_foot_traj"].ydot)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].y_des)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].y)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].error_y)
  # plt.plot(t_u, fsm, 'k--')
  plt.legend(["w", "x", "y", "z", "w", "x", "y", 'z'])


def plot_feet_positions(plant, context, x, toe_frame, contact_point, world,
                        t_x, t_x_slice, foot_type, contact_type):
  foot_x = np.zeros((6, t_x.size))
  for i in range(t_x.size):
    # x = np.hstack((q[i, :], v[i, :]))
    plant.SetPositionsAndVelocities(context, x[i, :])
    foot_x[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                 contact_point, world)
    foot_x[3:6, i] = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, contact_point,
      world,
      world) @ x[i, -nv:]
  fig = plt.figure('foot pos: ' + filename)
  state_indices = slice(2, 3)
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
  q_est = np.array(q_est)
  v_est = np.array(v_est)

  pos_indices = slice(4, 7)
  vel_indices = slice(3, 6)
  plt.figure("Positions")
  plt.plot(t_x, q[:, pos_indices], '-')
  plt.plot(t_x_est, q_est[:, pos_indices], '--')
  plt.figure("Velocities")
  plt.plot(t_x, v[:, vel_indices], '-')
  plt.plot(t_x_est, v_est[:, vel_indices], '--')

def plot_state(x, u_meas, t_x, u, t_u, x_datatypes, u_datatypes):

  name = filename.split("/")[-1]
  pos_indices = slice(19, 22, 2)
  vel_indices = slice(33, 37)
  u_indices = slice(6, 8)
  # threshold = 1e-2
  # jumps = []
  # for i in range(x.shape[0] - 1):
  #   if norm(x[i + 1, pos_indices] - x[i, pos_indices]) > threshold:
  #     jumps.append(t_x[i])
  # jumps = np.array(jumps)
  plt.figure("positions: " + name)
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + name)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts meas: " + name)
  # plt.figure("Delay characterization")
  plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.figure("efforts meas: " + name)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.show()


if __name__ == "__main__":
  main()
