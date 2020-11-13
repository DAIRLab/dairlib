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
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from load_lcm_trajs import load_lcm_trajs
from scipy import integrate
import scipy.io


def get_index_at_time(times, t):
  return np.argwhere(times - t > 0)[0][0]


def print_u(t_idx, length, osc_debug):
  print('y = ' + str(osc_debug.y[t_idx:t_idx + length, :]))
  print('y_des = ' + str(osc_debug.y_des[t_idx:t_idx + length, :]))
  print('error_y = ' + str(osc_debug.error_y[t_idx:t_idx + length, :]))
  print('ydot = ' + str(osc_debug.ydot[t_idx:t_idx + length, :]))
  print('ydot_des = ' + str(osc_debug.ydot_des[t_idx:t_idx + length, :]))
  print('error_ydot = ' + str(osc_debug.error_ydot[t_idx:t_idx + length, :]))
  print('yddot_des = ' + str(osc_debug.yddot_des[t_idx:t_idx + length, :]))
  print('yddot_command = ' + str(
    osc_debug.yddot_command[t_idx:t_idx + length, :]))
  print('yddot_command_sol = ' + str(
    osc_debug.yddot_command_sol[t_idx:t_idx + length, :]))


def main():
  # sys.settrace()
  global filename
  global x_traj_nominal
  global u_traj_nominal
  global x_hybrid_trajs_nominal
  global u_hybrid_trajs_nominal
  global state_slice
  global time_slice
  global t0_, tf_
  global t_minus, t_plus, t_final, impact_idx
  global plant
  global world
  global nq, nv, nx, nu
  # Set default directory for saving figures
  matplotlib.rcParams["savefig.directory"] = \
    "/home/yangwill/Documents/research/projects/cassie/jumping/analysis" \
    "/figures/"

  ## Set up the MBP

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

  state_names_w_spr = [[] for i in range(len(pos_map) + len(vel_map))]
  for name in pos_map:
    state_names_w_spr[pos_map[name]] = name
    print(name)
    print(pos_map[name])
  for name in vel_map:
    state_names_w_spr[nq + vel_map[name]] = name

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  context = plant.CreateDefaultContext()

  ## Load the saved lcm trajectories

  n_modes = 3
  folder_path = "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/"
  trajectory_name = "June_5_jumping_0.2"
  mode_name = "cassie_jumping_trajectory_x_u"
  x_traj_nominal, x_hybrid_trajs_nominal, u_traj_nominal, \
  u_hybrid_trajs_nominal, decision_vars, datatypes \
    = load_lcm_trajs(37, nu, n_modes, folder_path, trajectory_name, mode_name)

  ## Load the lcm log from the experiment
  if len(sys.argv) < 2:
    sys.stderr.write("Provide log file as command line argument!")
    sys.stderr.write("Must be an absolute path")
    sys.exit(1)

  filename = sys.argv[1]
  log = lcm.EventLog(filename, "r")
  print("Loading log")
  x, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, osc_debug, fsm, estop_signal, \
  switch_signal, t_controller_switch = process_lcm_log.process_log(log, pos_map, vel_map)

  start_time = 1.5
  end_time = 3.0

  # plot_nominal_com_traj(com_traj, lcm_com_traj)
  # plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj)
  # plot_nominal_input_traj(u_traj_nominal, state_traj_mode0)

  # calcNetImpulse(plant, context, t_contact_info, contact_info, t_x, q, v)

  t_start_idx = get_index_at_time(t_x, start_time)
  t_end_idx = get_index_at_time(t_x, end_time)
  t_x_slice = slice(t_start_idx, t_end_idx)

  plot_simulation_state(x, t_x, t_x_slice, state_names_w_spr)
  # plot_nominal_state(x_traj_nominal, datatypes, start_time, end_time)

  # For printing out osc_values at a specific time interval
  t_u_start_idx = get_index_at_time(t_u, start_time)
  t_u_end_idx = get_index_at_time(t_u, end_time)
  t_u_slice = slice(t_u_start_idx, t_u_end_idx)
  print("Num osc samples", t_u_end_idx - t_u_start_idx)

  plot_u_u(u, datatypes, t_u,
           t_u_end_idx, t_u_start_idx)

  # calc_costs(t_x, t_u, osc_debug, u)
  #
  # plot_nominal_u(nu, datatypes,
  #                             u_traj_nominal.get_segment_times(),
  #                             u_traj_nominal)

  plot_ground_reaction_forces(contact_info, t_x, t_x_slice)

  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  # Foot plotting
  if True:
    plot_feet_simulation(plant, context, x, l_toe_frame,
                         front_contact_disp,
                         world, t_x, t_x_slice, "left_", "_front")
    plot_feet_simulation(plant, context, x, r_toe_frame,
                         front_contact_disp,
                         world, t_x, t_x_slice, "right_", "_front")
    plot_feet_simulation(plant, context, x, l_toe_frame,
                         rear_contact_disp,
                         world, t_x, t_x_slice, "left_", "_rear")
    plot_feet_simulation(plant, context, x, r_toe_frame,
                         rear_contact_disp,
                         world, t_x, t_x_slice, "right_", "_rear")

  if False:
    fig = plt.figure("osc_output: " + filename)
    plt.plot(t_u[t_u_slice], osc_debug[0].ydot_des[t_u_slice], label="0")
    # plt.plot(t_u[t_u_slice], osc_debug[0].ydot[t_u_slice],
    #          label="0")
    plt.plot(t_u[t_u_slice], osc_debug[1].ydot_des[t_u_slice], label="1")
    # plt.plot(t_u[t_u_slice], osc_debug[1].ydot[t_u_slice],
    #          label="1")
    plt.legend()
  plt.show()


def calc_costs(t_x, t_u, osc_debug, u):
  # Cost is norm of efforts applied over impact event + OSC tracking error
  t_impact_start_idx = np.argwhere(t_x >= 0.0)[0][0]
  t_impact_end_idx = np.argwhere(t_x >= 0.3)[0][0]

  t_impact_start = t_x[t_impact_start_idx]
  t_impact_end = t_x[t_impact_end_idx]

  n_timesteps = t_impact_end_idx - t_impact_start_idx
  u_cost = np.zeros(n_timesteps)

  W0 = 2000 * np.eye(3)
  W0[1, 1] = 200
  W1 = 20 * np.eye(3)
  W1[1, 1] = 10

  for i in range(t_impact_start_idx, t_impact_end_idx):
    u_i = np.reshape(u[get_index_at_time(t_u, t_x[i])],
                     (nu, 1))

    u_cost[i - t_impact_start_idx] = norm(u_i)
  accumulated_u_cost = np.trapz(u_cost, t_x[t_impact_start_idx:
                                            t_impact_end_idx])
  osc_end_idx = get_index_at_time(t_u, t_x[t_impact_end_idx])
  tracking_err_0 = osc_debug[0].yddot_command_sol[osc_end_idx, :] - \
                   osc_debug[0].yddot_des[osc_end_idx, :]
  tracking_err_1 = osc_debug[0].yddot_command_sol[osc_end_idx, :] - \
                   osc_debug[0].yddot_des[osc_end_idx, :]
  tracking_cost = tracking_err_0.T @ W0 @ tracking_err_0 + \
                  tracking_err_1.T @ W1 @ tracking_err_1
  return tracking_cost, accumulated_u_cost


def calcNetImpulse(plant, context, t_contact_info, contact_info, t_x, x):
  n_dim = 3
  impact_duration = 0.05
  net_impulse = np.zeros((n_dim, 1))

  M = plant.CalcMassMatrixViaInverseDynamics(context)
  M_inv = np.linalg.inv(M)

  # Get the index for when the first grf is non-zero
  t_start_idx = np.min(np.argwhere(contact_info > 0), 0)[1]
  # Get the index for impact_duration (s) after the first non-zero grf
  t_end_idx = get_index_at_time(t_contact_info,
                                t_x[t_start_idx] + impact_duration)

  l_contact_frame = plant.GetBodyByName("toe_left").body_frame()
  r_contact_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))

  t_slice = slice(t_start_idx, t_end_idx)
  impulses = np.zeros((contact_info.shape[0], n_dim))
  for i in range(contact_info.shape[0]):
    for j in range(n_dim):
      impulses[i, j] = np.trapz(contact_info[i, t_slice, j],
                                t_contact_info[t_slice])

  impulse_from_contact = np.zeros((t_end_idx - t_start_idx, x.shape[1]))

  for i in range(t_start_idx, t_end_idx):
    plant.SetPositionsAndVelocities(context, x[i])
    J_l_r = plant.CalcJacobianTranslationalVelocity(context,
                                                    JacobianWrtVariable.kV,
                                                    l_contact_frame,
                                                    rear_contact_disp,
                                                    world, world)
    J_l_f = plant.CalcJacobianTranslationalVelocity(context,
                                                    JacobianWrtVariable.kV,
                                                    l_contact_frame,
                                                    front_contact_disp,
                                                    world, world)
    J_r_r = plant.CalcJacobianTranslationalVelocity(context,
                                                    JacobianWrtVariable.kV,
                                                    r_contact_frame,
                                                    rear_contact_disp,
                                                    world, world)
    J_r_f = plant.CalcJacobianTranslationalVelocity(context,
                                                    JacobianWrtVariable.kV,
                                                    r_contact_frame,
                                                    front_contact_disp,
                                                    world, world)

    impulse_from_contact[i - t_start_idx] = J_l_r.T @ contact_info[0, i]
    impulse_from_contact[i - t_start_idx] = J_l_f.T @ contact_info[1, i]
    impulse_from_contact[i - t_start_idx] = J_r_r.T @ contact_info[2, i]
    impulse_from_contact[i - t_start_idx] = J_r_f.T @ contact_info[3, i]

  # Assuming_the position change is negligible
  net_impulse_from_contact = np.zeros(x.shape[1])
  for j in range(x.shape[1]):
    net_impulse_from_contact[j] = np.trapz(impulse_from_contact[:, j],
                                           t_contact_info[
                                             t_slice])
  delta_v = M_inv @ net_impulse_from_contact
  print("Interval between: ", t_x[t_start_idx], t_x[t_end_idx])
  print(impulses)
  print(delta_v)
  return net_impulse


def plot_ground_reaction_forces(contact_info, t_x, t_x_slice):
  fig = plt.figure('contact data: ' + filename)
  plt.plot(t_x[t_x_slice], contact_info[0, t_x_slice, 2],
           label='$\lambda_n left_r$')
  plt.plot(t_x[t_x_slice], contact_info[1, t_x_slice, 2],
           label='$\lambda_n left_f$')
  plt.plot(t_x[t_x_slice], contact_info[2, t_x_slice, 2],
           label='$\lambda_n right_r$')
  plt.plot(t_x[t_x_slice], contact_info[3, t_x_slice, 2],
           label='$\lambda_n right_f$')
  plt.legend()


def plot_nominal_state(x_traj_nominal, state_names_wo_spr, start_time,
                       end_time):
  x_nominal = []
  xdot_nominal = []
  # v_nominal = []
  # xdot_traj_nominal = x_traj_nominal.derivative(1)
  pos_slice = slice(11, 15)
  # pos_slice = slice(4, 7)
  vel_slice = slice(19 + 10, 19 + 14)
  # vel_slice = slice(19 + 4, 19 + 7)
  # t_nominal = np.linspace(x_traj_nominal.start_time(),
  #                         x_traj_nominal.end_time(), 3000)
  t_nominal = np.linspace(start_time - 1.0, end_time - 1.0, 2000)
  for t in (t_nominal):
    x_nominal.append(x_traj_nominal.value(t))
    # xdot_nominal.append(xdot_traj_nominal.value(t))
  # fig = plt.figure('Nominal Traj')
  fig = plt.figure('Nominal state: ' + filename)
  x_nominal = np.array(x_nominal)
  xdot_nominal = np.array(xdot_nominal)
  # v_nominal = np.array(v_nominal)
  # plt.plot(t_nominal, x_nominal[:, pos_slice, 0])
  # plt.plot(t_nominal, x_nominal[:, vel_slice, 0])
  # plt.plot(t_nominal, x_nominal[:, 17, 0])
  plt.plot(t_nominal, x_nominal[:, vel_slice, 0])
  # plt.plot(t_nominal, xdot_nominal[:, vel_slice, 0])
  # plt.plot(t_x[t_x_slice], x_nominal[:, pos_slice, 0])
  # plt.legend(state_names_wo_spr[pos_slice])
  plt.legend(state_names_wo_spr[vel_slice])
  # plt.legend(state_names_wo_spr[pos_slice])
  # plt.legend(state_names_wo_spr[15])
  # plt.plot(x_traj_nominal.get_segment_times(), np.zeros(len(x_traj_nominal.get_segment_times())), 'k*')


def plot_nominal_u(nu, datatypes, t_nominal,
                   input_traj):
  fig = plt.figure('target controller inputs: ' + filename)
  inputs = np.zeros((1000, nu))
  times = np.zeros(1000)
  for i in range(1000):
    timestep = t_nominal[-1] / 1000 * i
    inputs[i] = input_traj.value(timestep).ravel()
    times[i] = timestep
  plt.plot(times, inputs)
  plt.ylim(-100, 300)
  plt.legend(datatypes[-10:])


def plot_u_u(u, datatypes, t_u,
             t_u_end_idx, t_u_start_idx):
  fig = plt.figure('controller inputs: ' + filename)
  osc_indices = slice(t_u_start_idx, t_u_end_idx)
  actuator_indices = slice(4, 8)
  plt.plot(t_u[osc_indices], u[osc_indices, actuator_indices])
  plt.ylim(-300, 300)
  plt.legend((datatypes[-10:])[actuator_indices])
  plt.xlabel("time (s)")
  plt.ylabel("torque (Nm)")


def plot_feet_simulation(plant, context, x, toe_frame, contact_point, world,
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


def plot_simulation_state(x, t_x, t_slice, state_names):
  fig = plt.figure('simulation positions')
  pos_indices = slice(7, nq)
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(state_names[pos_indices])
  fig = plt.figure('simulation velocities: ' + filename)
  vel_indices = slice(33, 37)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(state_names[vel_indices])


def plot_nominal_feet_traj(l_foot_traj, r_foot_traj):
  start_time = l_foot_traj.start_time()
  end_time = l_foot_traj.end_time()
  fig = plt.figure('feet trajectories: ' + filename)
  l_foot_points = []
  n_points = 2000
  times = np.linspace(start_time, end_time, n_points)
  for t in times:
    l_foot_points.append(l_foot_traj.value(t))
  l_foot_points = np.array(l_foot_points)
  plt.plot(times, l_foot_points[:, :, 0])


def plot_nominal_com_traj(com_traj):
  start_time = com_traj.start_time()
  end_time = com_traj.end_time()
  fig = plt.figure('target com trajectory: ' + filename)
  com_dot = com_traj.derivative(1)
  points = []
  dpoints = []
  n_points = 4500
  times = np.linspace(start_time, end_time, n_points)
  for t in times:
    points.append(com_traj.value(t))
    dpoints.append(com_dot.value(t))
  points = np.array(points)
  dpoints = np.array(dpoints)
  plt.plot(times, points[:, :, 0])
  plt.plot(times, dpoints[:, :, 0])


def plot_nominal_input_traj(u_traj_nominal, datatypes):
  start_time = u_traj_nominal.start_time()
  end_time = u_traj_nominal.end_time()
  fig = plt.figure('target input trajectory: ' + filename)
  points = []
  times = []
  input_slice = slice(4, 8)
  for i in range(1000):
    t = start_time + (end_time - start_time) * i / 1000
    times.append(t)
    points.append(u_traj_nominal.value(t))
  points = np.array(points)
  plt.plot(times, points[:, input_slice, 0])
  plt.legend(datatypes[4 + nx * 2: 8 + nx * 2])


if __name__ == "__main__":
  main()
