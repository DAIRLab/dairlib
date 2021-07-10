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
import dairlib


def main():
  global t_start
  global t_end
  global t_slice
  global t_u_slice
  global t_osc_debug_slice
  global filename
  global nq
  global nv
  global nx
  global pos_map
  global vel_map
  global act_map

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  filename = filename.split("/")[-1]

  # Get the urdf model
  cassie_state_channel_name = process_lcm_log.get_state_channel_name(log)
  urdf_file_path = ""
  for event in log:
    if event.channel == cassie_state_channel_name:
      msg = dairlib.lcmt_robot_output.decode(event.data)
      if msg.num_positions == 19:
        urdf_file_path = "examples/Cassie/urdf/cassie_fixed_springs.urdf"
      elif msg.num_positions == 23:
        urdf_file_path = "examples/Cassie/urdf/cassie_v2.urdf"
      else:
        raise ValueError("The dimension of q is not correct")
      break
  print("urdf_file_path = " + urdf_file_path + "\n")

  # Build a MBP
  builder = DiagramBuilder()
  plant_w_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant_w_spr).AddModelFromFile(FindResourceOrThrow(urdf_file_path))
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

  global front_contact_disp, rear_contact_disp, mid_contact_disp
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  mid_contact_disp = (front_contact_disp + rear_contact_disp) / 2

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant_w_spr)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant_w_spr)

  matplotlib.rcParams["savefig.directory"] = path

  x, u_meas, t_x, u, t_u, contact_switch, t_contact_switch, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, t_osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
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
  # t_start = 10.35
  # t_end = 10.4
  ### Convert times to indices
  t_start_idx = np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  t_u_slice = slice(start_time_idx, end_time_idx)
  # start_time_idx = np.argwhere(np.abs(t_osc_debug - t_start) < 1e-3)[0][0]
  # end_time_idx = np.argwhere(np.abs(t_osc_debug - t_end) < 1e-3)[0][0]
  # t_osc_debug_slice = slice(start_time_idx, end_time_idx)
  t_osc_debug_slice = slice(0, len(t_osc_debug))

  ### All plotting scripts here
  # plot_contact_est(full_log, t_osc_debug, fsm, t_u, u, t_x, x, u_meas)
  # PlotEkfMeasurementError(t_osc_debug, fsm)
  # plt.legend(["Left Foot force", "Right Foot force", "l_contact", "r_contact", "fsm", "pelvis y (250x)", "pelvis ydot (250x)", "abs_error_per_contact (1000x)"])

  # plot_measured_torque(t_u, u, t_x, t_osc_debug, u_meas, u_datatypes, fsm)

  plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, fsm)

  # plot_osc_debug(t_osc_debug, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)

  # plot_feet_positions(plant_w_spr, context, x, l_toe_frame, mid_contact_disp, world,
  #   t_x, t_slice, "left foot", True)
  # plot_feet_positions(plant_w_spr, context, x, l_toe_frame, front_contact_disp, world,
  #   t_x, t_slice, "left foot", True)
  # plot_feet_positions(plant_w_spr, context, x, l_toe_frame, rear_contact_disp, world,
  #   t_x, t_slice, "left foot", True)

  # plot_state_customized(x, t_x, u, t_u, x_datatypes, u_datatypes)
  # plt.plot(t_osc_debug, 0.1 * fsm)
  #
  # PlotCenterOfMass(x, t_x, plant_w_spr, world, context)
  #
  PlotVdot(x, t_x, x_datatypes, True)

  # PlotOscQpSol(t_osc_debug, osc_output)

  plt.show()

def PlotOscQpSol(t_osc_debug, osc_output):
  if len(osc_output) == 0:
    raise ValueError("osc_output is empty. Check the channel name.")

  lambda_c_dim = osc_output[0].qp_output.lambda_c_dim
  epsilon_dim = osc_output[0].qp_output.epsilon_dim
  v_dim = osc_output[0].qp_output.v_dim

  contact_forces = np.zeros((len(osc_output), lambda_c_dim))
  epsilons = np.zeros((len(osc_output), epsilon_dim))
  vdot = np.zeros((len(osc_output), v_dim))
  for i in range(len(osc_output)):
    contact_forces[i] = osc_output[i].qp_output.lambda_c_sol
    epsilons[i] = osc_output[i].qp_output.epsilon_sol
    vdot[i] = osc_output[i].qp_output.dv_sol

  plt.figure("Qp sol -- contact forces")
  plt.plot(t_osc_debug[:], contact_forces)
  plt.legend([str(i) for i in range(lambda_c_dim)])

  plt.figure("Qp sol -- epsilons")
  plt.plot(t_osc_debug[:], epsilons)
  plt.legend([str(i) for i in range(epsilon_dim)])

  plt.figure("Qp sol -- vdot")
  plt.plot(t_osc_debug[:], vdot[:, 0:6])
  plt.legend([str(i) for i in range(6)])


def PlotEkfMeasurementError(t_osc_debug, fsm):
  file_array = np.loadtxt("../ekf_error_w_momentum_observer.txt", delimiter=',')
  error_mag = np.zeros(file_array.shape[0])
  for i in range(len(error_mag)):
    error_mag[i] = np.sum(np.abs(file_array[i, 2:])) / file_array[i, 1]

  # plt.figure("ekf measurement (feet pos) error")
  plt.plot(file_array[:, 0], 1000 * error_mag)
  # plt.legend(["abs_error_per_contact"])
  # plt.plot(t_osc_debug, 0.01 * fsm)


# cutoff_freq is in Hz
def ApplyLowPassFilter(x, t, cutoff_freq):
  dt = np.diff(t)
  x_filtered = x[0, :]
  for i in range(len(dt)):
    alpha = 2 * np.pi * dt[i] * cutoff_freq / (2 * np.pi * dt[i] * cutoff_freq + 1)
    x_filtered = alpha * x[i + 1, :] + (1 - alpha) * x_filtered
    x[i + 1, :] = x_filtered
  return x

def CompareVdot(x, t_x, vdot, t_vdot):
  # Finite differencing seems accurate enough

  dx = np.diff(x, axis=0)
  dt = np.diff(t_x)
  vdot_numerical = dx[:, nq:]
  for i in range(len(dt)):
    vdot_numerical[i, :] /= dt[i]

  print("t_vdot.shape = ", t_vdot.shape)
  print("vdot.shape = ", vdot.shape)
  print("t_x.shape = ", t_x.shape)
  print("vdot_numerical.shape = ", vdot_numerical.shape)

  idx = 0

  plt.figure()
  plt.plot(t_vdot, vdot[:, idx])
  plt.plot(t_x[1:], vdot_numerical[:, idx])


def PlotVdot(x, t_x, x_datatypes, low_pass_filter = True):
  # Remove the first element (in simulation, we have two 0 timestamps)
  if t_x[0] == 0:
    x = x[1:, :]
    t_x = t_x[1:]

  # Low pass filter to velocity before doing finite differencing, because there
  # is encoder noise
  if low_pass_filter:
    x[:, nq:] = ApplyLowPassFilter(x[:, nq:], t_x, 100)

  # Finite differencing seems accurate enough
  dx = np.diff(x, axis=0)
  dt = np.diff(t_x)
  vdot_numerical = dx[:, nq:]
  for i in range(len(dt)):
    vdot_numerical[i, :] /= dt[i]

  # Testing -- Apply low pass filter to vdot as well
  if low_pass_filter:
    vdot_numerical = ApplyLowPassFilter(vdot_numerical, t_x[1:], 100)

  # Testing -- plot squared accleration
  # vdot_numerical = np.square(vdot_numerical)

  vel_indices = slice(6, nv)
  v_datatypes = x_datatypes[nq:]

  name = "acceleration-- " + filename
  if low_pass_filter:
    name = "filtered " + name
  plt.figure(name)
  plt.plot(t_x[1:], vdot_numerical[:, vel_indices])
  plt.legend(v_datatypes[vel_indices])


def plot_contact_est(log, t_osc_debug, fsm, t_u, u, t_x, x, u_meas):
  t_contact = []
  contact = []
  for i in range(len(log["CASSIE_CONTACT_DISPATCHER"])):
    msg = log["CASSIE_CONTACT_DISPATCHER"][i]
    t_contact.append(msg.utime / 1e6)
    contact.append(list(msg.contact))
  t_contact = np.array(t_contact)
  contact = np.array(contact)

  use_contact_force = len(log["CASSIE_GM_CONTACT_DISPATCHER"]) > 0
  t_contact_force = []
  contact_force = []
  contact_force = np.zeros((len(log["CASSIE_GM_CONTACT_DISPATCHER"]), 2))
  if use_contact_force:
    for i in range(len(log["CASSIE_GM_CONTACT_DISPATCHER"])):
      msg = log["CASSIE_GM_CONTACT_DISPATCHER"][i]
      t_contact_force.append(msg.timestamp / 1e6)
      for j in range(msg.num_point_pair_contacts):
        contact_force[i][j] = msg.point_pair_contact_info[j].contact_force[2]
    t_contact_force = np.array(t_contact_force)
    contact_force = np.array(contact_force)

  plt.figure("Contact estimation-- " + filename)
  if use_contact_force:
    # print("plottingn contact force")
    # plt.plot(t_contact_force[t_slice], contact_force[t_slice, 0])
    # plt.plot(t_contact_force[t_slice], contact_force[t_slice, 1])
    # plt.xlabel('Time Since Nominal Impact (s)')
    # plt.ylabel('Estimated Normal Contact Force (N)')
    # plt.legend(["Left Foot force", "Right Foot force"])
    pass

  # plt.figure("Contact estimation")
  plt.plot(t_contact[t_slice], 10 * contact[t_slice], '-')
  # plt.legend(["l_contact", "r_contact"])

  plt.plot(t_osc_debug, 30 * fsm)

  plt.plot(t_x[t_slice], 250 * x[t_slice, 5])
  plt.plot(t_x[t_slice], 250 * x[t_slice, nq + 4])

  # plt.plot(t_u[t_u_slice], u[t_u_slice, 0])
  # plt.plot(t_x[t_slice], u_meas[t_slice, 0])

  # plt.legend(["Left Foot force", "Right Foot force", "l_contact", "r_contact", "fsm", "pelvis y (250x)"])
  # plt.legend(["Left Foot force", "Right Foot force", "l_contact", "r_contact", "fsm", "pelvis y (250x)", "pelvis ydot (250x)", "u", "u_meas"])
  plt.legend(["l_contact", "r_contact", "fsm", "pelvis y (250x)", "pelvis ydot (250x)"])


def plot_osc_debug(t_osc_debug, fsm, osc_debug, t_cassie_out, estop_signal, osc_output):
  input_cost = np.zeros(t_osc_debug.shape[0])
  acceleration_cost = np.zeros(t_osc_debug.shape[0])
  soft_constraint_cost = np.zeros(t_osc_debug.shape[0])
  tracking_cost = np.zeros((t_osc_debug.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  num_tracking_cost = 0

  for i in range(t_osc_debug.shape[0]):
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
  plt.plot(t_osc_debug[t_osc_debug_slice], input_cost[t_osc_debug_slice])
  plt.plot(t_osc_debug[t_osc_debug_slice], acceleration_cost[t_osc_debug_slice])
  plt.plot(t_osc_debug[t_osc_debug_slice], soft_constraint_cost[t_osc_debug_slice])
  plt.plot(t_osc_debug[t_osc_debug_slice], tracking_cost[t_osc_debug_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))
  # osc_traj0 = "swing_ft_traj"
  # osc_traj0 = "optimal_rom_traj"
  # osc_traj0 = "com_traj"  # for standing controller
  # osc_traj0 = "lipm_traj"
  osc_traj1 = "lipm_traj"
  # osc_traj2 = "pelvis_balance_traj"
  osc_traj2 = "pelvis_heading_traj"
  # osc_traj2 = "pelvis_rot_traj"  # for standing controller
  osc_traj3 = "swing_hip_yaw_traj"

  # osc_traj0 = "left_support_traj"
  # for i in range(10):
  #   plot_osc(osc_debug, osc_traj0, i, "pos")
  # for i in range(10):
  #   plot_osc(osc_debug, osc_traj0, i, "vel")
  # for i in range(10):
  #   plot_osc(osc_debug, osc_traj0, i, "accel")

  #
  # # plot_osc(osc_debug, osc_traj0, 0, "pos")
  # # plt.plot(t_osc_debug[t_osc_debug_slice], 0.1 * fsm[t_osc_debug_slice])
  # plot_osc(osc_debug, osc_traj0, 1, "pos")
  # # plt.plot(t_osc_debug[t_osc_debug_slice], 0.1 * fsm[t_osc_debug_slice])
  # # plot_osc(osc_debug, osc_traj0, 2, "pos")
  # # plt.plot(t_osc_debug[t_osc_debug_slice], 0.1 * fsm[t_osc_debug_slice])
  #
  # # plot_osc(osc_debug, osc_traj0, 0, "vel")
  # plot_osc(osc_debug, osc_traj0, 1, "vel")
  # # plot_osc(osc_debug, osc_traj0, 2, "vel")
  #
  # # plot_osc(osc_debug, osc_traj0, 0, "accel")
  # # plt.plot(t_osc_debug[t_osc_debug_slice], 0.1 * fsm[t_osc_debug_slice])
  # plot_osc(osc_debug, osc_traj0, 1, "accel")
  # # plot_osc(osc_debug, osc_traj0, 2, "accel")
  # # plt.plot(t_osc_debug[t_osc_debug_slice], 0.1 * fsm[t_osc_debug_slice])

  # plot_osc(osc_debug, osc_traj1, 0, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_osc_debug_slice], fsm[t_osc_debug_slice])
  # plot_osc(osc_debug, osc_traj1, 1, "pos")
  # plot_osc(osc_debug, osc_traj1, 2, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_osc_debug_slice], fsm[t_osc_debug_slice])
  #
  # plot_osc(osc_debug, osc_traj1, 0, "vel")
  # plot_osc(osc_debug, osc_traj1, 1, "vel")
  # plot_osc(osc_debug, osc_traj1, 2, "vel")
  #
  plot_osc(osc_debug, osc_traj2, 0, "pos")
  plot_osc(osc_debug, osc_traj2, 1, "pos")
  plot_osc(osc_debug, osc_traj2, 2, "pos")

  plot_osc(osc_debug, osc_traj2, 0, "vel")
  plot_osc(osc_debug, osc_traj2, 1, "vel")
  plot_osc(osc_debug, osc_traj2, 2, "vel")

  plot_osc(osc_debug, osc_traj2, 0, "accel")
  plot_osc(osc_debug, osc_traj2, 1, "accel")
  plot_osc(osc_debug, osc_traj2, 2, "accel")

  # plot_osc(osc_debug, osc_traj1, 0, "accel")
  # plot_osc(osc_debug, osc_traj1, 1, "accel")
  # plot_osc(osc_debug, osc_traj1, 2, "accel")

  # plot_osc(osc_debug, osc_traj3, 0, "accel")
  # plot_osc(osc_debug, osc_traj3, 0, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_osc_debug_slice], fsm[t_osc_debug_slice])
  # plot_osc(osc_debug, osc_traj3, 0, "vel")


def plot_osc(osc_debug, osc_traj, dim, derivative):
  fig = plt.figure(osc_traj + " " + derivative + " tracking " + str(dim))
  if (derivative == "pos"):
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].y_des[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].y[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].error_y[t_osc_debug_slice, dim])
    plt.legend(["y_des", "y", "error_y"])
    # plt.legend(["y_des", "y"])
  elif (derivative == "vel"):
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].ydot_des[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].ydot[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].error_ydot[t_osc_debug_slice, dim])
    plt.legend(["ydot_des", "ydot", "error_ydot"])
  elif (derivative == "accel"):
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].yddot_des[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].yddot_command[t_osc_debug_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_osc_debug_slice], osc_debug[osc_traj].yddot_command_sol[t_osc_debug_slice, dim])
    plt.legend(["yddot_des", "yddot_command", "yddot_command_sol"])


def plot_feet_positions(plant, context, x, toe_frame, contact_point, world,
                        t_x, t_x_slice, foot_type, wrt_pelvis=False):
  foot_x = np.zeros((6, t_x.size))
  for i in range(t_x.size):
    plant.SetPositionsAndVelocities(context, x[i, :])
    foot_x[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                 contact_point, world)
    foot_x[3:6, i] = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, contact_point,
      world,
      world) @ x[i, -nv:]

  if wrt_pelvis:
    foot_x[:3,:] -= x[:, 4:7].T
    foot_x[3:,:] -= x[:, nq+3:nq+6].T

  contact_name = ''
  if (contact_point == front_contact_disp).all():
    contact_name = ' ' + toe_frame.name() + ', front_contact'
  elif (contact_point == rear_contact_disp).all():
    contact_name = ' ' + toe_frame.name() + ', rear_contact'
  elif (contact_point == mid_contact_disp).all():
    contact_name = ' ' + toe_frame.name() + ', mid_contact'

  string_wrt_pelvis = ' wrt pelvis' if wrt_pelvis else ''
  fig = plt.figure('foot pos' + string_wrt_pelvis + '-- ' + filename)
  # state_indices = slice(4, 5)
  # state_indices = slice(1, 6, 3) # only the z component
  state_indices = slice(5, 6)
  state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
  state_names = [foot_type + name for name in state_names]
  plt.plot(t_x[t_x_slice], foot_x.T[t_x_slice, state_indices], label=contact_name)
  # plt.legend(state_names[state_indices])
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
  plt.figure("EKF positions-- " + filename)
  plt.plot(t_x, q[:, pos_indices], '-')
  plt.plot(t_x_est, q_est[:, pos_indices], '--')
  plt.figure("EKF velocities-- " + filename)
  plt.plot(t_x, v[:, vel_indices], '-')
  plt.plot(t_x_est, v_est[:, vel_indices], '--')
  plt.figure("IMU-- " + filename)
  plt.plot(t_x, imu, 'k-')


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, fsm):
  # pos_indices = slice(0 + 7, 23, 2)
  # vel_indices = slice(23 + 6, 45, 2)
  pos_indices = slice(0,7)
  pos_indices2 = slice(7,7 + 8)
  vel_indices = slice(nq, nq + 6)
  u_indices = slice(6, 8)
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)
  vel_indices = slice(nq + 6, nq + nv - 2)
  vel_indices = slice(nq + 6, nq + 6 + 8)
  vel_indices2 = slice(nq + 6 + 8, nq + nv)
  # vel_indices = slice(nq + 3, nq + 6)
  u_indices = slice(0, 10)

  plt.figure("positions-- " + filename)
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["knee_joint_right"]])
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["ankle_spring_joint_right"]])
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("positions2-- " + filename)
  plt.plot(t_x[t_slice], x[t_slice, pos_indices2])
  plt.legend(x_datatypes[pos_indices2])
  plt.figure("velocities-- " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("velocities2-- " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices2])
  plt.legend(x_datatypes[vel_indices2])
  # plt.plot(t_u[t_u_slice], fsm[t_u_slice])
  plt.figure("efforts-- " + filename)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.plot(t_u[t_u_slice], 30 * fsm[t_u_slice])
  # plt.figure("efforts meas-- " + filename)
  # plt.figure("Delay characterization")
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  # plt.legend(u_datatypes[u_indices])

def plot_state_customized(x, t_x, u, t_u, x_datatypes, u_datatypes):
  pos_indices = slice(5,7)
  vel_indices = slice(nq + 3, nq + 3 + 3)

  plt.figure()
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.xlabel("time (s)")
  plt.ylabel("position (m) or velocity (m/s)")
  plt.legend(["pelvis_y", "pelvis_z", "pelvis_xdot", "pelvis_ydot", "pelvis_zdot"])


def plot_measured_torque(t_u, u, t_x, t_osc_debug, u_meas, u_datatypes, fsm):
  u_indices = slice(0, 8)

  plt.figure("efforts meas-- " + filename)
  plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  plt.plot(t_u[t_u_slice], u[t_u_slice])
  # plt.plot(t_u[t_u_slice], 30 * fsm[t_u_slice])
  plt.plot(t_osc_debug[t_osc_debug_slice], 30 * fsm[t_osc_debug_slice])

def PlotCenterOfMass(x, t_x, plant, world, context):
  # Compute COM and Comdot
  com = np.zeros((3, t_x.shape[0]))
  comdot = np.zeros((3, t_x.shape[0]))
  for i in range(t_x.shape[0]):
    xi = x[i, :]
    plant.SetPositionsAndVelocities(context, xi)
    com[:, i] = plant.CalcCenterOfMassPositionInWorld(context)
    J = plant.CalcJacobianCenterOfMassTranslationalVelocity(context,
      JacobianWrtVariable.kV, world, world)
    comdot[:, i] = J @ x[i, nq:]

  # Plot com along the cubic splines
  figname = "com traj"
  plt.figure(figname, figsize=(6.4, 4.8))
  plt.plot(t_x, com.T)
  plt.xlabel('time (s)')
  plt.legend(['x', 'y', 'z'])
  figname = "comdot traj"
  plt.figure(figname, figsize=(6.4, 4.8))
  plt.plot(t_x, comdot.T)
  plt.xlabel('time (s)')
  plt.legend(['x', 'y', 'z'])
  # figname = "comddot traj along the traj"
  # plt.figure(figname, figsize=(6.4, 4.8))
  # plt.plot(t_x, comddot.T, 'ko', markersize=4)
  # plt.xlabel('time (s)')
  # plt.legend(['x', 'y', 'z', 'x at knots', 'y at knots', 'z at knots'])


if __name__ == "__main__":
  main()
