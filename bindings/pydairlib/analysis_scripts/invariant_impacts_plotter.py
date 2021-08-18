import numpy as np
import lcm
from scipy.integrate import trapz

from pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.common.plot_styler import PlotStyler
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.lcm import lcm_trajectory
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.cassie.cassie_utils import *
import pydairlib.multibody
from process_lcm_log import process_log

import matplotlib.pyplot as plt


def main():
  global ps
  global nominal_impact_time
  global impact_time
  global figure_directory
  global data_directory
  global terrain_heights
  global perturbations
  global penetration_allowances
  global threshold_durations
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  jumping_traj = lcm_trajectory.DirconTrajectory(filename)
  output_trajs = lcm_trajectory.LcmTrajectory(
    "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d_processed")
  lcm_right_foot_traj = output_trajs.GetTrajectory("right_foot_trajectory0")
  right_foot_traj = PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[0:3], lcm_right_foot_traj.datapoints[3:6])
  for mode in range(1, 3):
    lcm_right_foot_traj = output_trajs.GetTrajectory("right_foot_trajectory" + str(mode))
    right_foot_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[0:3], lcm_right_foot_traj.datapoints[3:6]))\

  input_traj = jumping_traj.ReconstructInputTrajectory()
  state_traj = jumping_traj.ReconstructStateTrajectory()
  nominal_impact_time = jumping_traj.GetStateBreaks(2)[0]

  impact_time = nominal_impact_time + 2.0
  # impact_time = nominal_impact_time

  terrain_heights = np.arange(0.00, 0.055, 0.005)
  penetration_allowances = np.array([1e-5, 1e-4, 1e-3, 5e-3])
  # durations = np.arange(0.000, 0.125, 0.025)
  durations = np.arange(0.000, 0.060, 0.010)
  durations = np.array([0.00, 0.03])
  perturbations = np.arange(-0.500, 0.600, 0.100)

  # For MUJOCO
  threshold_durations = np.arange(0.00, 0.11, 0.01)
  # penetration_allowances = np.array([1e-5])

  # Plotting options
  duration = '0.000'
  # duration = 'stiff'

  # construct_hardware_torque_plot()
  # plot_vel_discontinuity_example(right_foot_traj)
  # construct_knee_efforts_plot()
  # for d in durations:
  #   load_logs('%.3f' % d)


  accel_error = np.zeros((durations.shape[0], terrain_heights.shape[0], penetration_allowances.shape[0]))
  power_loss = np.zeros((durations.shape[0], terrain_heights.shape[0], penetration_allowances.shape[0]))
  for i in range(durations.shape[0]):
    print('%.3f' % durations[i])
    accel_error[i], power_loss[i] = count_successful_jumps('%.3f' % durations[i])
  construct_param_study_plots(durations, accel_error, power_loss)

  # count_successful_jumps(duration)
  # construct_knee_torque_bands_plot()
  # ps.add_legend(['%.0f (ms)' % (d*1e3) for d in durations])
  ps.show_fig()


def plot_vel_discontinuity_example(traj):
  times = np.arange(nominal_impact_time - 0.1, nominal_impact_time + 0.1, 0.001)
  accel = np.zeros((times.shape[0], 3))
  pos = np.zeros((times.shape[0], 3))
  vel = np.zeros((times.shape[0], 3))
  vel_err = np.zeros((times.shape[0], 3))
  accum_err = -0.2
  for i in range(times.shape[0]):
    accel[i, :] = traj.EvalDerivative(times[i], 2)[:, 0]
    pos[i, :] = traj.value(times[i])[:, 0]
    vel[i, :] = traj.EvalDerivative(times[i], 1)[:, 0]
    if(times[i] > nominal_impact_time - 0.01):
      accum_err = 0
    vel_err[i, :] = traj.EvalDerivative(times[i] + 0.01, 1)[:, 0] + accum_err
    # vel_err[i, :] = traj.EvalDerivative(times[i] + 0.01, 1)[:, 0]
    # if(np.mod(i, 5) == 0):
    accum_err += 0.1*(vel[i] - vel_err[i])
    # accum_err *= 0.6
  plt.figure("Velocity tracking during impact")
  reference_time = nominal_impact_time - 0.1
  ps.plot(1e3*(times - reference_time), vel[:, 2], xlabel='Time $t$ ', ylabel='Velocity $\dot y$', color=ps.blue)
  ps.plot(1e3*(times - reference_time), vel_err[:, 2], color=ps.red)
  ps.plot(1e3*(times - reference_time), vel_err[:, 2] - vel[:, 2], color=ps.grey)
  ps.add_legend(['target velocity', 'actual velocity', 'tracking error'])
  plt.xticks([])
  plt.yticks([])
  ps.save_fig('velocity_tracking_during_impact.png')
  # ps.show_fig()
  # plt.figure("Velocity error")
  # ps.add_legend(['feedback error'])
  # ps.save_fig('velocity_error_during_impact.png')
  # ps.plot([nominal_impact_time, nominal_impact_time], [-5, 5], '--')

  # plt.legend(['x','y','z'])

def load_logs(duration):
  builder = DiagramBuilder()

  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  plant_w_spr.Finalize()
  controller_channel = 'OSC_JUMPING'

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  osc_traj1 = "com_traj"
  osc_traj2 = "pelvis_rot_tracking_data"

  # For full jumping traj
  t_samples = 19000
  u_samples = 9000
  # For mujoco
  # t_samples = 10000
  # u_samples = 5000
  # For pelvis zvel perturbation
  # t_samples = 6000
  # u_samples = 3000
  # parameter_dim = perturbations.shape[0]
  parameter_dim = terrain_heights.shape[0]
  t_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], t_samples))
  x_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], t_samples, nx))
  t_u_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples))
  u_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples, nu))
  t_osc_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples))
  com_yddot_des_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples, 3))
  com_yddot_cmd_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples, 3))
  pelvis_rot_yddot_des_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples, 3))
  pelvis_rot_yddot_cmd_matrix = np.zeros((parameter_dim, penetration_allowances.shape[0], u_samples, 3))
  folder_path = '/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/' + duration + '/'
  # folder_path = '/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/mujoco/' + duration + '/'

  for i in range(terrain_heights.shape[0]):
  # for i in range(perturbations.shape[0]):
  # for i in range(threshold_durations.shape[0]):
    for j in range(penetration_allowances.shape[0]):
      # Mujoco logs
      # log_suffix = 'duration_%.3f_mujoco' % threshold_durations[i]
      # Drake logs
      log_suffix = 'height_%.4f-stiff_%.5f' % (terrain_heights[i], penetration_allowances[j])
      # log_suffix = 'pelvis_zvel_%.3f' % perturbations[i]

      log_path = folder_path + 'lcmlog-' + log_suffix

      print(log_path)
      log = lcm.EventLog(log_path, "r")
      x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
      osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
      osc_output, full_log, t_lcmlog_u = process_log(log, pos_map, vel_map, act_map, controller_channel)

      t_matrix[i, j, :] = t_x[:t_samples]
      x_matrix[i, j, :, :] = x[:t_samples]
      t_u_matrix[i, j, :] = t_u[:u_samples]
      u_matrix[i, j, :, :] = u[:u_samples]
      t_osc_matrix[i, j, :] = osc_debug[osc_traj1].t[:u_samples]
      com_yddot_des_matrix[i, j, :, :] = osc_debug[osc_traj1].yddot_des[:u_samples, :]
      com_yddot_cmd_matrix[i, j, :, :] = osc_debug[osc_traj1].yddot_command[:u_samples, :]
      pelvis_rot_yddot_des_matrix[i, j, :, :] = osc_debug[osc_traj2].yddot_des[:u_samples, :]
      pelvis_rot_yddot_cmd_matrix[i, j, :, :] = osc_debug[osc_traj2].yddot_command[:u_samples, :]

  np.save(data_directory + 't_x_' + duration, t_matrix)
  np.save(data_directory + 'x_' + duration, x_matrix)
  np.save(data_directory + 't_u_' + duration, t_u_matrix)
  np.save(data_directory + 'u_' + duration, u_matrix)
  np.save(data_directory + 't_osc_com_' + duration, t_osc_matrix)
  np.save(data_directory + 'osc_com_accel_des_' + duration, com_yddot_des_matrix)
  np.save(data_directory + 'osc_com_accel_cmd_' + duration, com_yddot_cmd_matrix)
  np.save(data_directory + 'osc_pelvis_rot_accel_des_' + duration, pelvis_rot_yddot_des_matrix)
  np.save(data_directory + 'osc_pelvis_rot_accel_cmd_' + duration, pelvis_rot_yddot_cmd_matrix)
  return

def construct_param_study_plots(durations, accel_error, power_loss):
  accel_error_norm = 2000 * (100 * 0.07)**2
  colors = [ps.blue, ps.red]
  color_idx = 0
  for i in range(durations.shape[0]):
    # ps.plot(terrain_heights * 1e2, accel_error[i, :, 0] / accel_error_norm, xlabel='Platform Height (cm)', ylabel='Weighted Acceleration Error $J_{acc}$', color = colors[color_idx])
    # ps.plot(terrain_heights * 1e2, accel_error[i, :, 0] / accel_error_norm, xlabel='Platform Height (cm)', ylabel='Tracking Error', color = colors[color_idx])
    # ps.plot(penetration_allowances, accel_error[i, 0, :] / accel_error_norm, xlabel='Ground Stiffness', ylabel='Weighted Acceleration Error $J_{acc}$')
    # ps.plot(terrain_heights * 1e2, power_loss[i, :, 0], xlabel='Platform Height (cm)', ylabel='Control Effort $J_{mot}$', color=colors[color_idx])
    ps.plot(terrain_heights * 1e2, power_loss[i, :, 0], xlabel='Platform Height (cm)', ylabel='Control Effort', color=colors[color_idx])
    # ps.plot(penetration_allowances, power_loss[i, 0, :], xlabel='Ground Stiffness', ylabel='Control Effort $J_{mot}$')
    color_idx += 1
  # plt.xscale('log')
  # ps.add_legend(['%.0f (ms)' % (d*1e3) for d in durations])
  ps.add_legend(['Default Controller', 'Impact Invariant Controller'], loc=2)
  # ps.save_fig('param_study_accel_err_height_for_video.png')
  # ps.save_fig('param_study_accel_err_stiffness.png')
  ps.save_fig('param_study_power_loss_height_for_video.png')
  # ps.save_fig('param_study_power_loss_stiffness.png')

def count_successful_jumps(duration, param = ''):
  t_matrix = np.load(data_directory + 't_x_' + param + duration + '.npy')
  x_matrix = np.load(data_directory + 'x_' + param + duration + '.npy')
  t_u_matrix = np.load(data_directory + 't_u_' + param + duration + '.npy')
  u_matrix = np.load(data_directory + 'u_' + param + duration + '.npy')
  t_osc_matrix = np.load(data_directory + 't_osc_com_' + duration + '.npy')
  osc_yddot_des_matrix = np.load(data_directory + 'osc_com_accel_des_' + param + duration + '.npy')
  osc_yddot_cmd_matrix = np.load(data_directory + 'osc_com_accel_cmd_' + param + duration + '.npy')
  osc_pelvis_yddot_des_matrix = np.load(data_directory + 'osc_pelvis_rot_accel_des_' + param + duration + '.npy')
  osc_pelvis_yddot_cmd_matrix = np.load(data_directory + 'osc_pelvis_rot_accel_cmd_' + param + duration + '.npy')

  # steady_state_time = 0.7
  steady_state_time = 3.0
  max_adj_window = 0.050
  accel_error_time = impact_time + max_adj_window
  success_height = 0.75
  z_fb_idx = 6
  successes = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  efforts = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  max_efforts = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  accel_error = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  u_slice = slice(0, 10)

  W_CoM = np.zeros((3,3))
  W_CoM[0, 0] = 2000
  W_CoM[1, 1] = 200
  W_CoM[2, 2] = 2000
  W_pelvis = np.zeros((3,3))
  W_pelvis[0, 0] = 100
  W_pelvis[1, 1] = 100
  W_pelvis[2, 2] = 100

  for i in range(terrain_heights.shape[0]):
    # for i in range(perturbations.shape[0]):
    for j in range(penetration_allowances.shape[0]):
      t_idx = np.argwhere(t_matrix[i, j] == steady_state_time)[0, 0]
      t_u_start_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time - 0.05)) < 2e-3)[0, 0]
      t_u_end_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time + 0.05)) < 2e-3)[0, 0]
      # t_u_eval_idx = np.argwhere(np.abs(t_osc_matrix[i, j] - accel_error_time) < 5e-3)[0, 0]
      t_u_eval_idx = np.argwhere(np.abs(t_osc_matrix[i, j] - accel_error_time) < 1e-2)[:, 0]
      # import pdb; pdb.set_trace()
      # if (x_matrix[i, j, t_idx, z_fb_idx] > (success_height + terrain_heights[i])):
      #   successes[i, j] = 1
      t_u_slice = slice(t_u_start_idx, t_u_end_idx)
      efforts[i, j] = trapz(np.square(np.sum(u_matrix[i, j, t_u_slice, u_slice], axis=1)),
                            t_u_matrix[i, j, t_u_slice])
      max_efforts[i, j] = np.max(u_matrix[i, j, t_u_slice, u_slice])
      for sample in range(t_u_eval_idx.shape[0]):
        accel_error[i, j] += osc_yddot_des_matrix[i, j, t_u_eval_idx[sample], :] @ W_CoM @ osc_yddot_cmd_matrix[i, j, t_u_eval_idx[sample], :].T
        accel_error[i, j] += osc_pelvis_yddot_des_matrix[i, j, t_u_eval_idx[sample], :] @ W_pelvis @ osc_pelvis_yddot_cmd_matrix[i, j, t_u_eval_idx[sample], :].T
      accel_error[i, j] *= 1.0/(t_u_eval_idx.shape[0])
      # import pdb; pdb.set_trace()
    # import pdb; pdb.set_trace()


  # ps.plot(terrain_heights, np.average(max_efforts, axis=1), xlabel='terrain height (m)', ylabel='actuator saturation (% of max)')
  # ps.plot(terrain_heights, np.average(accel_error, axis=1), xlabel='terrain height (m)', ylabel='average pelvis acceleration error (% of max)')
  # print('max effort: ')
  # print(np.mean(max_efforts))
  # print('power loss: ')
  # print(np.mean(efforts, axis=1))
  # print('mean:' )
  # print(np.mean(accel_error, axis=1))
  # print('median:' )
  # print(np.median(accel_error, axis=1))
  # print('stdev:' )
  # print(np.std(accel_error, axis=1))
  # import pdb; pdb.set_trace()
  return accel_error, efforts


def construct_knee_efforts_plot():
  t_u_no_adjustment = np.load('t_u_0.000.npy')
  t_u_with_adjustment = np.load('t_u_0.100.npy')
  u_no_adjustment = np.load('u_combined_efforts_0.000.npy')
  u_with_adjustment = np.load('u_combined_efforts_0.100.npy')
  t_u_no_adjustment -= impact_time
  t_u_with_adjustment -= impact_time
  t_u_no_adjustment *= 1e3
  t_u_with_adjustment *= 1e3
  ps.plot(t_u_no_adjustment, u_no_adjustment, ylim=[-50, 600], xlabel='time since impact (ms)',
          ylabel='motor torque (Nm)',
          title='combined knee motor torques')
  ps.plot(t_u_with_adjustment, u_with_adjustment, ylim=[-50, 600], xlabel='time since impact (ms)',
          ylabel='motor torque (Nm)',
          title='combined knee motor torques')
  ps.add_legend(['No adjustment', '200 ms window around the nominal impact time'])
  ps.show_fig()


def construct_knee_torque_bands_plot():
  # durations = np.array([0.000, 0.025, 0.050, 0.075, 0.100])
  durations = np.array([0.000, 0.050, 0.100])

  for i in range(durations.shape[0]):
    print(durations[i])
    construct_all_knee_efforts_plot('%.3f' % durations[i], ps.cmap(i))
  ps.save_fig('knee_motor_bands.png')
  ps.add_legend(['%.3f ms' % duration for duration in durations])
  ps.plot(np.zeros(0), np.zeros(0), xlabel='time since impact (ms)', ylabel='motor torque (Nm)')
  ps.show_fig()


def construct_all_knee_efforts_plot(duration, color):
  t_matrix = np.load(data_directory + 't_x_' + duration + '.npy')
  x_matrix = np.load(data_directory + 'x_' + duration + '.npy')
  t_u_matrix = np.load(data_directory + 't_u_' + duration + '.npy')
  u_matrix = np.load(data_directory + 'u_' + duration + '.npy')

  for i in range(terrain_heights.shape[0]):
    for j in range(penetration_allowances.shape[0]):
      # t_idx = np.argwhere(t_matrix[i, j] == 3.0)[0, 0]
      t_u_start_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time - 0.25)) < 2e-3)[0, 0]
      t_u_end_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time + 0.5)) < 2e-3)[0, 0]
      t_u_slice = slice(t_u_start_idx, t_u_end_idx)
      # if i == 3 and j == 0:


  low_idx = 0
  high_idx = 5
  t_u_start_idx = np.argwhere(np.abs(t_u_matrix[low_idx, 0] - (impact_time - 0.25)) < 2e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_u_matrix[low_idx, 0] - (impact_time + 0.5)) < 2e-3)[0, 0]
  t_u_low_slice = slice(t_u_start_idx, t_u_end_idx)
  t_u_start_idx = np.argwhere(np.abs(t_u_matrix[high_idx, 0] - (impact_time - 0.25)) < 2e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_u_matrix[high_idx, 0] - (impact_time + 0.5)) < 2e-3)[0, 0]
  t_u_high_slice = slice(t_u_start_idx, t_u_end_idx)
  # mid = np.sum(u_matrix[4, 0, t_u_slice, 6:8], axis=1)
  ps.plot(t_u_matrix[i, j, t_u_low_slice], np.median(np.sum(u_matrix[:, :, t_u_low_slice, 6:8], axis=3), axis=(0, 1)), linestyle=color,
          grid=False)
  # lower_bound = np.sum(u_matrix[low_idx, 0, t_u_low_slice, 6:8], axis=1)
  # upper_bound = np.sum(u_matrix[high_idx, 0, t_u_high_slice, 6:8], axis=1)
  lower_bound = np.median(np.sum(u_matrix[:, :, t_u_low_slice, 6:8], axis=3), axis=(0, 1)) - np.std(np.sum(u_matrix[:, :, t_u_low_slice, 6:8], axis=3), axis=(0, 1))
  upper_bound = np.median(np.sum(u_matrix[:, :, t_u_low_slice, 6:8], axis=3), axis=(0, 1)) + np.std(np.sum(u_matrix[:, :, t_u_low_slice, 6:8], axis=3), axis=(0, 1))
  import pdb; pdb.set_trace()
  # ps.plot_bands(t_u_matrix[low_idx, 0, t_u_low_slice], t_u_matrix[high_idx, 0, t_u_high_slice], lower_bound,
  #               upper_bound, color=color)
  ps.plot_bands(t_u_matrix[0, 0, t_u_low_slice], t_u_matrix[0, 0, t_u_low_slice], lower_bound,
                upper_bound, color=color)

def construct_hardware_torque_plot():

  builder = DiagramBuilder()

  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  plant_w_spr.Finalize()
  controller_channel = 'OSC_JUMPING'

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)


  hardware_impact = 30.0 + nominal_impact_time + 0.09
  # Drake logs
  log_indices = ['12', '15']
  hardware_log_path = '/home/yangwill/Documents/research/projects/cassie/hardware/logs/01_27_21/'
  colors = [ps.blue, ps.red]
  color_idx = 0
  for log_idx in log_indices:
    log_path = hardware_log_path + 'lcmlog-' + log_idx

    print(log_path)
    log = lcm.EventLog(log_path, "r")
    x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
    osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
    osc_output, full_log, t_lcmlog_u = process_log(log, pos_map, vel_map, act_map, controller_channel)
    t_u_start_idx = np.argwhere(np.abs(t_u - (hardware_impact - 0.2)) < 2e-3)[0, 0]
    t_u_end_idx = np.argwhere(np.abs(t_u - (hardware_impact + 0.3)) < 2e-3)[0, 0]
    t_u_slice = slice(t_u_start_idx, t_u_end_idx)
    u_indices = slice(6, 8)
    plt.figure("Combined knee motor efforts")
    ps.plot(t_u[t_u_slice] - hardware_impact, np.sum(u[t_u_slice, u_indices], axis=1), xlabel='Time Since Nominal Impact (s)', ylabel='Combined Knee Motor Torque (Nm)', color=colors[color_idx])
    color_idx += 1

  durations = np.arange(0.0, 0.200, 0.1)
  # ps.add_legend(['%.0f (ms)' % (d*1e3) for d in durations])
  ps.add_legend(['Default Controller', 'Impact Invariant Controller'])
  ps.save_fig('jan_27_hardware_knee_efforts_for_video.png')
  ps.show_fig()

if __name__ == '__main__':
  main()
