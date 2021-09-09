import numpy as np
import lcm
# from scipy.integrate import trapz
import pickle
from pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.common.plot_styler import PlotStyler
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.lcm import lcm_trajectory
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.cassie.cassie_utils import *
from pydairlib.cassie.kinematics_helper import KinematicsHelper
import pydairlib.multibody
from process_lcm_log import process_log
from cassie_impact_data import CassieImpactData
import scipy.linalg as linalg
from scipy import interpolate
import matplotlib.pyplot as plt
import cassie_loss_utils


def plot_error_bands(impact_data, save_figs=False):
  data_range = np.arange(28, 34, 1)
  v_sim = []
  v_hardware = []
  n_samples = 10000

  t_master = impact_data.t_x_hardware['15']
  for i in data_range:
    v_sim_interp = interpolate.interp1d(impact_data.t_x_sim['%.2d' % i], impact_data.x_trajs_sim['%.2d' % i], axis=0,
                                        bounds_error=False)
    v_hardware_interp = interpolate.interp1d(impact_data.t_x_hardware['%.2d' % i],
                                             impact_data.x_trajs_hardware['%.2d' % i], axis=0, bounds_error=False)
    v_sim.append(v_sim_interp(impact_data.t_x_sim['%.2d' % i]))
    v_hardware.append(v_hardware_interp(impact_data.t_x_sim['%.2d' % i]))

  # plt.xlim([-10, 30])
  # plt.show()
  v_all = np.stack(v_sim, axis=-1)
  vproj_all = np.stack(v_hardware, axis=-1)

  v_std = np.std(v_all, axis=2)
  v_mean = np.mean(v_all, axis=2)
  vproj_std = np.std(vproj_all, axis=2)
  vproj_mean = np.mean(vproj_all, axis=2)
  plt.figure('joint velocities')
  for i in range(12):
    ps.plot(t_master, v_mean[:, i], color=ps.cmap(i))
    ps.plot_bands(t_master, t_master, (v_mean - v_std)[:, i], (v_mean + v_std)[:, i], color=ps.cmap(i))
  # plt.xlim([-10, 50])
  # plt.ylim([-10, 10])
  plt.title('Joint Velocities')
  plt.xlabel('Time since Start of Impact (ms)')
  plt.ylabel('Velocity (rad/s)')
  if save_figs:
    ps.save_fig('joint_velocities_w_dev.png')
  plt.figure('projected joint velocities')
  for i in range(12):
    ps.plot(t_master, vproj_mean[:, i], color=ps.cmap(i))
    ps.plot_bands(t_master, t_master, (vproj_mean - vproj_std)[:, i], (vproj_mean + vproj_std)[:, i], color=ps.cmap(i))
  # plt.xlim([-10, 50])
  # plt.ylim([-3, 1])
  plt.title('Projected Joint Velocities')
  plt.xlabel('Time since Start of Impact (ms)')
  plt.ylabel('Velocity (rad/s)')
  if save_figs:
    ps.save_fig('projected_joint_velocities_w_dev.png')

  plt.show()


def get_window_around_contact_event(x_traj, t_x, start_time, end_time):
  start_idx = np.argwhere(np.isclose(t_x, start_time, atol=5e-4))[0][0]
  end_idx = np.argwhere(np.isclose(t_x, end_time, atol=5e-4))[0][0]
  # import pdb; pdb.set_trace()
  # start_idx = np.argwhere(np.isclose(t_x, start_time, atol=1e-4))[0][0]
  # end_idx = np.argwhere(np.isclose(t_x, end_time, atol=1e-4))[0][0]
  window = slice(start_idx, end_idx)
  return t_x[window], x_traj[window]


def plot_velocity_trajectory(impact_data, log_num, indices, save_fig=False):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.1

  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  # t_sim, x_sim = get_window_around_contact_event(x_sim, t_sim, start_time, end_time)
  # plt.figure(log_num)
  for i in indices:
    plt.figure(log_num + '_' + x_datatypes[i] + ': ' + str(i))
    ps.plot(t_hardware, x_hardware[:, i], xlabel='time', ylabel='velocity')
    ps.plot(t_sim, x_sim[:, i])
    ps.add_legend(['hardware', 'sim'])
    if save_fig:
      str_idx = max(x_datatypes[i].find('left'), x_datatypes[i].find('right'))
      ps.save_fig(x_datatypes[i][:str_idx - 1] + '/' + x_datatypes[i] + '_' + log_num)


def plot_centroidal_trajectory(impact_data, log_num, use_center_of_mass=False, fixed_feet=False, save_figs=False):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.05

  # import pdb; pdb.set_trace()

  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  t_sim, x_sim = get_window_around_contact_event(x_sim, t_sim, start_time, end_time)

  com_pos_sim = np.empty((t_sim.shape[0], 3))
  com_vel_sim = np.empty((t_sim.shape[0], 3))
  com_pos_hardware = np.empty((t_hardware.shape[0], 3))
  com_vel_hardware = np.empty((t_hardware.shape[0], 3))
  for i in range(t_sim.shape[0]):
    x_sim_i = x_sim[i]
    if use_center_of_mass:
      com_pos_sim[i] = kinematics_calculator.compute_center_of_mass_pos(x_sim_i)
      com_vel_sim[i] = kinematics_calculator.compute_center_of_mass_vel(x_sim_i)
    else:
      if fixed_feet:
        com_pos_sim[i], com_vel_sim[i] = kinematics_calculator.compute_pelvis_pos_w_fixed_feet(x_sim_i)
      else:
        com_pos_sim[i] = x_sim_i[4:7]
        com_vel_sim[i] = x_sim_i[26:29]
    #
  for i in range(t_hardware.shape[0]):
    x_i = x_hardware[i]
    if use_center_of_mass:
      com_pos_hardware[i] = kinematics_calculator.compute_center_of_mass_pos(x_i)
      com_vel_hardware[i] = kinematics_calculator.compute_center_of_mass_vel(x_i)
    else:
      if fixed_feet:
        com_pos_hardware[i], com_vel_hardware[i] = kinematics_calculator.compute_pelvis_pos_w_fixed_feet(x_i)
      else:
        com_pos_hardware[i] = x_i[4:7]
        com_vel_hardware[i] = x_i[26:29]
  t_common = 0
  n_samples_common = min(t_hardware.shape[0], t_sim.shape[0])
  if (t_hardware.shape[0] < t_sim.shape[0]):
    t_common = t_hardware
  elif (t_hardware.shape[0] >= t_sim.shape[0]):
    t_common = t_sim

  figure_title = 'pelvis'

  if False:
    y_lim = [-2.0, 1.0]
    y_error_lim = [-0.2, 0.35]
    if use_center_of_mass:
      figure_title = 'com'
      y_lim = [-2.0, 1.0]
      y_error_lim = [-0.2, 0.35]

  plt.figure(figure_title + ': ' + log_num)
  ps.plot(t_common, com_vel_sim[:n_samples_common, 0])
  ps.plot(t_common, com_vel_hardware[:n_samples_common, 0])
  # plt.figure('y' + log_num)
  ps.plot(t_common, com_vel_sim[:n_samples_common, 1])
  ps.plot(t_common, com_vel_hardware[:n_samples_common, 1])
  # plt.figure('z' + log_num)
  ps.plot(t_common, com_vel_sim[:n_samples_common, 2])
  ps.plot(t_common, com_vel_hardware[:n_samples_common, 2])
  ps.add_legend(['x sim', 'x hardware', 'y sim', 'y hardware', 'z sim', 'z_hardware'])
  if save_figs:
    ps.save_fig(figure_title + '_' + log_num)

  plt.figure(figure_title + '_error: ' + log_num)
  ps.plot(t_common, com_vel_sim[:n_samples_common, 0] - com_vel_hardware[:n_samples_common, 0])
  ps.plot(t_common, com_vel_sim[:n_samples_common, 1] - com_vel_hardware[:n_samples_common, 1])
  ps.plot(t_common, com_vel_sim[:n_samples_common, 2] - com_vel_hardware[:n_samples_common, 2])
  ps.add_legend(['x', 'y', 'z'])
  if save_figs:
    ps.save_fig(figure_title + '_error_' + log_num)

  return


def grf_single_log(impact_data, log_num):
  lambda_hardware = impact_data.contact_forces_hardware[log_num]
  lambda_sim = impact_data.contact_forces_sim[log_num]
  t_hardware = impact_data.t_x_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  ps.plot(t_hardware, lambda_hardware[0, :, 2])
  ps.plot(t_hardware, lambda_hardware[2, :, 2])
  ps.plot(t_sim, lambda_sim[0, :, 2])
  ps.plot(t_sim, lambda_sim[1, :, 2])
  ps.plot(t_sim, lambda_sim[2, :, 2])
  ps.plot(t_sim, lambda_sim[3, :, 2])


def plot_feet_positions_at_impact(impact_data, log_num):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.1
  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)

  foot_pos_hardware = np.empty((t_hardware.shape[0], 4))
  foot_vel_hardware = np.empty((t_hardware.shape[0], 4))
  for i in range(t_hardware.shape[0]):
    x_i = x_hardware[i]
    foot_pos_hardware[i] = kinematics_calculator.compute_foot_z_position(x_i)
    foot_vel_hardware[i] = kinematics_calculator.compute_foot_z_vel(x_i)

  plt.figure(log_num)
  ps.plot(t_hardware, foot_pos_hardware)
  ps.plot(t_hardware, foot_vel_hardware)


def plot_loss_breakdown(impact_data, log_num, loss_func, save_figs=False):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]
  compare_final = True
  prefix = ''
  if compare_final:
    prefix = 'final_'

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.05
  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  t_sim, x_sim = get_window_around_contact_event(x_sim, t_sim, start_time, end_time)
  min_time_length = min(t_hardware.shape[0], t_sim.shape[0])
  t_hardware, x_hardware, t_sim, x_sim = t_hardware[:min_time_length], \
                                         x_hardware[:min_time_length], \
                                         t_sim[:min_time_length], \
                                         x_sim[:min_time_length]

  n_positions = loss_func.position_slice.stop - loss_func.position_slice.start
  n_velocities = loss_func.velocity_slice.stop - loss_func.velocity_slice.start

  pos_diff = x_hardware[:, loss_func.position_slice] - x_sim[:, loss_func.position_slice]
  if compare_final:
    pos_diff = x_hardware[-1, loss_func.position_slice] - x_sim[-1, loss_func.position_slice]
  pos_diff_vec = pos_diff.ravel()
  pos_losses_vec = pos_diff_vec * pos_diff_vec
  pos_losses = pos_losses_vec.reshape((-1, n_positions))
  if save_figs:
    plt.figure(prefix + 'position_loss_breakdown: ' + log_num)
    plt.bar(x_datatypes[loss_func.position_slice], pos_losses.sum(axis=0))
    ps.save_fig(prefix + 'pos_error_breakdown_' + log_num)

  vel_diff = x_hardware[:, loss_func.velocity_slice] - x_sim[:, loss_func.velocity_slice]
  if compare_final:
    vel_diff = x_hardware[-1, loss_func.velocity_slice] - x_sim[-1, loss_func.velocity_slice]
  vel_diff_vec = vel_diff.ravel()
  vel_losses_vec = vel_diff_vec * vel_diff_vec
  vel_losses = vel_losses_vec.reshape((-1, n_velocities))
  if save_figs:
    plt.figure(prefix + 'velocity_loss_breakdown: ' + log_num)
    plt.bar(x_datatypes[loss_func.velocity_slice], vel_losses.sum(axis=0))
    ps.save_fig(prefix + 'vel_error_breakdown_' + log_num)

  return pos_losses.sum(axis=0), vel_losses.sum(axis=0)


def main():
  global ps
  global nominal_impact_time
  global impact_time
  global figure_directory
  global data_directory
  global sim_data_directory
  global terrain_heights
  global perturbations
  global penetration_allowances
  global threshold_durations
  global x_datatypes
  global kinematics_calculator

  # start_time = 30.64
  # end_time = start_time + 0.05
  # data_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/data/'
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  sim_data_directory = '/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/'
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/mujoco_to_real_comparison/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)
  ps.set_figsize([20, 12])
  loss_func = cassie_loss_utils.CassieLoss('2021_08_27_weights')

  with open("x_datatypes", "rb") as fp:
    x_datatypes = pickle.load(fp)

  # load all the data used for plotting
  impact_data = CassieImpactData(use_mujoco=False)
  # impact_data = CassieImpactData(use_mujoco=False)
  kinematics_calculator = KinematicsHelper()

  joint_vel_indices = range(29, 45)
  hip_joints_indices = range(29, 35)
  fb_vel_indices = range(23, 29)
  # joint_pos_indices = range(7, 23)

  # joint_vel_indices = range(35, 37)
  # joint_vel_indices = range(29, 39)
  joint_vel_indices = [33, 34, 35, 36, 37, 38]
  # joint_vel_indices = [35, 36]

  # for log_num in ['08', '15', '24']:
  pos_losses = []
  vel_losses = []
  for log_num in impact_data.log_nums_real:
    # plt.figure(log_num)
    # grf_single_log(impact_data, log_num)
    # plot_velocity_trajectory(impact_data, log_num, joint_vel_indices, save_fig=False)
    # plot_feet_positions_at_impact(impact_data, log_num)
    # plot_centroidal_trajectory(impact_data, log_num)
    plot_centroidal_trajectory(impact_data, log_num, use_center_of_mass=False, fixed_feet=True, save_figs=False)
    # pos_loss, vel_loss = plot_loss_breakdown(impact_data, log_num, loss_func, save_figs=True)
    # pos_losses.append(pos_loss)
    # vel_losses.append(vel_loss)
    ps.show_fig()
    pass
  # pos_losses = np.array(pos_losses)
  # vel_losses = np.array(vel_losses)

  # plt.figure('total_pos_loss_breakdown')
  # plt.bar(x_datatypes[loss_func.position_slice], pos_losses.sum(axis=0))
  # ps.save_fig('total_pos_loss_breakdown')

  # plt.figure('total_vel_loss_breakdown')
  # plt.bar(x_datatypes[loss_func.velocity_slice], vel_losses.sum(axis=0))
  # ps.save_fig('total_vel_loss_breakdown')

  # import pdb; pdb.set_trace()
  # plot_velocity_trajectory(impact_data, '08', hip_joints_indices)
  # plot_velocity_trajectory(impact_data, '12', joint_vel_indices, save_fig=False)
  # plot_feet_positions_at_impact(impact_data, '12')
  # grf_single_log(impact_data, '08')
  # plot_error_bands(impact_data)
  # plot_loss_breakdown(impact_data, '08', loss_func, save_figs=False)
  ps.show_fig()
  pass


if __name__ == '__main__':
  main()
