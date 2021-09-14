import numpy as np
import pickle
from pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.common.plot_styler import PlotStyler
# from pydairlib.cassie.cassie_utils import *
from pydrake.systems.framework import *
from pydairlib.cassie.kinematics_helper import KinematicsHelper
import pydairlib.multibody
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


def simulate_from_trajectory(x_init, start_time, end_time, dt):



  pass


def get_rigid_impact_trajectory(x_traj, t_x, impact_time, end_time):
  impact_idx = np.argwhere(np.isclose(t_x, impact_time, atol=5e-4))[0][0]
  x_pre_traj = x_traj[:impact_idx]
  x_post = kinematics_calculator.compute_rigid_impact_map(x_traj[impact_idx])
  # x_post_traj = simulate_from_trajectory(x_post, impact_time, end_time, 5e-4)
  x_post_traj = np.repeat(x_post[np.newaxis, : ], t_x.shape[0] - impact_idx, axis=0)
  x_rigid_traj = np.concatenate((x_pre_traj, x_post_traj))
  return t_x, x_rigid_traj

def plot_velocity_trajectory(impact_data, log_num, indices, save_fig=False, compare_rigid=False):
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_hardware = impact_data.t_x_hardware[log_num]
  x_drake = impact_data.x_trajs_drake[log_num]
  t_drake = impact_data.t_x_drake[log_num]
  x_mujoco = impact_data.x_trajs_mujoco[log_num]
  t_mujoco = impact_data.t_x_mujoco[log_num]

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.05
  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  t_drake, x_drake = get_window_around_contact_event(x_drake, t_drake, start_time, end_time)
  t_mujoco, x_mujoco = get_window_around_contact_event(x_mujoco, t_mujoco, start_time, end_time)

  # impact_time = 0.5 * (start_time + end_time)
  if compare_rigid:
    impact_time = start_time + 0.003
    t_rigid, x_rigid = get_rigid_impact_trajectory(x_drake, t_drake, impact_time, end_time)
  # import pdb; pdb.set_trace()
  # plt.figure(log_num)
  for i in indices:
    plt.figure(log_num + '_' + x_datatypes[i] + ': ' + str(i))
    ps.plot(t_hardware, x_hardware[:, i], xlabel='time', ylabel='velocity')
    ps.plot(t_drake, x_drake[:, i])
    ps.plot(t_mujoco, x_mujoco[:, i])
    if compare_rigid:
      ps.plot(t_rigid, x_rigid[:, i])
      ps.add_legend(['Drake', 'MuJoCo', 'Real Data', 'Rigid Impact'])
    else:
      ps.add_legend(['Drake', 'MuJoCo', 'Real Data'])
    if save_fig:
      str_idx = max(x_datatypes[i].find('left'), x_datatypes[i].find('right'))
      # if 'base' in x_datatypes[i]:
      #   str_idx = 0
      # str_idx = max(str_idx, x_datatypes[i].find('_') + 1)
      # import pdb; pdb.set_trace()
      ps.save_fig(x_datatypes[i][:str_idx - 1] + '/' + x_datatypes[i] + '_' + log_num)
  # ps.show_fig()


def plot_centroidal_trajectory(impact_data, log_num, use_center_of_mass=False, fixed_feet=False, compare_rigid=False, save_figs=False):
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_hardware = impact_data.t_x_hardware[log_num]
  x_drake = impact_data.x_trajs_drake[log_num]
  t_drake = impact_data.t_x_drake[log_num]
  x_mujoco = impact_data.x_trajs_mujoco[log_num]
  t_mujoco = impact_data.t_x_mujoco[log_num]

  start_time = impact_data.start_times[log_num]
  end_time = start_time + 0.05

  # import pdb; pdb.set_trace()

  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  t_drake, x_drake = get_window_around_contact_event(x_drake, t_drake, start_time, end_time)
  t_mujoco, x_mujoco = get_window_around_contact_event(x_mujoco, t_mujoco, start_time, end_time)
  t_rigid, x_rigid = np.zeros(1), np.zeros(1)

  if compare_rigid:
    impact_time = start_time + 0.013
    t_rigid, x_rigid = get_rigid_impact_trajectory(x_drake, t_drake, impact_time, end_time)

  com_pos_drake = np.empty((t_drake.shape[0], 3))
  com_vel_drake = np.empty((t_drake.shape[0], 3))
  com_pos_mujoco = np.empty((t_mujoco.shape[0], 3))
  com_vel_mujoco = np.empty((t_mujoco.shape[0], 3))
  com_pos_hardware = np.empty((t_hardware.shape[0], 3))
  com_vel_hardware = np.empty((t_hardware.shape[0], 3))
  com_pos_rigid = np.empty((t_rigid.shape[0], 3))
  com_vel_rigid = np.empty((t_rigid.shape[0], 3))
  for i in range(t_drake.shape[0]):
    x_sim_i = x_drake[i]
    if use_center_of_mass:
      com_pos_drake[i] = kinematics_calculator.compute_center_of_mass_pos(x_sim_i)
      com_vel_drake[i] = kinematics_calculator.compute_center_of_mass_vel(x_sim_i)
    else:
      if fixed_feet:
        com_pos_drake[i], com_vel_drake[i] = kinematics_calculator.compute_pelvis_pos_w_fixed_feet(x_sim_i)
      else:
        com_pos_drake[i] = x_sim_i[4:7]
        com_vel_drake[i] = x_sim_i[26:29]
  for i in range(t_mujoco.shape[0]):
    x_sim_i = x_mujoco[i]
    if use_center_of_mass:
      com_pos_mujoco[i] = kinematics_calculator.compute_center_of_mass_pos(x_sim_i)
      com_vel_mujoco[i] = kinematics_calculator.compute_center_of_mass_vel(x_sim_i)
    else:
      if fixed_feet:
        com_pos_mujoco[i], com_vel_mujoco[i] = kinematics_calculator.compute_pelvis_pos_w_fixed_feet(x_sim_i)
      else:
        com_pos_mujoco[i] = x_sim_i[4:7]
        com_vel_mujoco[i] = x_sim_i[26:29]
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
  for i in range(t_rigid.shape[0]):
    x_i = x_rigid[i]
    if use_center_of_mass:
      com_pos_rigid[i] = kinematics_calculator.compute_center_of_mass_pos(x_i)
      com_vel_rigid[i] = kinematics_calculator.compute_center_of_mass_vel(x_i)
    else:
      if fixed_feet:
        com_pos_rigid[i], com_vel_rigid[i] = kinematics_calculator.compute_pelvis_pos_w_fixed_feet(x_i)
      else:
        com_pos_rigid[i] = x_i[4:7]
        com_vel_rigid[i] = x_i[26:29]

  t_common = 0
  min_time_window = np.argmin([t_hardware.shape[0], t_drake.shape[0], t_mujoco.shape[0]])

  # n_samples_common = min(t_hardware.shape[0], t_drake.shape[0])
  # n_samples_common = min(n_samples_common, t_mujoco.shape[0])
  n_samples_common = 0
  if min_time_window == 0:
    t_common = t_hardware
    n_samples_common = t_hardware.shape[0]
  elif min_time_window == 1:
    t_common = t_drake
    n_samples_common = t_drake.shape[0]
  elif min_time_window == 2:
    t_common = t_mujoco
    n_samples_common = t_mujoco.shape[0]

  figure_title = 'pelvis'
  if use_center_of_mass:
    figure_title = 'com'

  t_common = t_common - t_common[0]
  t_common = 1000 * t_common
  plt.figure(figure_title + ': ' + log_num)
  # ps.plot(t_common, com_vel_sim[:n_samples_common, 0])
  # ps.plot(t_common, com_vel_hardware[:n_samples_common, 0])
  # plt.figure('y' + log_num)
  # ps.plot(t_common, com_vel_sim[:n_samples_common, 1])
  # ps.plot(t_common, com_vel_hardware[:n_samples_common, 1])
  # plt.figure('z' + log_num)
  ps.plot(t_common, com_vel_drake[:n_samples_common, 2], color=ps.penn_color_wheel[0])
  ps.plot(t_common, com_vel_mujoco[:n_samples_common, 2], color=ps.penn_color_wheel[1])
  ps.plot(t_common, com_vel_rigid[:n_samples_common, 2], color=ps.penn_color_wheel[2])
  ps.plot(t_common, com_vel_hardware[:n_samples_common, 2], color=ps.penn_color_wheel[3], xlabel='Time (ms)', ylabel='Pelvis - Foot Velocity (m/s)', xlim=[0, 50])
  legend = ['Drake', 'MuJoCo', 'Real Data']
  if compare_rigid:
    legend = ['Drake', 'MuJoCo', 'Rigid', 'Real Data']
  ps.add_legend(legend)
  # ps.add_legend(['x sim', 'x hardware', 'y sim', 'y hardware', 'z sim', 'z_hardware'])
  if save_figs:
    if compare_rigid:
      ps.save_fig(figure_title + '_rigid_' + log_num)
    else:
      ps.save_fig(figure_title + '_' + log_num)

  # plt.figure(figure_title + '_error: ' + log_num)
  # ps.plot(t_common, com_vel_sim[:n_samples_common, 0] - com_vel_hardware[:n_samples_common, 0])
  # ps.plot(t_common, com_vel_sim[:n_samples_common, 1] - com_vel_hardware[:n_samples_common, 1])
  # ps.plot(t_common, com_vel_sim[:n_samples_common, 2] - com_vel_hardware[:n_samples_common, 2])
  # ps.add_legend(['x', 'y', 'z'])
  # ps.add_legend(['z'])
  # if save_figs and impact_data.use_mujoco:
  #   ps.save_fig(figure_title + '_error_' + log_num)

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


def plot_loss_breakdown(impact_data, log_num, loss_func, save_figs=False, compare_final=False):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]
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
  n_omega = loss_func.rot_vel_slice.stop - loss_func.rot_vel_slice.start

  pos_diff = x_hardware[:, loss_func.position_slice] - x_sim[:, loss_func.position_slice]
  if compare_final:
    pos_diff = x_hardware[-1, loss_func.position_slice] - x_sim[-1, loss_func.position_slice]
  sqrt_pos_weights = np.sqrt(loss_func.weights.pos)
  pos_diff_vec = (pos_diff @ sqrt_pos_weights).ravel()
  pos_losses_vec = pos_diff_vec * pos_diff_vec
  pos_losses = pos_losses_vec.reshape((-1, n_positions))
  if save_figs:
    plt.figure(prefix + 'position_loss_breakdown: ' + log_num)
    plt.bar(x_datatypes[loss_func.position_slice], pos_losses.sum(axis=0))
    ps.save_fig(prefix + 'pos_error_breakdown_' + log_num)

  vel_diff = x_hardware[:, loss_func.velocity_slice] - x_sim[:, loss_func.velocity_slice]
  if compare_final:
    vel_diff = x_hardware[-1, loss_func.velocity_slice] - x_sim[-1, loss_func.velocity_slice]
  sqrt_vel_weights = np.sqrt(loss_func.weights.vel)
  vel_diff_vec = (vel_diff @ sqrt_vel_weights).ravel()
  vel_losses_vec = vel_diff_vec * vel_diff_vec
  vel_losses = vel_losses_vec.reshape((-1, n_velocities))
  if save_figs:
    plt.figure(prefix + 'velocity_loss_breakdown: ' + log_num)
    plt.bar(x_datatypes[loss_func.velocity_slice], vel_losses.sum(axis=0))
    ps.save_fig(prefix + 'vel_error_breakdown_' + log_num)

  omega_diff = x_hardware[:, loss_func.rot_vel_slice] - x_sim[:, loss_func.rot_vel_slice]
  if compare_final:
    vel_diff = x_hardware[-1, loss_func.rot_vel_slice] - x_sim[-1, loss_func.rot_vel_slice]
  sqrt_omega_weights = np.sqrt(loss_func.weights.omega)
  omega_diff_vec = (omega_diff @ sqrt_omega_weights).ravel()
  omega_losses_vec = omega_diff_vec * omega_diff_vec
  omega_losses = omega_losses_vec.reshape((-1, n_omega))
  if save_figs:
    plt.figure(prefix + 'omega_loss_breakdown: ' + log_num)
    plt.bar(x_datatypes[loss_func.rot_vel_slice], omega_losses.sum(axis=0))
    ps.save_fig(prefix + 'omega_error_breakdown_' + log_num)

  return pos_losses.sum(axis=0), vel_losses.sum(axis=0), omega_losses.sum(axis=0)


def main():
  global ps
  global figure_directory
  global data_directory
  global sim_data_directory
  global x_datatypes
  global kinematics_calculator

  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  # figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/mujoco_to_real_comparison/'
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/both_comparison/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory, figsize=[10, 6])
  loss_func = cassie_loss_utils.CassieLoss('2021_09_07_weights')

  plt.close()

  with open("x_datatypes", "rb") as fp:
    x_datatypes = pickle.load(fp)

  # load all the data used for plotting
  impact_data = CassieImpactData()
  kinematics_calculator = KinematicsHelper()

  # impact_data.save('optimal_trajs_wo_rigid')
  # with open('/home/yangwill/Documents/research/projects/impact_uncertainty/data/' + 'optimal_trajs_wo_rigid' + '.pkl', 'rb') as f:
  #   test_data = pickle.load(f)


  joint_vel_indices = range(29, 45)
  hip_joints_indices = range(29, 35)
  fb_vel_indices = range(23, 29)
  joint_pos_indices = range(7, 23)

  # joint_vel_indices = range(35, 37)
  joint_vel_indices = range(33, 45)
  # joint_vel_indices = [33, 34, 35, 36, 37, 38]
  # joint_vel_indices = [35, 36]
  # joint_vel_indices = range(23, 26)

  pos_losses = []
  vel_losses = []
  omega_losses = []

  for log_num in impact_data.log_nums_real:
    # plt.figure(log_num)
    # grf_single_log(impact_data, log_num)
    # plot_velocity_trajectory(impact_data, log_num, joint_vel_indices, save_fig=True)
    # plot_velocity_trajectory(impact_data, log_num, joint_vel_indices, save_fig=False, compare_rigid=True)
    # plot_feet_positions_at_impact(impact_data, log_num)
    # plot_centroidal_trajectory(impact_data, log_num)
    # plot_centroidal_trajectory(impact_data, log_num, use_center_of_mass=False, fixed_feet=True, compare_rigid=True, save_figs=True)
    # pos_loss, vel_loss, omega_loss = plot_loss_breakdown(impact_data, log_num, loss_func, save_figs=True, compare_final=False)
    # pos_losses.append(pos_loss)
    # vel_losses.append(vel_loss)
    # omega_losses.append(omega_loss)
    # ps.show_fig()
    pass

  # pos_losses = np.array(pos_losses)
  # vel_losses = np.array(vel_losses)
  # omega_losses = np.array(omega_losses)

  # plt.figure('total_pos_loss_breakdown')
  # plt.bar(x_datatypes[loss_func.position_slice], pos_losses.sum(axis=0))
  # ps.save_fig('total_pos_loss_breakdown')

  # plt.figure('total_vel_loss_breakdown')
  # plt.bar(x_datatypes[loss_func.velocity_slice], vel_losses.sum(axis=0))
  # ps.save_fig('total_vel_loss_breakdown')

  # plt.figure('total_omega_loss_breakdown')
  # plt.bar(x_datatypes[loss_func.rot_vel_slice], omega_losses.sum(axis=0))
  # ps.save_fig('total_omega_loss_breakdown')

  ## All debugging scripts go here
  plot_centroidal_trajectory(impact_data, '25', use_center_of_mass=False, fixed_feet=True, compare_rigid=True, save_figs=True)
  # plot_velocity_trajectory(impact_data, '33', joint_vel_indices, save_fig=True, compare_rigid=True)
  # import pdb; pdb.set_trace()
  # plot_velocity_trajectory(impact_data, '08', hip_joints_indices)
  # plot_velocity_trajectory(impact_data, '15', joint_vel_indices, save_fig=False)
  # plot_feet_positions_at_impact(impact_data, '12')
  # grf_single_log(impact_data, '08')
  # plot_error_bands(impact_data)
  # plot_loss_breakdown(impact_data, '08', loss_func, save_figs=False)
  ps.show_fig()
  pass


if __name__ == '__main__':
  main()
