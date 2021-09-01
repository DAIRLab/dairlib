import numpy as np
import lcm
from scipy.integrate import trapz
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

import matplotlib.pyplot as plt


def get_window_around_contact_event(x_traj, t_x, start_time, end_time):
  # start_idx = np.argwhere(np.isclose(t_x, self.start_time, atol=5e-4))[0][0]
  # end_idx = np.argwhere(np.isclose(t_x, self.end_time, atol=5e-4))[0][0]
  start_idx = np.argwhere(np.isclose(t_x, start_time, atol=5e-4))[1][0]
  end_idx = np.argwhere(np.isclose(t_x, end_time, atol=5e-4))[1][0]
  window = slice(start_idx, end_idx)
  return t_x[window], x_traj[window]

def plot_velocity_trajectory(impact_data, log_num, indices):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]

  # import pdb; pdb.set_trace()

  t_hardware, x_hardware = get_window_around_contact_event(x_hardware, t_hardware, start_time, end_time)
  t_sim, x_sim = get_window_around_contact_event(x_sim, t_sim, start_time, end_time)
  for i in indices:
    # plt.figure(x_datatypes[i] + ': ' + str(i))
    plt.figure(x_datatypes[i] + ': ' + str(i) + log_num)
    ps.plot(t_hardware, x_hardware[:, i])
    ps.plot(t_sim, x_sim[:, i])

def plot_centroidal_trajectory(impact_data, log_num, use_center_of_mass=False):
  t_hardware = impact_data.t_x_hardware[log_num]
  x_hardware = impact_data.x_trajs_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]

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
      com_pos_sim[i] = x_sim_i[4:7]
      com_vel_sim[i] = x_sim_i[26:29]
    #
  for i in range(t_hardware.shape[0]):
    x_i = x_hardware[i]
    if use_center_of_mass:
      com_pos_hardware[i] = kinematics_calculator.compute_center_of_mass_pos(x_i)
      com_vel_hardware[i] = kinematics_calculator.compute_center_of_mass_vel(x_i)
    else:
      com_pos_hardware[i] = x_i[4:7]
      com_vel_hardware[i] = x_i[26:29]
    #
  # ps.plot(t_sim, com_pos_sim)
  # ps.plot(t_hardware, com_pos_hardware)
  plt.figure('x')
  ps.plot(t_sim, com_vel_sim[:, 0])
  ps.plot(t_hardware, com_vel_hardware[:, 0])
  plt.figure('y')
  ps.plot(t_sim, com_vel_sim[:, 1])
  ps.plot(t_hardware, com_vel_hardware[:, 1])
  plt.figure('z')
  ps.plot(t_sim, com_vel_sim[:, 2])
  ps.plot(t_hardware, com_vel_hardware[:, 2])

  return

def grf_single_log(impact_data, log_num):
  lambda_hardware = impact_data.contact_forces_hardware[log_num]
  lambda_sim = impact_data.contact_forces_sim[log_num]
  t_hardware = impact_data.t_x_hardware[log_num]
  t_sim = impact_data.t_x_sim[log_num]
  ps.plot(t_hardware, lambda_hardware[0, :, 2])
  ps.plot(t_hardware, lambda_hardware[2, :, 2])
  ps.plot(t_sim, lambda_sim[0, :, 2])
  ps.plot(t_sim, lambda_sim[2, :, 2])


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
  global x_datatypes
  global start_time
  global end_time
  global kinematics_calculator

  start_time = 30.64
  end_time = start_time + 0.05
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  with open("x_datatypes", "rb") as fp:
    x_datatypes = pickle.load(fp)

  impact_data = CassieImpactData()
  kinematics_calculator = KinematicsHelper()

  joint_vel_indices = range(29, 45)
  hip_joints_indices = range(29, 35)
  fb_vel_indices = range(23, 29)
  # joint_pos_indices = range(7, 23)
  # hip_index = range(29,30)

  # load all the data used for plotting
  # for log_num in impact_data.log_nums_real:
  #   plt.figure(log_num)
  #   grf_single_log(impact_data, log_num)
  #   plot_velocity_trajectory(impact_data, log_num, hip_index)
  plot_centroidal_trajectory(impact_data, '15')
  # plot_velocity_trajectory(impact_data, '08', hip_joints_indices)
  # plot_velocity_trajectory(impact_data, '21', joint_vel_indices)

  ps.show_fig()


if __name__ == '__main__':
  main()
