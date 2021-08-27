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
  ps.plot(t_hardware, x_hardware[:, indices])
  ps.plot(t_sim, x_sim[:, indices])


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

  global start_time
  global end_time

  start_time = 30.6477
  end_time = start_time + 0.05
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  impact_data = CassieImpactData()

  joint_vel_indices = slice(29, 45)
  hip_joints_indices = slice(29, 35)
  fb_vel_indices = slice(23, 29)

  # load all the data used for plotting
  for log_num in impact_data.log_nums_real:
    plt.figure(log_num)
    grf_single_log(impact_data, log_num)

  # plot_velocity_trajectory(impact_data, '08', hip_joints_indices)

  ps.show_fig()


if __name__ == '__main__':
  main()
