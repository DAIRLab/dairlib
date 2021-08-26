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



def plot_velocity_trajectory(impact_data, log_num, indices):
  x_hardware = impact_data.x_trajs_hardware[log_num]
  x_sim = impact_data.x_trajs_sim[log_num]
  ps.plot()


def plot_single_log(impact_data, log_num):
  lambda_hardware = impact_data.contact_forces_hardware[log_num]
  lambda_sim = impact_data.contact_forces_sim[log_num]
  t_hardware = impact_data.t_xs_hardware[log_num]
  t_sim = impact_data.t_xs_sim[log_num]
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
    plot_single_log(impact_data, log_num)




  ps.show_fig()


if __name__ == '__main__':
    main()