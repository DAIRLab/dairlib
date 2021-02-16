import numpy as np
import lcm
from scipy.integrate import trapz

from pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.parameter_studies.plot_styler import PlotStyler
from pydrake.trajectories import PiecewisePolynomial
import pydairlib.lcm_trajectory
from bindings.pydairlib.analysis_scripts.process_lcm_log import process_log
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.cassie.cassie_utils import *
import pydairlib.multibody

import matplotlib.pyplot as plt


def main():
  global ps
  global impact_time
  global figure_directory
  global data_directory
  global terrain_heights
  global penetration_allowances
  global threshold_durations
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  jumping_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)
  input_traj = jumping_traj.ReconstructInputTrajectory()
  state_traj = jumping_traj.ReconstructStateTrajectory()
  impact_time = jumping_traj.GetStateBreaks(2)[0] + 2.0 + 0.075

  terrain_heights = np.arange(0.00, 0.055, 0.005)
  penetration_allowances = np.array([1e-5, 1e-4, 1e-3])

  # For MUJOCO
  threshold_durations = np.arange(0.00, 0.11, 0.01)
  penetration_allowances = np.array([1e-5])



  # Plotting options
  # duration = '0.000'
  duration = 'stiff'

  # construct_knee_efforts_plot()
  # load_logs(duration)
  count_successful_jumps(duration)
  # construct_knee_torque_bands_plot()


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

  t_samples = 19000
  u_samples = 9000
  # t_samples = 10000
  # u_samples = 5000
  t_matrix = np.zeros((terrain_heights.shape[0], penetration_allowances.shape[0], t_samples))
  x_matrix = np.zeros((terrain_heights.shape[0], penetration_allowances.shape[0], t_samples, nx))
  t_u_matrix = np.zeros((terrain_heights.shape[0], penetration_allowances.shape[0], u_samples))
  u_matrix = np.zeros((terrain_heights.shape[0], penetration_allowances.shape[0], u_samples, nu))
  # folder_path = '/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/' + duration + '/'
  folder_path = '/home/yangwill/Documents/research/projects/cassie/sim/jumping/logs/param_studies/mujoco/' + duration + '/'


  # for i in range(terrain_heights.shape[0]):
  for i in range(threshold_durations.shape[0]):
    for j in range(penetration_allowances.shape[0]):
      # Mujoco logs
      log_suffix = 'duration_%.3f_mujoco' % threshold_durations[i]
      # Drake logs
      # log_suffix = 'height_%.4f-stiff_%.5f' % (terrain_heights[i], penetration_allowances[j])

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

  np.save(data_directory + 't_x_' + duration, t_matrix)
  np.save(data_directory + 'x_' + duration, x_matrix)
  np.save(data_directory + 't_u_' + duration, t_u_matrix)
  np.save(data_directory + 'u_' + duration, u_matrix)
  return


def count_successful_jumps(duration):
  t_matrix = np.load(data_directory + 't_x_' + duration + '.npy')
  x_matrix = np.load(data_directory + 'x_' + duration + '.npy')
  t_u_matrix = np.load(data_directory + 't_u_' + duration + '.npy')
  u_matrix = np.load(data_directory + 'u_' + duration + '.npy')

  success_height = 0.75
  z_fb_idx = 6
  successes = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  efforts = np.zeros((t_matrix.shape[0], t_matrix.shape[1]))
  u_slice = slice(0, 10)

  for i in range(terrain_heights.shape[0]):
    for j in range(penetration_allowances.shape[0]):
      t_idx = np.argwhere(t_matrix[i, j] == 3.0)[0,0]
      t_u_start_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time - 0.1)) < 2e-3)[0, 0]
      t_u_end_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time + 0.1)) < 2e-3)[0, 0]
      if(x_matrix[i, j, t_idx, z_fb_idx] > (success_height + terrain_heights[i])):
        successes[i, j] = 1
        t_u_slice = slice(t_u_start_idx, t_u_end_idx)
        efforts[i, j] = trapz(np.square(np.sum(u_matrix[i, j, t_u_slice, u_slice], axis=1)), t_u_matrix[i, j, t_u_slice])

  import pdb; pdb.set_trace()

  return


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
  ps.save_fig(figure_directory + 'knee_motor_bands.png')
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
      t_idx = np.argwhere(t_matrix[i, j] == 3.0)[0,0]
      t_u_start_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time - 0.25)) < 2e-3)[0, 0]
      t_u_end_idx = np.argwhere(np.abs(t_u_matrix[i, j] - (impact_time + 0.5)) < 2e-3)[0, 0]
      t_u_slice = slice(t_u_start_idx, t_u_end_idx)
      if i == 5 and j == 0:
        ps.plot(t_u_matrix[i, j, t_u_slice], np.sum(u_matrix[i, j, t_u_slice, 6:8], axis=1), linestyle=color, grid=False)

  low_idx = 0
  high_idx = 10
  t_u_start_idx = np.argwhere(np.abs(t_u_matrix[low_idx, 0] - (impact_time - 0.25)) < 2e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_u_matrix[low_idx, 0] - (impact_time + 0.5)) < 2e-3)[0, 0]
  t_u_low_slice = slice(t_u_start_idx, t_u_end_idx)
  t_u_start_idx = np.argwhere(np.abs(t_u_matrix[high_idx, 0] - (impact_time - 0.25)) < 2e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_u_matrix[high_idx, 0] - (impact_time + 0.5)) < 2e-3)[0, 0]
  t_u_high_slice = slice(t_u_start_idx, t_u_end_idx)
  # mid = np.sum(u_matrix[4, 0, t_u_slice, 6:8], axis=1)
  lower_bound = np.sum(u_matrix[low_idx, 0, t_u_low_slice, 6:8], axis=1)
  upper_bound = np.sum(u_matrix[high_idx, 0, t_u_high_slice, 6:8], axis=1)
  ps.plot_bands(t_u_matrix[low_idx, 0, t_u_low_slice], t_u_matrix[high_idx, 0, t_u_high_slice], lower_bound, upper_bound, color=color)

if __name__ == '__main__':
  main()
