import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

from scipy.linalg import null_space

import dairlib

from bindings.pydairlib.lcm import lcm_trajectory
from bindings.pydairlib.multibody import MakeNameToPositionsMap, \
  MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
  CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap
from bindings.pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.common import plot_styler, plotting_utils
from bindings.pydairlib.cassie.cassie_utils import *
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
import pydairlib.analysis.mbp_plotting_utils as mbp_plots

from pydrake.trajectories import PiecewisePolynomial
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder


def blend_sigmoid(t, tau, window):
  x = (t + window) / tau
  return np.exp(x) / (1 + np.exp(x))

def blend_function(t, tau, window, blend_type = 'sigmoid'):
  if blend_type == 'sigmoid':
    return blend_sigmoid(t, tau, window)
  elif blend_type == 'binary':
    return 1
  else:
    return 0

def load_logs(plant, t_impact, window):
  log_dir = '/home/yangwill/Documents/research/papers/impact_invariant_control/data/five_link_biped/'
  filename = 'lcmlog-error'
  print(log_dir + filename)
  log = lcm.EventLog(log_dir + filename, "r")
  default_channels = {'RABBIT_STATE': dairlib.lcmt_robot_output,
                      'RABBIT_INPUT': dairlib.lcmt_robot_input,
                      'OSC_DEBUG_WALKING': dairlib.lcmt_osc_output}
  callback = mbp_plots.load_default_channels
  start_time = 0
  duration = -1
  robot_output, robot_input, osc_debug = \
    get_log_data(log, default_channels, start_time, duration, callback,
                 plant,
                 'RABBIT_STATE', 'RABBIT_INPUT', 'OSC_DEBUG_WALKING')
  return robot_output, robot_input, osc_debug

def main():
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    "examples/impact_invariant_control/five_link_biped.urdf")
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
  plant.Finalize()
  context = plant.CreateDefaultContext()
  world = plant.world_frame()

  l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
  r_contact_frame = plant.GetBodyByName("right_foot").body_frame()

  pos_map = MakeNameToPositionsMap(plant)
  vel_map = MakeNameToVelocitiesMap(plant)
  act_map = MakeNameToActuatorsMap(plant)

  state_names = CreateStateNameVectorFromMap(plant)
  state_legend = [s[:-4].replace('_', ' ').title() for s in state_names]

  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()
  nc = 2
  ny = 5
  pt_on_body = np.zeros(3)
  blend_type = 'sigmoid'
  use_blending_window = False

  n_samples = 1000
  window_length = 0.05
  tau = 0.002
  joint_vel_indices = slice(3, 7)
  selected_joint_indices = slice(3, 7)

  # Map to 2d
  TXZ = np.array([[1, 0, 0], [0, 0, 1]])

  filename = "examples/impact_invariant_control/saved_trajectories/rabbit_walking"
  dircon_traj = lcm_trajectory.DirconTrajectory(plant, filename)
  state_traj = dircon_traj.ReconstructStateTrajectory()

  transition_time = dircon_traj.GetStateBreaks(1)[0]
  x_pre = state_traj.value(transition_time - 1e-6)
  x_post = state_traj.value(transition_time)

  robot_output, robot_input, osc_debug = load_logs(plant, transition_time, window_length)


  start_time = state_traj.start_time()
  end_time = 0.5 * state_traj.end_time()
  t = np.linspace(start_time, end_time, 1000)
  t_impact = dircon_traj.GetStateBreaks(0)[-1]
  t_proj = np.array(
    [t_impact[0] - 1.5 * window_length, t_impact[0] + 1.5*window_length])

  vel_desired = np.zeros((t.shape[0], nv))
  vel_actual = np.zeros((t.shape[0], nv))
  vel_proj_desired = np.zeros((t.shape[0], nv))
  vel_proj_actual = np.zeros((t.shape[0], nv))
  taskspace_vel_actual = np.zeros((t.shape[0], ny))
  taskspace_vel_desired = np.zeros((t.shape[0], ny))
  taskspace_vel_error = np.zeros((t.shape[0], ny))
  taskspace_vel_actual_proj = np.zeros((t.shape[0], ny))
  taskspace_vel_desired_proj = np.zeros((t.shape[0], ny))
  taskspace_vel_error_proj = np.zeros((t.shape[0], ny))
  taskspace_vel_error_osc = np.zeros((t.shape[0], ny))
  vel_corrected = np.zeros((t.shape[0], nv))
  vel_correction = np.zeros((t.shape[0], nv))
  vel_corrected_blend = np.zeros((t.shape[0], nv))
  alphas = np.zeros((t.shape[0], 1))
  start_idx = np.argwhere(np.abs(robot_output['t_x'] - start_time) < 1e-3)[0][0]
  end_idx = np.argwhere(np.abs(robot_output['t_x'] - end_time) < 1e-3)[0][0]

  for i in range(t.shape[0]):
    x_des = state_traj.value(robot_output['t_x'][i + start_idx])[:, 0]
    plant.SetPositions(context, x_des[:nq])
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV,
                                                        r_contact_frame,
                                                        pt_on_body, world,
                                                        world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV,
                                                        l_contact_frame,
                                                        pt_on_body, world,
                                                        world)
    # choose the correct contact jacobian depending on the time
    if t[i] <= end_time:
      TV_J = M_inv @ J_r.T
    else:
      TV_J = M_inv @ J_l.T
    TV_J_space = null_space(TV_J.T).T
    TV_proj = TV_J_space.T @ TV_J_space

    alpha = 0
    if (np.abs(t[i] - transition_time) < 1.5 * window_length):
      if (t[i] < transition_time):
        alpha = blend_function(t[i] - transition_time, tau,
                              window_length, blend_type)
      else:
        alpha = blend_function(transition_time - t[i], tau,
                              window_length, blend_type)
    alphas[i] = alpha

    vel_desired[i] = x_des[-nv:]
    if not use_blending_window:
      alpha = 1
    vel_proj_desired[i] = alpha * TV_proj @ vel_desired[i] + (1 - alpha) * vel_desired[i]


  joint_vel_plot = plot_styler.PlotStyler()
  joint_vel_plot.plot(t, vel_desired[:, selected_joint_indices], xlabel='Time (s)', ylabel='Velocity (rad/s)')
  joint_vel_plot.add_legend(state_legend[selected_joint_indices])
  joint_vel_plot.tight_layout()
  joint_vel_plot.save_fig('rabbit_gen_vel.png')

  proj_vel_plot = plot_styler.PlotStyler()
  ylim = joint_vel_plot.fig.gca().get_ylim()
  proj_vel_plot.plot(t, vel_proj_desired[:, selected_joint_indices],
                     xlabel='Time (s)', ylabel='Velocity (rad/s)', ylim=ylim)
  proj_vel_plot.add_legend(state_legend[selected_joint_indices])
  proj_vel_plot.tight_layout()

  # ax = proj_vel_plot.fig.axes[0]
  # ax.fill_between(t_proj, joint_vel_plot.fig.axes[0].get_ylim()[0], joint_vel_plot.fig.axes[0].get_ylim()[1],
  #                 color=proj_vel_plot.grey, alpha=0.2)
  proj_vel_plot.save_fig('proj_vel_plot.png')

  gen_vel_plot = plot_styler.PlotStyler()
  gen_vel_plot.plot(t, vel_desired[:, selected_joint_indices],
                    xlabel='Time (s)', ylabel='Velocity (rad/s)')
  gen_vel_plot.plot(t, vel_actual[:, selected_joint_indices],
                    xlabel='Time (s)', ylabel='Velocity (rad/s)')
  # gen_vel_plot.plot(t, vel_desired[:, selected_joint_indices] - vel_actual[:, selected_joint_indices],
  #                   xlabel='time (s)', ylabel='velocity (m/s)', grid=False)
  # gen_vel_plot.add_legend(['Desired Velocity', 'Measured Velocity'])
  ylim = gen_vel_plot.fig.gca().get_ylim()
  gen_vel_plot.save_fig('gen_vel_plot.png')
  # ps.save_fig('generalized_velocities_around_impact.png')



  corrected_vel_plot = plot_styler.PlotStyler()
  corrected_vel_plot.plot(t, vel_corrected,
                          title='Impact-Invariant Correction',
                          xlabel='time (s)', ylabel='velocity (m/s)',
                          ylim=ylim)
  # corrected_vel_plot.save_fig('corrected_vel_plot.png')

  blended_vel_plot = plot_styler.PlotStyler()
  ax = blended_vel_plot.fig.axes[0]
  # blended_vel_plot.plot(t, vel_actual[:, selected_joint_indices],
  #                       title='Blended Impact-Invariant Correction',
  #                       xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
  blended_vel_plot.plot(t, vel_corrected_blend[:, selected_joint_indices],
                        title='Blended Impact-Invariant Correction',
                        xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
  ax.fill_between(t_proj, ylim[0], ylim[1], color=blended_vel_plot.grey,
                  alpha=0.2)
  # blended_vel_plot.save_fig('blended_vel_plot.png')



  gen_vel_error = plot_styler.PlotStyler()
  ax = gen_vel_error.fig.axes[0]
  gen_vel_error.plot(t, vel_desired[:, selected_joint_indices],
                     xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.blue)
  gen_vel_error.plot(t, vel_actual[:, selected_joint_indices],
                     xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.red)
  gen_vel_error.plot(t, vel_desired[:, selected_joint_indices] - vel_actual[:, selected_joint_indices],
                     xlabel='Time', ylabel='Velocity', grid=False, color=gen_vel_error.grey)
  gen_vel_error.add_legend(['Desired Velocity', 'Measured Velocity', 'Velocity Error'])
  ax.set_yticklabels([])
  ax.set_xticklabels([])
  # ax.spines['bottom'].set_visible(False)
  # ax.spines['left'].set_visible(False)
  ylim = gen_vel_error.fig.gca().get_ylim()
  # gen_vel_error.save_fig('gen_vel_error.png')

  projected_vel_error = plot_styler.PlotStyler()
  ax = projected_vel_error.fig.axes[0]
  projected_vel_error.plot(t, vel_proj_desired[:, selected_joint_indices] - vel_proj_actual[:, selected_joint_indices],
                           title='Impact-Invariant Projection Error',
                           xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
  ax.fill_between(t_proj, ylim[0], ylim[1], color=projected_vel_error.grey,
                  alpha=0.2)
  # projected_vel_error.save_fig('projected_vel_error.png')

  corrected_vel_error = plot_styler.PlotStyler()
  ax = corrected_vel_error.fig.axes[0]
  corrected_vel_error.plot(t, vel_desired[:, selected_joint_indices] - vel_corrected[:, selected_joint_indices],
                           title='Impact-Invariant Correction Error',
                           xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
  ax.fill_between(t_proj, ylim[0], ylim[1], color=corrected_vel_error.grey,
                  alpha=0.2)

  taskspace_error_plot = plot_styler.PlotStyler()
  ax = taskspace_error_plot.fig.axes[0]
  taskspace_error_plot.plot(t, taskspace_vel_actual,
                            title='Task Space',
                            xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.blue)
  taskspace_error_plot.plot(t, taskspace_vel_actual_proj,
                            title='Task Space',
                            xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.red)
  # taskspace_error_plot.plot(t, taskspace_vel_error_osc,
  #                          title='Task Space',
  #                          xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.grey)
  # ax.fill_between(t_proj, ylim[0], ylim[1], color=corrected_vel_error.grey,
  #                 alpha=0.2)
  # corrected_vel_error.save_fig('corrected_vel_error.png')

  blending_function_plot = plot_styler.PlotStyler()
  ax = blending_function_plot.fig.axes[0]
  blending_function_plot.plot(t, alphas, title="Blending Function")
  ax.fill_between(t_proj, ax.get_ylim()[0], ax.get_ylim()[1],
                  color=blending_function_plot.grey, alpha=0.2)
  blending_function_plot.save_fig('blending_function_plot.png')

  plt.show()


if __name__ == '__main__':
  main()
