import sys

import lcm
import dairlib
import drake
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy import integrate
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.trajectories import PiecewisePolynomial
from IPython import get_ipython

from pydairlib.common import FindResourceOrThrow
from pydairlib.common import plot_styler
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.lcm import lcm_trajectory
from pydairlib.lcm import process_lcm_log
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.cassie.cassie_utils import *
import pydairlib.multibody

import matplotlib.pyplot as plt
import matplotlib.animation as animation


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
  global log_dir
  global vel_map
  global pos_map
  global act_map
  global t_start
  global t_end
  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  # figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/journal_version/figures/rabbit/'
  ps = plot_styler.PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  plt.close()

  TXZ = np.array([[1, 0, 0], [0, 0, 1]])

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(FindResourceOrThrow("examples/impact_invariant_control/five_link_biped.urdf"))
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
  plant.Finalize()

  filename = "/home/yangwill/Documents/research/projects/five_link_biped/walking/saved_trajs/rabbit_walking"
  dircon_traj = lcm_trajectory.DirconTrajectory(filename)
  state_traj = dircon_traj.ReconstructStateTrajectory()
  input_traj = dircon_traj.ReconstructInputTrajectory()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)


  nominal_impact_time = dircon_traj.GetStateBreaks(1)[0]
  t_impact = nominal_impact_time + 0.01
  t_start = nominal_impact_time - 0.05
  # t_start = nominal_impact_time - 0.1
  t_end = nominal_impact_time + 0.06

  # filename = sys.argv[1]
  # log_dir = '/home/yangwill/Documents/research/projects/five_link_biped/invariant_impacts/logs_for_iros_2021_paper/'
  log_dir = '/home/yangwill/Documents/research/projects/five_link_biped/invariant_impacts/logs/'
  # log_files = ['lcmlog-0001', 'lcmlog-050', 'lcmlog-1001']
  log_files = ['lcmlog-000', 'lcmlog-025_0KD', 'lcmlog-025']
  # log_files = ['lcmlog-000', 'lcmlog-025']
  # log_files = ['lcmlog-000_1', 'lcmlog-010', 'lcmlog-010_0KD_1']
  # log_files = ['lcmlog-0251']
  # log_files = ['lcmlog-0101']
  colors = [ps.blue, ps.orange, ps.red]
  linestyles = ['-', 'dotted', '--', '-.']
  color_idx = 0

  t_samples = np.arange(0, 0.5, 0.00025)
  x_nominal = np.zeros((t_samples.shape[0], 14))
  u_nominal = np.zeros((t_samples.shape[0], 4))
  for i in range(t_samples.shape[0]):
    x_nominal[i, :] = state_traj.value(t_samples[i])[:, 0]
    u_nominal[i, :] = input_traj.value(t_samples[i])[:, 0]

  # plt.figure("inputs")
  # ps.plot(t_samples, u_nominal[:, -4:])
  # ps.add_legend(u_datatypes[-4:])

  # animate_velocity_error(plant)
  plot_ablation_study()

  ps.show_fig()

def animate_velocity_error(plant):
  filename = 'lcmlog-025_0KD'
  log = lcm.EventLog(log_dir + filename, "r")
  x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
  t_contact_info, osc_debug, fsm, estop_signal, \
  switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, \
  t_u_pd, osc_output, full_log, t_lcmlog_u = \
    process_lcm_log.process_log(log, pos_map, vel_map, act_map, 'RABBIT_INPUT')

  t_u_start_idx = np.argwhere(np.abs(t_u - (t_start)) < 1e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_u - (t_end)) < 1e-3)[0, 0]
  t_slice = slice(t_u_start_idx, t_u_end_idx)

  x_range = [-50, 50]
  y_range = [-6, 0]

  fig = plt.figure()
  ax = plt.axes()
  n_frames = 10
  ax.set_xlim(x_range)
  ax.set_ylim(y_range)
  ps.plot(1e3*(t_u[t_slice] - nominal_impact_time), osc_debug['right_knee_pin_traj'].ydot_des[t_slice], xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity (rad/s)', color=ps.blue)
  ps.add_legend(['Reference Trajectory'])


  def frame(i):
    # ax.clear()

    ax.set_xlim(x_range)
    ax.set_ylim(y_range)

    i = int(i / n_frames * t_u[t_slice].shape[0])

    plot = ps.plot(1e3*(t_u[t_slice][i] - nominal_impact_time), osc_debug['right_knee_pin_traj'].ydot[t_slice][i], color=ps.red)

    # if not ('google.colab' in str(get_ipython())):
    #   plt.draw()
    #   plt.pause(t_osc[t_slice][-1]/n_frames)
    return plot

  # if not ('google.colab' in str(get_ipython())):
  plt.ion()
  plt.show()
  for i in range(n_frames):
    frame(i)

  anim = animation.FuncAnimation(fig, frame, frames=n_frames, blit=False, repeat=False)

  # plt.close()
  writervideo = animation.FFMpegWriter(fps=60)
  anim.save(ps.directory + 'actual_velocity_right_knee.mp4', writervideo)
  # import pdb; pdb.set_trace()
  ps.show_fig()

def plot_soft_constraint():
  log_files = ['lcmlog-025', 'lcmlog-025_2', 'lcmlog-025_3']
  colors = [ps.blue, ps.orange, ps.red]
  linestyles = ['-', 'dotted', '--', '-.']
  color_idx = 0

  for filename in log_files:
    log = lcm.EventLog(log_dir + filename, "r")
    x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
    t_contact_info, osc_debug, fsm, estop_signal, \
    switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, \
    t_u_pd, osc_output, full_log, t_lcmlog_u = \
      process_lcm_log.process_log(log, pos_map, vel_map, act_map, 'RABBIT_INPUT')

    t_u_start_idx = np.argwhere(np.abs(t_u - (t_start)) < 1e-3)[0, 0]
    t_u_end_idx = np.argwhere(np.abs(t_u - (t_end)) < 1e-3)[0, 0]
    t_slice = slice(t_u_start_idx, t_u_end_idx)


    # plt.figure("velocities")
    # ps.plot(t_x, x[:, -7:], color=colors[color_idx])

    lambdas = []
    soft_constraint_costs = []
    for osc in osc_output:
      qp_output = osc.qp_output
      # soft_constraint_costs.append(osc.soft_constraint_cost)
      soft_constraint_costs.append(osc.fsm_state)
      # lambdas.append(qp_output.lambda_c_sol)
      lambdas.append(qp_output.dv_sol)
    # import pdb; pdb.set_trace()
    lambdas = np.array(lambdas)

def plot_ablation_study():
  global log_dir
  log_dir = log_dir + 'early_impact/'

  # log_files = ['lcmlog-000', 'lcmlog-025_0KD', 'lcmlog-025']

  # log_files = ['lcmlog-000', 'lcmlog-050-iros21', 'lcmlog-050-proj_soft_constraint']
  log_files = ['lcmlog-000', 'lcmlog-025-iros21', 'lcmlog-025-proj_soft_constraint']
  # log_files = ['lcmlog-000', 'lcmlog-010-iros21', 'lcmlog-010-proj_soft_constraint']

  # log_files = ['lcmlog-025', 'lcmlog-025_1', 'lcmlog-025_2']
  # log_files = ['lcmlog-025', 'lcmlog-025_2', 'lcmlog-025_3']
  # log_files = ['lcmlog-025']
  # log_files = ['lcmlog-000_1', 'lcmlog-010', 'lcmlog-010_0KD_1']
  colors = [ps.blue, ps.orange, ps.red]
  linestyles = ['-', 'dotted', '--', '-.']
  color_idx = 0

  for filename in log_files:
    log = lcm.EventLog(log_dir + filename, "r")
    x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
    t_contact_info, osc_debug, fsm, estop_signal, \
    switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, \
    t_u_pd, osc_output, full_log, t_lcmlog_u = \
      process_lcm_log.process_log(log, pos_map, vel_map, act_map, 'RABBIT_INPUT')

    t_u_start_idx = np.argwhere(np.abs(t_u - (t_start)) < 1e-3)[0, 0]
    t_u_end_idx = np.argwhere(np.abs(t_u - (t_end)) < 1e-3)[0, 0]
    t_slice = slice(t_u_start_idx, t_u_end_idx)

    # TODO: clean up these plotting scripts
    # Have options to plot anything in the paper

    controller_types = [
      'Default Controller',
      'Impact Invariant Projection',
      'Impact Invariant Projection with Projected Soft Constraints']

    error_type = 'actual'
    # error_type = 'projected'

    plot_type = 'default'
    # plot_type = 'L1'
    # plot_type = 'absolute'

    t_from_impact = 1e3*(t_u[t_slice] - nominal_impact_time)

    plt.figure("inputs")
    for i, linestyle in enumerate(linestyles):
      ps.plot(t_from_impact, u[t_slice, i], xlabel='Time Since Nominal Impact (ms)', ylabel='Controller Efforts (Nm)', color=colors[color_idx], linestyle=linestyle)
    # ps.plot(1e3*(t_u[t_slice] - nominal_impact_time), u[t_slice, 3], xlabel='Time Since Impact (ms)', ylabel='Motor Effort (Nm)', color=colors[color_idx])


    plt.figure("swing_leg_joints")
    right_knee_pin_error_abs = np.abs(osc_debug['right_knee_pin_traj'].ydot_des[t_slice] - osc_debug['right_knee_pin_traj'].ydot[t_slice])
    right_hip_pin_error_abs = np.abs(osc_debug['right_hip_pin_traj'].ydot_des[t_slice] - osc_debug['right_hip_pin_traj'].ydot[t_slice])
    if error_type == "actual":
      right_knee_pin_error = osc_debug['right_knee_pin_traj'].ydot_des[t_slice] - osc_debug['right_knee_pin_traj'].ydot[t_slice]
      right_hip_pin_error = osc_debug['right_hip_pin_traj'].ydot_des[t_slice] - osc_debug['right_hip_pin_traj'].ydot[t_slice]
    elif error_type == 'projected':
      right_knee_pin_error = osc_debug['right_knee_pin_traj'].error_ydot
      right_hip_pin_error = osc_debug['right_hip_pin_traj'].error_ydot

    if plot_type == 'default':
      ps.plot(t_from_impact, right_knee_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')
      ps.plot(t_from_impact, right_hip_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')
    elif plot_type == 'L1':
      ps.plot(t_from_impact, right_knee_pin_error_abs + right_hip_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Norm Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')
    elif plot_type == 'absolute':
      ps.plot(t_from_impact, right_knee_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')
      ps.plot(t_from_impact, right_hip_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')

    plt.figure("stance_leg_joints")
    left_knee_pin_error_abs = np.abs(osc_debug['left_knee_pin_traj'].ydot_des[t_slice] - osc_debug['left_knee_pin_traj'].ydot[t_slice])
    left_hip_pin_error_abs = np.abs(osc_debug['left_hip_pin_traj'].ydot_des[t_slice] - osc_debug['left_hip_pin_traj'].ydot[t_slice])
    if error_type == "actual":
      left_knee_pin_error = osc_debug['left_knee_pin_traj'].ydot_des[t_slice] - osc_debug['left_knee_pin_traj'].ydot[t_slice]
      left_hip_pin_error = osc_debug['left_hip_pin_traj'].ydot_des[t_slice] - osc_debug['left_hip_pin_traj'].ydot[t_slice]
    elif error_type == 'projected':
      left_knee_pin_error = osc_debug['left_knee_pin_traj'].error_ydot
      left_hip_pin_error = osc_debug['left_hip_pin_traj'].error_ydot

    if plot_type == 'default':
      ps.plot(t_from_impact, left_knee_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')
      ps.plot(t_from_impact, left_hip_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')
    elif plot_type == 'L1':
      ps.plot(t_from_impact, left_knee_pin_error_abs + left_hip_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Norm Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')
    elif plot_type == 'absolute':
      ps.plot(t_from_impact, left_knee_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')
      ps.plot(t_from_impact, left_hip_pin_error_abs, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')

    color_idx += 1


  # Plotting settings
  legend_elements = [matplotlib.lines.Line2D([0], [0], color=colors[0], lw=3, label=controller_types[0]),
                     matplotlib.lines.Line2D([0], [0], color=colors[1], lw=3, label=controller_types[1]),
                     matplotlib.lines.Line2D([0], [0], color=colors[2], lw=3, label=controller_types[2]),
                     matplotlib.patches.Patch(facecolor=ps.grey, label='Projection Window')]


  plt.figure("stance_leg_joints")
  ax = plt.gca()
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)
  ax.legend(handles = legend_elements, loc=2, edgecolor='k')
  plt.xlim([-50, 50])
  # plt.ylim([0, 0.5])
  # ps.save_fig('stance_leg_velocity_error.png')

  plt.figure("swing_leg_joints")
  ax = plt.gca()
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)
  ax.legend(handles = legend_elements, loc=2, edgecolor='k')
  plt.xlim([-50, 50])
  # plt.ylim([0, 0.5])
  # ps.save_fig('swing_leg_velocity_error.png')

  plt.figure("inputs")
  ax = plt.gca()
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)
  plt.xlim([-50, 50])

  controller_legend = ax.legend(handles = legend_elements, loc=2, edgecolor='k')
  ax.add_artist(controller_legend)
  legend_elements_lines = [matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[0], lw=3, label='Hip (Non-Impacting Leg)'),
                                                    matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[1], lw=3, label='Hip (Impacting Leg)'),
                                                    matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[2], lw=3, label='Knee (Non-Impacting Leg)'),
                                                    matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[3], lw=3, label='Knee (Impacting Leg)')]
  line_legend = ax.legend(handles = legend_elements_lines, loc='lower left', ncol=1, edgecolor='k')
  ax.add_artist(line_legend)
  # ps.save_fig('rabbit_controller_efforts.png')
  # ps.save_fig('impacting_knee_controller_effort.png')

  # plt.figure("legend")
  # ax = plt.gca()
  # legend_elements = [matplotlib.lines.Line2D([0], [0], color=colors[0], lw=3, label='Default Controller'),
  #                    matplotlib.lines.Line2D([0], [0], color=colors[2], lw=3, label='Impact Invariant Projection'),
  #                    matplotlib.patches.Patch(facecolor=ps.grey, label='Projection Window')]
  # legend_elements_lines = [matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[0], lw=3, label='Hip (Non-Impacting Leg)'),
  #                          matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[1], lw=3, label='Hip (Impacting Leg)'),
  #                          matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[2], lw=3, label='Knee (Non-Impacting Leg)'),
  #                          matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[3], lw=3, label='Knee (Impacting Leg)')]
  # line_legend = ax.legend(handles = legend_elements_lines, loc='lower left', ncol=1, edgecolor='k')
  # ax.add_artist(line_legend)
  # ax.legend(handles = legend_elements, loc=2, edgecolor='k')
  # ps.save_fig('rabbit_legend_for_video.png')

def plot_foot_velocities(plant, context, t_state, x):
  n_points = t_state.shape[0]
  nq = plant.num_positions()
  nv = plant.num_velocities()
  l_foot_points = np.zeros((t_state.shape[0], 4))
  r_foot_points = np.zeros((t_state.shape[0], 4))
  time_slice = get_time_slice(t_state, t_start, t_end)
  l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
  r_contact_frame = plant.GetBodyByName("right_foot").body_frame()
  world = plant.world_frame()
  pt_on_body = np.zeros(3)

  for i in range(n_points):
    plant.SetPositionsAndVelocities(context, x[i])
    J_l = plant.CalcJacobianTranslationalVelocity(context,
                                                  JacobianWrtVariable.kV,
                                                  l_contact_frame,
                                                  pt_on_body, world, world)
    J_r = plant.CalcJacobianTranslationalVelocity(context,
                                                  JacobianWrtVariable.kV,
                                                  r_contact_frame,
                                                  pt_on_body, world, world)
    l_foot_points[i, 0:2] = np.reshape(TXZ @ plant.CalcPointsPositions(
      context, l_contact_frame, pt_on_body, world), (2,))
    r_foot_points[i, 0:2] = np.reshape(TXZ @ plant.CalcPointsPositions(
      context, r_contact_frame, pt_on_body, world), (2,))
    l_foot_points[i, 2:4] = np.reshape(TXZ @ J_l @ x[i, -nv:], (2,))
    r_foot_points[i, 2:4] = np.reshape(TXZ @ J_r @ x[i, -nv:], (2,))
  fig = plt.figure("Foot positions (simulation): " + filename)
  # plt.plot(t_state, l_foot_points[:, 0:2])
  # plt.plot(t_state[time_slice], r_foot_points[time_slice, 0:2])
  # plt.plot(t_state[time_slice], r_foot_points[time_slice, 2:4])
  plt.plot(t_state[time_slice], r_foot_points[time_slice, 3])

  # plt.legend(["right foot ($\dot x$)",
  #             "right foot ($\dot z$)",
  #             "right foot ($\dot x$)",
  #             "right foot ($\dot z$)"])
  plt.legend(["$\dot z$ right foot 1e-5",
              "$\dot z$ right foot 2e-4"])
  plt.xlabel("time (s)")
  plt.ylabel("velocity (m/s)")


def plot_optimal_controller_efforts(plant, t_nominal, u_traj_nominal,
                                    datatypes):
  n_points = 1000
  sampled_points = np.zeros((1000, plant.num_actuators()))
  t_points = np.linspace(t_nominal[0], t_nominal[-1], n_points)
  time_slice = get_time_slice(t_points, t0_, tf_)
  for i in range(n_points):
    sampled_points[i, :] = u_traj_nominal.value(t_points[i])[:, 0]
  fig = plt.figure("Optimal controller efforts: " + filename)
  plt.plot(t_points[time_slice], sampled_points[time_slice])
  plt.legend(datatypes)


def plot_controller_efforts(plant, t_lqr, control_inputs, datatypes, linespec):
  # fig = plt.figure("Controller efforts: " + filename)
  fig = plt.figure("Controller efforts: ")
  time_slice = get_time_slice(t_lqr, t0_, tf_)
  plt.plot(t_lqr[time_slice], control_inputs[time_slice], linespec)
  plt.xlabel("time (s)")
  plt.ylabel("torque (Nm)")
  plt.legend(datatypes)


# def plot_simulation_state(plant, times, x, datatypes, label):
#   # fig = plt.figure("Simulation State: " + filename)
#   time_slice = get_time_slice(times, t0_, tf_)
#   plt.plot(times[time_slice], x[time_slice, state_slice], label=label)
#   plt.legend(datatypes[state_slice])
#   plt.xlabel("time (s)")
#   plt.ylabel("velocity (rad/s)")


if __name__ == "__main__":
  main()
