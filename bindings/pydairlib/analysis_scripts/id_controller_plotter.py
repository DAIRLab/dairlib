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
import pydairlib.lcm_trajectory
from IPython import get_ipython

from pydairlib.common import FindResourceOrThrow
from pydairlib.parameter_studies.plot_styler import PlotStyler
from pydrake.trajectories import PiecewisePolynomial
import pydairlib.lcm_trajectory
from bindings.pydairlib.analysis_scripts.process_log import process_log
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
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  TXZ = np.array([[1, 0, 0], [0, 0, 1]])

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(FindResourceOrThrow("examples/five_link_biped/five_link_biped.urdf"))
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
  plant.Finalize()

  filename = "/home/yangwill/Documents/research/projects/five_link_biped/walking/saved_trajs/rabbit_walking"
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)
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
  log_dir = '/home/yangwill/Documents/research/projects/five_link_biped/invariant_impacts/'
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
  t_state, t_osc, x, u, fsm, contact_info, contact_info_locs, osc_debug, osc_output, t_lcmlog_u = \
    process_log(log, pos_map, vel_map)

  t_u_start_idx = np.argwhere(np.abs(t_osc - (t_start)) < 1e-3)[0, 0]
  t_u_end_idx = np.argwhere(np.abs(t_osc - (t_end)) < 1e-3)[0, 0]
  t_slice = slice(t_u_start_idx, t_u_end_idx)

  x_range = [-50, 50]
  y_range = [-6, 0]

  fig = plt.figure()
  ax = plt.axes()
  n_frames = 10
  ax.set_xlim(x_range)
  ax.set_ylim(y_range)
  ps.plot(1e3*(t_osc[t_slice] - nominal_impact_time), osc_debug['right_knee_pin_traj'].ydot_des[t_slice], xlabel='Time Since Nominal Impact (ms)', ylabel='Joint Velocity (rad/s)', color=ps.blue)
  ps.add_legend(['Reference Trajectory'])


  def frame(i):
    # ax.clear()

    ax.set_xlim(x_range)
    ax.set_ylim(y_range)

    i = int(i / n_frames * t_osc[t_slice].shape[0])

    plot = ps.plot(1e3*(t_osc[t_slice][i] - nominal_impact_time), osc_debug['right_knee_pin_traj'].ydot[t_slice][i], color=ps.red)

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
  import pdb; pdb.set_trace()
  ps.show_fig()

def plot_ablation_study():
  log_files = ['lcmlog-000', 'lcmlog-025_0KD', 'lcmlog-025']
  # log_files = ['lcmlog-000', 'lcmlog-025']
  # log_files = ['lcmlog-000_1', 'lcmlog-010', 'lcmlog-010_0KD_1']
  # log_files = ['lcmlog-0251']
  # log_files = ['lcmlog-0101']
  colors = [ps.blue, ps.orange, ps.red]
  linestyles = ['-', 'dotted', '--', '-.']
  color_idx = 0

  for filename in log_files:
    log = lcm.EventLog(log_dir + filename, "r")
    t_state, t_osc, x, u, fsm, contact_info, contact_info_locs, osc_debug, osc_output, t_lcmlog_u = \
      process_log(log, pos_map, vel_map)
    # import pdb; pdb.set_trace()

    plt.figure("velocities")
    ps.plot(t_state, x[:, -7:], color=colors[color_idx])

    # ps.plot(t_osc, u)
    # ps.add_legend(u_datatypes)

    t_u_start_idx = np.argwhere(np.abs(t_osc - (t_start)) < 1e-3)[0, 0]
    t_u_end_idx = np.argwhere(np.abs(t_osc - (t_end)) < 1e-3)[0, 0]
    t_slice = slice(t_u_start_idx, t_u_end_idx)

    plt.figure("inputs")
    for i, linestyle in enumerate(linestyles):
      ps.plot(1e3*(t_osc[t_slice] - nominal_impact_time), u[t_slice, i], xlabel='Time Since Nominal Impact (ms)', ylabel='Controller Efforts (Nm)', color=colors[color_idx], linestyle=linestyle)

    plt.figure("swing_leg_joints")
    # ps.plot(t_osc, osc_debug['right_knee_pin_traj'].ydot, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['right_knee_pin_traj'].ydot_des, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['right_knee_pin_traj'].error_ydot, linestyle='r')
    right_knee_pin_error = np.abs(osc_debug['right_knee_pin_traj'].ydot_des[t_slice] - osc_debug['right_knee_pin_traj'].ydot[t_slice])
    # ps.plot(1e3*(t_osc[t_slice] - t_start), right_knee_pin_error, xlabel='Time After Impact (ms)', ylabel='Velocity Error (m/s)', linestyle=colors[color_idx])
    # plt.figure("right_hip")
    # ps.plot(t_osc, osc_debug['right_hip_pin_traj'].ydot, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['right_hip_pin_traj'].ydot_des, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['right_hip_pin_traj'].error_ydot, linestyle='r')
    right_hip_pin_error = np.abs(osc_debug['right_hip_pin_traj'].ydot_des[t_slice] - osc_debug['right_hip_pin_traj'].ydot[t_slice])
    ps.plot(1e3*(t_osc[t_slice] - nominal_impact_time), right_knee_pin_error + right_hip_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Norm Joint Velocity Error (rad/s)', color=colors[color_idx], title='Impacting Leg')
    plt.figure("stance_leg_joints")
    # ps.plot(t_osc, osc_debug['left_knee_pin_traj'].ydot, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['left_knee_pin_traj'].ydot_des, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['left_knee_pin_traj'].error_ydot, linestyle='r')
    left_knee_pin_error = np.abs(osc_debug['left_knee_pin_traj'].ydot_des[t_slice] - osc_debug['left_knee_pin_traj'].ydot[t_slice])
    # ps.plot(1e3*(t_osc[t_slice] - t_start), left_knee_pin_error, xlabel='Time After Impact (ms)', ylabel='Velocity Error (m/s)', linestyle=colors[color_idx])
    # plt.figure("left_hip")
    # ps.plot(t_osc, osc_debug['left_hip_pin_traj'].ydot, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['left_hip_pin_traj'].ydot_des, linestyle=colors[color_idx])
    # ps.plot(t_osc, osc_debug['left_hip_pin_traj'].error_ydot, linestyle='r')
    left_hip_pin_error = np.abs(osc_debug['left_hip_pin_traj'].ydot_des[t_slice] - osc_debug['left_hip_pin_traj'].ydot[t_slice])
    ps.plot(1e3*(t_osc[t_slice] - nominal_impact_time), left_knee_pin_error + left_hip_pin_error, xlabel='Time Since Nominal Impact (ms)', ylabel='L1 Norm Joint Velocity Error (rad/s)', color=colors[color_idx], title='Non-Impacting Leg')
    color_idx += 1

  plt.figure("stance_leg_joints")
  ax = plt.gca()
  # projection_window_a = matplotlib.patches.Rectangle((50, 100), 40, 30, edgecolor=ps.grey, facecolor=ps.grey, alpha=1.0)
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)
  # ps.add_legend(['Default Controller', 'No Derivative Feedback', 'Impact Invariant Projection', 'Projection Window'], loc=3)
  # ax.add_patch(projection_window_a)
  plt.xlim([-50, 50])
  plt.ylim([0, 0.5])
  ps.save_fig('stance_leg_velocity_error.png')

  plt.figure("swing_leg_joints")
  ax = plt.gca()
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)
  # ps.add_legend(['Default Controller', 'No Derivative Feedback', 'Impact Invariant Projection', 'Projection Window'], loc=3)
  # projection_window_b = matplotlib.patches.Rectangle((50, 100), 40, 30, edgecolor=ps.grey, facecolor=ps.grey, alpha=1.0)
  # ax.add_patch(projection_window_b)
  plt.xlim([-50, 50])
  plt.ylim([0, 0.5])
  ps.save_fig('swing_leg_velocity_error.png')
  plt.figure("inputs")
  ax = plt.gca()
  ax.axvspan(-25, 25, alpha=0.5, color=ps.grey)



  # ps.add_legend(u_datatypes)
  plt.xlim([-50, 50])
  # ps.save_fig('rabbit_controller_efforts.png')

  plt.figure("legend")
  ax = plt.gca()
  legend_elements = [matplotlib.lines.Line2D([0], [0], color=colors[0], lw=3, label='Default Controller'),
                     matplotlib.lines.Line2D([0], [0], color=colors[2], lw=3, label='Impact Invariant Projection'),
                     matplotlib.patches.Patch(facecolor=ps.grey, label='Projection Window')]
  legend_elements_lines = [matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[0], lw=3, label='Hip (Non-Impacting Leg)'),
                           matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[1], lw=3, label='Hip (Impacting Leg)'),
                           matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[2], lw=3, label='Knee (Non-Impacting Leg)'),
                           matplotlib.lines.Line2D([0], [0], color='k', linestyle=linestyles[3], lw=3, label='Knee (Impacting Leg)')]
  line_legend = ax.legend(handles = legend_elements_lines, loc='lower left', ncol=1, edgecolor='k')
  ax.add_artist(line_legend)
  ax.legend(handles = legend_elements, loc=2, edgecolor='k')
  ps.save_fig('rabbit_legend_for_video.png')

def plot_foot_velocities(plant, context, t_state, x):
  n_points = t_state.shape[0]
  nq = plant.num_positions()
  nv = plant.num_velocities()
  l_foot_points = np.zeros((t_state.shape[0], 4))
  r_foot_points = np.zeros((t_state.shape[0], 4))
  time_slice = get_time_slice(t_state, t0_, tf_)
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
