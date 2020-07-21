import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import process_lcm_log
from numpy.linalg import norm
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from load_lcm_trajs import load_lcm_trajs


def main():
  matplotlib.rcParams["savefig.directory"] = \
    "/home/yangwill/Documents/research/projects/cassie/walking/analysis" \
    "/figures/"

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  plant.mutable_gravity_field().set_gravity_vector(

    -9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # relevant MBP parameters
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant)

  n_modes = 3
  folder_path = "/home/yangwill/Documents/research/projects/cassie/walking/saved_trajs/"
  trajectory_name = "walking_0.5_processed"
  left_foot_mode_name = "left_foot_trajectory"
  right_foot_mode_name = "right_foot_trajectory"
  pelvis_rot_mode_name = "pelvis_rot_trajectory"
  com_mode_name = "center_of_mass_trajectory"
  # mode_name = "pelvis_rot_trajectory"
  # x_traj_nominal, x_hybrid_trajs_nominal, u_traj_nominal, \
  # u_hybrid_trajs_nominal, decision_vars, datatypes \
  #   = load_lcm_trajs(37, nu, n_modes, folder_path, trajectory_name, mode_name)

  filename = sys.argv[1]
  log = lcm.EventLog(filename, "r")

  x, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, osc_debug, fsm, estop_signal, \
  switch_signal, t_controller_switch = process_lcm_log.process_log(log, pos_map, vel_map)

  plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, estop_signal)

  lcm_trajectory = pydairlib.lcm_trajectory.LcmTrajectory()
  lcm_trajectory.loadFromFile(folder_path + trajectory_name)
  l_foot_points = lcm_trajectory.getTrajectory(left_foot_mode_name).datapoints
  t_points_l_foot = lcm_trajectory.getTrajectory(left_foot_mode_name).time_vector
  l_foot_trajectory = PiecewisePolynomial.CubicHermite(t_points_l_foot, l_foot_points[:3], l_foot_points[3:6])
  r_foot_points = lcm_trajectory.getTrajectory(right_foot_mode_name).datapoints
  t_points_r_foot = lcm_trajectory.getTrajectory(right_foot_mode_name).time_vector
  r_foot_trajectory = PiecewisePolynomial.CubicHermite(t_points_r_foot, r_foot_points[:3], r_foot_points[3:6])
  com_points = lcm_trajectory.getTrajectory(com_mode_name).datapoints
  t_points_com = lcm_trajectory.getTrajectory(com_mode_name).time_vector
  com_trajectory = PiecewisePolynomial.CubicHermite(t_points_com, com_points[:3], com_points[3:6])
  pelvis_rot_points = lcm_trajectory.getTrajectory(pelvis_rot_mode_name).datapoints
  t_points_pelvis_rot = lcm_trajectory.getTrajectory(pelvis_rot_mode_name).time_vector
  # pelvis_rot_trajectory = PiecewisePolynomial.FirstOrderHold(t_points_pelvis_rot, pelvis_rot_points[:4])
  pelvis_rot_trajectory = PiecewisePolynomial.CubicHermite(t_points_pelvis_rot, pelvis_rot_points[:4],
                                                           pelvis_rot_points[-4:])

  # plt.figure("Contact info")
  # plt.plot(t_contact_info, contact_info[0, :, 2])
  # plt.plot(t_contact_info, contact_info[1, :, 2])
  # plt.plot(t_contact_info, contact_info[2, :, 2])
  # plt.plot(t_contact_info, contact_info[3, :, 2])
  # plt.legend(["lfoot_rear", "lfoot_front", "rfoot_rear", "rfoot_front"])
  # plot_nominal_output_trajectories(t_points_com, com_trajectory, pelvis_rot_trajectory, l_foot_trajectory,
  #                                  r_foot_trajectory)
  # plot_osc_debug(t_u, fsm, osc_debug)
  plt.show()


def plot_nominal_output_trajectories(t_points, com_trajectory, pelvis_rot_trajectory, l_foot_trajectory,
                                     r_foot_trajectory):
  t_sampled = np.linspace(0, t_points[-1], 100)
  l_foot = []
  r_foot = []
  com = []
  pelvis_rot = []
  pelvis_rot_trajectory_dot = pelvis_rot_trajectory.derivative(1)
  for t in t_sampled:
    l_foot.append(l_foot_trajectory.value(t))
    r_foot.append(r_foot_trajectory.value(t))
    com.append(com_trajectory.value(t))
    # pelvis_rot.append(pelvis_rot_trajectory.value(t))
    pelvis_rot.append(pelvis_rot_trajectory_dot.value(t))
  l_foot = np.array(l_foot)
  r_foot = np.array(r_foot)
  com = np.array(com)
  pelvis_rot = np.array(pelvis_rot)

  plt.figure("Nominal Trajectories")
  plt.plot(t_sampled, l_foot[:, :, 0])
  plt.plot(t_sampled, r_foot[:, :, 0])
  plt.plot(t_sampled, com[:, :, 0])
  plt.plot(t_sampled, pelvis_rot[:, :, 0])


def plot_osc_debug(t_u, fsm, osc_debug):
  fig = plt.figure("OSC debug")
  # plt.plot(t_osc_debug, osc_debug[0].y_des)
  # plt.plot(osc_debug["com_traj"].t, osc_debug["com_traj"].y_des)
  # plt.plot(osc_debug["com_traj"].t, osc_debug["com_traj"].y)
  # plt.plot(osc_debug["com_traj"].t, osc_debug["com_traj"].error_y)
  plt.plot(osc_debug["com_traj"].t, osc_debug["com_traj"].yddot_des)
  plt.plot(osc_debug["com_traj"].t, osc_debug["com_traj"].yddot_command_sol)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].y_des)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].error_y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].error_ydot)
  # plt.plot(osc_debug["pelvis_rot_tracking_data"].t, osc_debug["pelvis_rot_tracking_data"].error_y)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].yddot_des)
  # plt.plot(osc_debug["pelvis_rot_traj"].t, osc_debug["pelvis_rot_traj"].yddot_command_sol)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].y_des)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].y)
  # plt.plot(osc_debug["r_foot_traj"].t, osc_debug["r_foot_traj"].error_y)
  # plt.plot(osc_debug["r_foot_traj"].t , osc_debug["r_foot_traj"].ydot_des)
  # plt.plot(osc_debug["r_foot_traj"].t , osc_debug["r_foot_traj"].ydot)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].y_des)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].y)
  # plt.plot(osc_debug["l_foot_traj"].t, osc_debug["l_foot_traj"].error_y)
  plt.plot(t_u, fsm, 'k--')
  plt.legend(["0", "1", "2", "3", "4", "5", "6"])


def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes, estop_signal):
  pos_indices = slice(11, 15)
  vel_indices = slice(33, 37)
  u_indices = slice(4, 8)
  plt.figure("positions")
  plt.plot(t_x, x[:, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities")
  plt.plot(t_x, x[:, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts")
  plt.plot(t_u, u[:, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.show()


if __name__ == "__main__":
  main()
