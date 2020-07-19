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
import pydairlib.multibody_utils
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

  pos_map = pydairlib.multibody_utils.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody_utils.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody_utils.makeNameToActuatorsMap(plant)

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

  contact_info, contact_info_locs, control_inputs, estop_signal, osc_debug, \
  q, switch_signal, t_contact_info, t_controller_switch, t_osc, t_osc_debug, \
  t_state, v, fsm = process_lcm_log.process_log(log, pos_map, vel_map)

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
  pelvis_rot_trajectory = PiecewisePolynomial.FirstOrderHold(t_points_pelvis_rot, pelvis_rot_points[:4])

  t_sampled = np.linspace(0, t_points_l_foot[-1], 100)
  l_foot = []
  r_foot = []
  com = []
  pelvis_rot = []
  for t in t_sampled:
    l_foot.append(l_foot_trajectory.value(t))
    r_foot.append(r_foot_trajectory.value(t))
    com.append(com_trajectory.value(t))
    pelvis_rot.append(pelvis_rot_trajectory.value(t))
  l_foot = np.array(l_foot)
  r_foot = np.array(r_foot)
  com = np.array(com)
  pelvis_rot = np.array(pelvis_rot)

  plt.figure("Nominal Trajectories")
  plt.plot(t_sampled, l_foot[:, :, 0])
  plt.plot(t_sampled, r_foot[:, :, 0])
  # plt.plot(t_sampled, com[:, :, 0])
  # plt.plot(t_sampled, pelvis_rot[:, :, 0])

  # import pdb; pdb.set_trace()
  fig = plt.figure("OSC debug")
  # plt.plot(t_osc_debug, osc_debug[0].y_des)
  plt.plot(t_osc_debug, osc_debug[1].y_des)
  # plt.plot(t_osc_debug, osc_debug[2].y_des)

  plt.show()

if __name__ == "__main__":
  main()
