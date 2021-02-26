import sys
import lcm
import numpy as np
import scipy.linalg as linalg
import pathlib

import process_log

import matplotlib.pyplot as plt
import matplotlib
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
from bindings.pydairlib.parameter_studies.plot_styler import PlotStyler
from pydairlib.cassie.cassie_utils import *

import process_lcm_log

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder


def cassie_main():
  builder = DiagramBuilder()
  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant_wo_spr, scene_graph_wo_spr, True,
                                                   "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)

  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  context_w_spr = plant_w_spr.CreateDefaultContext()
  context_wo_spr = plant_wo_spr.CreateDefaultContext()

  nq = plant_wo_spr.num_positions()
  nv = plant_wo_spr.num_velocities()
  nx = plant_wo_spr.num_positions() + plant_wo_spr.num_velocities()
  nu = plant_wo_spr.num_actuators()
  nc = 10

  l_toe_frame = plant_wo_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_wo_spr.GetBodyByName("toe_right").body_frame()
  l_thigh_frame = plant_wo_spr.GetBodyByName("thigh_left").body_frame()
  r_thigh_frame = plant_wo_spr.GetBodyByName("thigh_right").body_frame()
  l_heel_frame = plant_wo_spr.GetBodyByName("heel_spring_left").body_frame()
  r_heel_frame = plant_wo_spr.GetBodyByName("heel_spring_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  heel_disp = np.array((.11877, -.01, 0.0))
  left_thigh_disp = np.array((0.0, 0.0, 0.045))
  right_thigh_disp = np.array((0.0, 0.0, -0.045))
  world = plant_wo_spr.world_frame()

  filename = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d"
  # filename = "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/walking_0.5"
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)
  state_traj = dircon_traj.ReconstructStateTrajectory()

  impact_time = dircon_traj.GetStateBreaks(1)[-1]
  x_pre = state_traj.value(impact_time - 1e-6)
  x_post = state_traj.value(dircon_traj.GetStateBreaks(2)[0])

  plant_wo_spr.SetPositionsAndVelocities(context_wo_spr, x_pre)
  M = plant_wo_spr.CalcMassMatrixViaInverseDynamics(context_wo_spr)
  M_inv = np.linalg.inv(M)
  J_l_f = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
  J_l_r = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
  J_r_f = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)
  J_r_r = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)

  left_heel = plant_wo_spr.CalcPointsPositions(context_wo_spr, l_heel_frame, heel_disp, world)
  left_thigh = plant_wo_spr.CalcPointsPositions(context_wo_spr, l_thigh_frame, left_thigh_disp, world)
  right_heel = plant_wo_spr.CalcPointsPositions(context_wo_spr, r_heel_frame, heel_disp, world)
  right_thigh = plant_wo_spr.CalcPointsPositions(context_wo_spr, r_thigh_frame, right_thigh_disp, world)
  left_rel_pos = left_heel - left_thigh
  right_rel_pos = right_heel - right_thigh

  J_l_heel = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, l_heel_frame, heel_disp, world, world)
  J_l_thigh = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, l_thigh_frame, left_thigh_disp, world, world)
  J_r_heel = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, r_heel_frame, heel_disp, world, world)
  J_r_thigh = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, r_thigh_frame, right_thigh_disp, world, world)
  J_l_loop = (left_rel_pos.transpose() @ (J_l_heel - J_l_thigh)) / np.linalg.norm(left_rel_pos)
  J_r_loop = (right_rel_pos.transpose() @ (J_r_heel - J_r_thigh)) / np.linalg.norm(right_rel_pos)

  # J_l_loop = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, left_loop, front_contact_disp, world, world)
  # J_l_loop = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, right_loop, front_contact_disp, world, world)


  # J_l_r = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, l_toe_frame, offset, world, world)
  # J_r_r = plant_wo_spr.CalcJacobianTranslationalVelocity(context_wo_spr, JacobianWrtVariable.kV, r_toe_frame, offset, world, world)
  # J_l = np.vstack((J_l_f, J_l_r[0:2, :], J_r_f, J_r_r[0:2, :]))
  # J = np.vstack((J_r_f, J_r_r[1:3, :]))
  # J = np.vstack((J_l_f, J_l_r[1:3, :], J_r_f, J_r_r[1:3, :]))
  J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r, J_l_loop, J_r_loop))
  # J = np.vstack((J_l_r, J_r_r))
  M_Jt = M_inv @ J.T
  P = linalg.null_space(M_Jt.T).T
  proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T

  # transform = J @ M_inv @ J.T @ np.linalg.pinv(J @ M_inv @ J.T)


  t_impact = impact_time
  t = np.arange(t_impact - 0.01, t_impact + 0.01, 0.0005)
  vel = np.zeros((t.shape[0], nv))
  # cc_vel = np.zeros((t.shape[0], 6))
  cc_vel = np.zeros((t.shape[0], 18))

  for i in range(t.shape[0]):
    x = state_traj.value(t[i])
    vel[i] = x[-nv:, 0]
    cc_vel[i] = P.T @ P @ vel[i]
    # cc_vel[i] = proj_ii @ vel[i]

  plt.figure("Joint Velocities around impacts")
  plt.plot(t, vel)
  plt.figure("Change of coordinates")
  plt.plot(t, cc_vel, 'b')

  v_pre = x_pre[-nv:]
  v_post = x_post[-nv:]
  plt.ylim([-10, 10])


  plt.show()


def rabbit_main():
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    "/home/yangwill/workspace/dairlib/examples/five_link_biped"
    "/five_link_biped.urdf")
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
  plant.Finalize()
  context = plant.CreateDefaultContext()

  world = plant.world_frame()

  l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
  r_contact_frame = plant.GetBodyByName("right_foot").body_frame()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()
  nc = 2

  # Map to 2d
  TXZ = np.array([[1, 0, 0], [0, 0, 1]])

  filename = "/home/yangwill/Documents/research/projects/five_link_biped/walking/saved_trajs/rabbit_walking"
  dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(filename)
  state_traj = dircon_traj.ReconstructStateTrajectory()
  breaks = dircon_traj.GetBreaks()
  import pdb; pdb.set_trace()

  x_pre = state_traj.value(dircon_traj.GetStateBreaks(0)[-1] - 1e-6)
  x_post = state_traj.value(dircon_traj.GetStateBreaks(1)[0])

  plant.SetPositionsAndVelocities(context, x_pre)

  M = plant.CalcMassMatrixViaInverseDynamics(context)
  M_inv = np.linalg.inv(M)
  pt_on_body = np.zeros(3)
  J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
  J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_contact_frame, pt_on_body, world, world)
  M_Jt = M_inv @ J_r.T
  P = linalg.null_space(M_Jt.T).T

  proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T

  J_M_Jt = J_r @ M_inv @ J_r.T
  proj_y_ii = np.eye(2) - J_M_Jt @ J_M_Jt @ np.linalg.inv(J_M_Jt.T @ J_M_Jt) @ J_M_Jt.T

  v_pre = x_pre[-nv:]
  v_post = x_post[-nv:]
  t_impact = dircon_traj.GetStateBreaks(0)[-1]

  ydot_pre = J_r @ v_pre
  ydot_post = J_r @ v_post
  transform = J_r @ M_inv @ J_r.T @ np.linalg.pinv(J_r @ M_inv @ J_r.T)




  t = np.linspace(t_impact - 0.05, t_impact + 0.05, 100)

  cc = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T

  # cc_vel = np.zeros((t.shape[0], nv - nc))
  cc_vel = np.zeros((t.shape[0], nv))
  TV_cc_vel = np.zeros((t.shape[0], nv - nc))
  vel = np.zeros((t.shape[0], nv))
  for i in range(t.shape[0]):
    x = state_traj.value(t[i])
    vel[i] = x[-nv:, 0]
    plant.SetPositionsAndVelocities(context, x)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    TV_J = M_inv @ J_r.T
    TV_J_space = linalg.null_space(TV_J.T).T
    cc_vel[i] = P.T @ P @ vel[i]
    TV_cc_vel[i] = TV_J_space @ vel[i]

  # filename = sys.argv[1]
  # folder = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/logs/stiffness/"
  # log = lcm.EventLog(folder + filename, "r")
  # t_state, t_lqr, \
  # x, u, fsm, contact_info, contact_info_locs = \
  #   process_log.process_log(log, pos_map, vel_map)

  plt.figure("Generalized velocities around impacts")
  ps.plot(t, vel, xlabel='time (s)', ylabel='velocity (m/s)')
  ps.save_fig('generalized_velocities_around_impact.png')
  plt.figure("Impact invariant projection")
  ps.plot(t, cc_vel, xlabel='time (s)', ylabel='velocity (m/s)')
  ps.save_fig('projected_velocities_around_impact.png')


  # t_start = 0.2
  # t_end = 0.25
  # t_start_idx = np.argwhere(np.abs(t_state - t_start) < 1e-3)[0][0]
  # t_end_idx = np.argwhere(np.abs(t_state - t_end) < 1e-3)[0][0]
  # t_slice = slice(t_start_idx, t_end_idx)
  # t_state = t_state[t_slice]
  #
  # vel_sim = np.zeros((t_state.shape[0], nv))
  # cc_vel_sim = np.zeros((t_state.shape[0], nv - nc))
  # for i in range(t_state.shape[0]):
  #   x_i = x[i, :]
  #   vel_sim[i] = x_i[-nv:]
  #   cc_vel_sim[i] = P @ vel_sim[i]

  # plt.plot(t, TV_cc_vel, 'r')
  # plt.legend(["constant M and J", "constant M, q dependent J"])
  plt.show()

  print(P @ (v_pre - v_post))


if __name__ == '__main__':
  global ps
  ps = PlotStyler()
  figure_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/figures/'
  ps.set_default_styling(directory=figure_directory)
  matplotlib.rcParams["savefig.directory"] = "/home/yangwill/Documents/research/presentations/"
  matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]
  font = {'size'   : 20}
  matplotlib.rc('font', **font)
  matplotlib.rcParams['lines.linewidth'] = 4
  cassie_main()
  # rabbit_main()