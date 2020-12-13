import sys
import lcm
import numpy as np
import scipy.linalg as linalg
import pathlib

import process_log

import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
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

  nq = plant_wo_spr.num_positions()
  nv = plant_wo_spr.num_velocities()
  nx = plant_wo_spr.num_positions() + plant_wo_spr.num_velocities()
  nu = plant_wo_spr.num_actuators()

  l_toe_frame = plant_w_spr.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant_w_spr.GetBodyByName("toe_right").body_frame()

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

  v_pre = x_pre[-nv:]
  v_post = x_post[-nv:]
  t_impact = dircon_traj.GetStateBreaks(0)[-1]

  t = np.linspace(t_impact - 0.05, t_impact + 0.05, 100)

  cc_vel = np.zeros((t.shape[0], nv - nc))
  TV_cc_vel = np.zeros((t.shape[0], nv - nc))
  vel = np.zeros((t.shape[0], nv))
  for i in range(t.shape[0]):
    x = state_traj.value(t[i])
    vel[i] = x[-nv:, 0]
    plant.SetPositionsAndVelocities(context, x)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    TV_J = M_inv @ J_r.T
    TV_J_space = linalg.null_space(TV_J.T).T
    cc_vel[i] = P @ vel[i]
    TV_cc_vel[i] = TV_J_space @ vel[i]

  filename = sys.argv[1]
  folder = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/logs/stiffness/"
  log = lcm.EventLog(folder + filename, "r")
  t_state, t_lqr, \
  x, u, fsm, contact_info, contact_info_locs = \
    process_log.process_log(log, pos_map, vel_map)

  plt.figure("Joint Velocities around impacts")
  plt.plot(t, vel)
  plt.figure("Change of coordinates")
  plt.plot(t, cc_vel, 'b')

  t_start = 0.2
  t_end = 0.25
  t_start_idx = np.argwhere(np.abs(t_state - t_start) < 1e-3)[0][0]
  t_end_idx = np.argwhere(np.abs(t_state - t_end) < 1e-3)[0][0]
  t_slice = slice(t_start_idx, t_end_idx)
  t_state = t_state[t_slice]

  vel_sim = np.zeros((t_state.shape[0], nv))
  cc_vel_sim = np.zeros((t_state.shape[0], nv - nc))
  for i in range(t_state.shape[0]):
    x_i = x[i, :]
    vel_sim[i] = x_i[-nv:]
    cc_vel_sim[i] = P @ vel_sim[i]

  plt.figure("Joint Velocities around impacts sim data")
  plt.plot(t_state, vel_sim)
  plt.figure("Change of coordinates sim data")
  plt.plot(t_state, cc_vel_sim, 'b')

  # plt.plot(t, TV_cc_vel, 'r')
  # plt.legend(["constant M and J", "constant M, q dependent J"])
  plt.show()

  print(P @ (v_pre - v_post))
  import pdb; pdb.set_trace()


if __name__ == '__main__':
  # cassie_main()
  rabbit_main()