import numpy as np
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from scipy.spatial.transform import Rotation as R
from pydairlib.multibody import *
from pydrake.multibody.tree import *
from pydrake.multibody.tree import JacobianWrtVariable
from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import LeftLoopClosureEvaluator
from pydairlib.cassie.cassie_utils import RightLoopClosureEvaluator
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource


class KinematicsHelper():

  def __init__(self, urdf="examples/Cassie/urdf/cassie_v2.urdf"):
    self.builder = DiagramBuilder()
    self.drake_sim_dt = 1e-5
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
    self.fixed_springs = urdf == "examples/Cassie/urdf/cassie_fixed_springs.urdf"

    AddCassieMultibody(self.plant, self.scene_graph, True, urdf, False, False)
    self.plant.Finalize()

    self.diagram = self.builder.Build()

    self.nq = self.plant.num_positions()
    self.nv = self.plant.num_velocities()

    self.pos_map_spr_to_wo_spr = np.empty(0)
    self.vel_map_spr_to_wo_spr = np.empty(0)
    self.load_maps()
    self.map_spr_to_wo_spr = np.zeros((37, 45))
    self.map_spr_to_wo_spr[0:19, 0:23] = self.pos_map_spr_to_wo_spr
    self.map_spr_to_wo_spr[19:37, 23:45] = self.vel_map_spr_to_wo_spr

    self.l_toe_frame = self.plant.GetBodyByName("toe_left").body_frame()
    self.r_toe_frame = self.plant.GetBodyByName("toe_right").body_frame()

    self.front_contact_disp = np.array((-0.0457, 0.112, 0))
    self.rear_contact_disp = np.array((0.088, 0, 0))

    self.world = self.plant.world_frame()
    self.context = self.plant.CreateDefaultContext()
    
  def set_state(self, state):
    new_state = np.copy(state)
    if self.fixed_springs:
      new_state = self.map_spr_to_wo_spr @ state
    self.plant.SetPositionsAndVelocities(self.context, new_state)

  def compute_center_of_mass_pos(self, state):
    self.set_state(state)
    return self.plant.CalcCenterOfMassPositionInWorld(self.context)

  def compute_center_of_mass_vel(self, state):
    self.set_state(state)
    return self.plant.CalcJacobianCenterOfMassTranslationalVelocity(self.context, JacobianWrtVariable.kV, self.world,
                                                                    self.world) @ state[-self.plant.num_velocities():]

  def compute_foot_positions(self, state):
    self.set_state(state)
    l_rear = self.plant.CalcPointsPositions(self.context, self.l_toe_frame,
                                            self.rear_contact_disp, self.world)[:, 0]
    l_front = self.plant.CalcPointsPositions(self.context, self.l_toe_frame,
                                             self.front_contact_disp, self.world)[:, 0]
    r_rear = self.plant.CalcPointsPositions(self.context, self.r_toe_frame,
                                            self.rear_contact_disp, self.world)[:, 0]
    r_front = self.plant.CalcPointsPositions(self.context, self.r_toe_frame,
                                             self.front_contact_disp, self.world)[:, 0]
    return l_rear, l_front, r_rear, r_front

  def compute_foot_velocities(self, state):
    self.set_state(state)
    l_rear = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.l_toe_frame, self.rear_contact_disp,
      self.world,
      self.world) @ state[-self.nv:]
    l_front = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.l_toe_frame, self.front_contact_disp,
      self.world,
      self.world) @ state[-self.nv:]
    r_rear = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.r_toe_frame, self.rear_contact_disp,
      self.world,
      self.world) @ state[-self.nv:]
    r_front = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.r_toe_frame, self.front_contact_disp,
      self.world,
      self.world) @ state[-self.nv:]
    return l_rear, l_front, r_rear, r_front

  def compute_foot_z_position(self, state):
    l_rear, l_front, r_rear, r_front = self.compute_foot_positions(state)
    return np.array([l_rear[2], l_front[2], r_rear[2], r_front[2]])

  def compute_foot_z_vel(self, state):
    l_rear, l_front, r_rear, r_front = self.compute_foot_velocities(state)
    return np.array([l_rear[2], l_front[2], r_rear[2], r_front[2]])

  def compute_pelvis_pos_w_fixed_feet(self, state):
    self.set_state(state)
    l_rear, l_front, r_rear, r_front = self.compute_foot_positions(state)
    l_rear_vel, l_front_vel, r_rear_vel, r_front_vel = self.compute_foot_velocities(state)
    pelvis_pos = state[4:7]
    pelvis_vel = state[26:29]
    foot_pos = [l_rear, l_front, r_rear, r_front]
    foot_vels = [l_rear_vel, l_front_vel, r_rear_vel, r_front_vel]

    # compute only the average foot velocities of those that are mostly static
    foot_pos_average = 0.25 * (l_rear + l_front + r_rear + r_front)
    foot_vel_average = 0.25 * (l_rear_vel + l_front_vel + r_rear_vel + r_front_vel)
    # import pdb; pdb.set_trace()
    # print(foot_vel_average[2])
    # print(foot_vel_average[2])
    # return pelvis_pos - foot_pos_average, pelvis_vel - foot_vel_average
    pelvis_vel_rel = np.zeros(2)
    pelvis_vel_rel[0] = np.linalg.norm(pelvis_vel[0:2] - foot_vel_average[0:2])
    pelvis_vel_rel[1] = pelvis_vel[2] - foot_vel_average[2]
    # return pelvis_pos - foot_pos_average, pelvis_vel - 0.5 * (l_rear_vel + r_rear_vel)
    return pelvis_pos - foot_pos_average, pelvis_vel - foot_vel_average
    # return pelvis_pos - foot_pos_average, pelvis_vel
    # return pelvis_pos - foot_pos_average, pelvis_vel_rel

  def compute_rigid_impact_map(self, state):
    # import pdb; pdb.set_trace()
    self.set_state(state)
    M = self.plant.CalcMassMatrixViaInverseDynamics(self.context)
    M_inv = np.linalg.inv(M)
    J_l_r = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.l_toe_frame, self.rear_contact_disp,
      self.world,
      self.world)
    J_l_f = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.l_toe_frame, self.front_contact_disp,
      self.world,
      self.world)
    J_r_r = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.r_toe_frame, self.rear_contact_disp,
      self.world,
      self.world)
    J_r_f = self.plant.CalcJacobianTranslationalVelocity(
      self.context, JacobianWrtVariable.kV, self.r_toe_frame, self.front_contact_disp,
      self.world,
      self.world)
    J_l_loop = LeftLoopClosureEvaluator(self.plant).EvalFullJacobian(self.context)
    J_r_loop = RightLoopClosureEvaluator(self.plant).EvalFullJacobian(self.context)

    J = np.vstack((J_l_r, J_l_f, J_r_r, J_r_f, J_l_loop, J_r_loop))
    I = np.eye(self.nv)
    JMJ_inv = np.linalg.inv(J @ M_inv @ J.T)
    v_post = (I - M_inv @ J.T @ JMJ_inv @ J) @ state[-self.nv:]
    return np.hstack((state[:self.nq], v_post))

  def load_maps(self):
    self.pos_map_spr_to_wo_spr = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
    self.vel_map_spr_to_wo_spr = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
