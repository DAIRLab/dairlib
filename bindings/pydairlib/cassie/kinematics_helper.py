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
    addCassieMultibody(self.plant, self.scene_graph, True,
                       urdf, False, False)
    self.plant.Finalize()

    self.diagram = self.builder.Build()

    self.nv = self.plant.num_velocities()

    self.l_toe_frame = self.plant.GetBodyByName("toe_left").body_frame()
    self.r_toe_frame = self.plant.GetBodyByName("toe_right").body_frame()

    self.front_contact_disp = np.array((-0.0457, 0.112, 0))
    self.rear_contact_disp = np.array((0.088, 0, 0))

    self.world = self.plant.world_frame()
    self.context = self.plant.CreateDefaultContext()

  def compute_center_of_mass_pos(self, state):
    self.plant.SetPositionsAndVelocities(self.context, state)
    return self.plant.CalcCenterOfMassPositionInWorld(self.context)

  def compute_center_of_mass_vel(self, state):
    self.plant.SetPositionsAndVelocities(self.context, state)
    return self.plant.CalcJacobianCenterOfMassTranslationalVelocity(self.context, JacobianWrtVariable.kV, self.world,
                                                                    self.world) @ state[-self.plant.num_velocities():]

  def compute_foot_positions(self, state):
    self.plant.SetPositionsAndVelocities(self.context, state)
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
    self.plant.SetPositionsAndVelocities(self.context, state)
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
    self.plant.SetPositionsAndVelocities(self.context, state)
    l_rear, l_front, r_rear, r_front = self.compute_foot_positions(state)
    l_rear_vel, l_front_vel, r_rear_vel, r_front_vel = self.compute_foot_velocities(state)
    pelvis_pos = state[4:7]
    pelvis_vel = state[26:29]
    foot_pos_average = 0.25 * (l_rear + l_front + r_rear + r_front)
    foot_vel_average = 0.25 * (l_rear_vel + l_front_vel + r_rear_vel + r_front_vel)
    return pelvis_pos - foot_pos_average, pelvis_vel - foot_vel_average

