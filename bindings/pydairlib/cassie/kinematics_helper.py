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

    self.world = self.plant.world_frame()
    self.context = self.plant.CreateDefaultContext()

  def compute_center_of_mass_pos(self, state):
    self.plant.SetPositionsAndVelocities(self.context, state)
    return self.plant.CalcCenterOfMassPositionInWorld(self.context)

  def compute_center_of_mass_vel(self, state):
    self.plant.SetPositionsAndVelocities(self.context, state)
    return self.plant.CalcJacobianCenterOfMassTranslationalVelocity(self.context, JacobianWrtVariable.kV, self.world,
                                                                    self.world) @ state[-self.plant.num_velocities():]
