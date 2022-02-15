import numpy as np
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from scipy.spatial.transform import Rotation as R
from pydairlib.multibody import *
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydairlib.multibody import MultiposeVisualizer


class DrakeToIsaacConverter():

    def __init__(self, drake_sim_dt=5e-5):
        self.builder = DiagramBuilder()
        self.drake_sim_dt = drake_sim_dt
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
        AddCassieMultibody(self.plant, self.scene_graph, True,
                           "examples/Cassie/urdf/cassie_v2.urdf", False, False)
        self.plant.Finalize()

        self.pos_map = makeNameToPositionsMap(self.plant)
        self.vel_map = makeNameToVelocitiesMap(self.plant)
        self.act_map = makeNameToActuatorsMap(self.plant)

        self.map_q_drake_to_isaac = np.zeros((16, 23))
        self.map_v_drake_to_isaac = np.zeros((16, 22))

        self.generate_matrices()

    def map_drake_pos_to_isaac_joint_pos(self, drake_pos):
        return self.map_q_drake_to_isaac @ drake_pos

    def map_drake_vel_to_isaac_joint_vel(self, drake_vel):
        return self.map_v_drake_to_isaac @ drake_vel

    def generate_matrices(self):
        self.map_q_drake_to_isaac = np.zeros((16, 23))
        self.map_v_drake_to_isaac = np.zeros((16, 22))
        self.map_q_drake_to_isaac[0, self.pos_map["hip_roll_left"]] = 1
        self.map_q_drake_to_isaac[1, self.pos_map["hip_yaw_left"]] = 1
        self.map_q_drake_to_isaac[2, self.pos_map["hip_pitch_left"]] = 1
        self.map_q_drake_to_isaac[3, self.pos_map["knee_left"]] = 1
        self.map_q_drake_to_isaac[4, self.pos_map["knee_joint_left"]] = 1
        self.map_q_drake_to_isaac[5, self.pos_map["ankle_joint_left"]] = 1
        self.map_q_drake_to_isaac[6, self.pos_map["ankle_spring_joint_left"]] = 1
        self.map_q_drake_to_isaac[7, self.pos_map["toe_left"]] = 1
        self.map_q_drake_to_isaac[8, self.pos_map["hip_roll_right"]] = 1
        self.map_q_drake_to_isaac[9, self.pos_map["hip_yaw_right"]] = 1
        self.map_q_drake_to_isaac[10, self.pos_map["hip_pitch_right"]] = 1
        self.map_q_drake_to_isaac[11, self.pos_map["knee_right"]] = 1
        self.map_q_drake_to_isaac[12, self.pos_map["knee_joint_right"]] = 1
        self.map_q_drake_to_isaac[13, self.pos_map["ankle_joint_right"]] = 1
        self.map_q_drake_to_isaac[14, self.pos_map["ankle_spring_joint_right"]] = 1
        self.map_q_drake_to_isaac[15, self.pos_map["toe_right"]] = 1

        self.map_v_drake_to_isaac[0, self.vel_map["hip_roll_leftdot"]] = 1
        self.map_v_drake_to_isaac[1, self.vel_map["hip_yaw_leftdot"]] = 1
        self.map_v_drake_to_isaac[2, self.vel_map["hip_pitch_leftdot"]] = 1
        self.map_v_drake_to_isaac[3, self.vel_map["knee_leftdot"]] = 1
        self.map_v_drake_to_isaac[4, self.vel_map["knee_joint_leftdot"]] = 1
        self.map_v_drake_to_isaac[5, self.vel_map["ankle_joint_leftdot"]] = 1
        self.map_v_drake_to_isaac[6, self.vel_map["ankle_spring_joint_leftdot"]] = 1
        self.map_v_drake_to_isaac[7, self.vel_map["toe_leftdot"]] = 1
        self.map_v_drake_to_isaac[8, self.vel_map["hip_roll_rightdot"]] = 1
        self.map_v_drake_to_isaac[9, self.vel_map["hip_yaw_rightdot"]] = 1
        self.map_v_drake_to_isaac[10, self.vel_map["hip_pitch_rightdot"]] = 1
        self.map_v_drake_to_isaac[11, self.vel_map["knee_rightdot"]] = 1
        self.map_v_drake_to_isaac[12, self.vel_map["knee_joint_rightdot"]] = 1
        self.map_v_drake_to_isaac[13, self.vel_map["ankle_joint_rightdot"]] = 1
        self.map_v_drake_to_isaac[14, self.vel_map["ankle_spring_joint_rightdot"]] = 1
        self.map_v_drake_to_isaac[15, self.vel_map["toe_rightdot"]] = 1