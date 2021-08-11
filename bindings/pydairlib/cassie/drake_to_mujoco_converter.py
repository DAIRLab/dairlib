import math

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


class DrakeToMujocoConverter():

  def __init__(self, drake_sim_dt=5e-5):
    self.builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
    addCassieMultibody(self.plant, self.scene_graph, True,
                       "examples/Cassie/urdf/cassie_v2.urdf", False, False)
    self.plant.Finalize()

    self.pos_map = makeNameToPositionsMap(self.plant)
    self.vel_map = makeNameToVelocitiesMap(self.plant)
    self.act_map = makeNameToActuatorsMap(self.plant)

    self.map_q_drake_to_mujoco = np.zeros((23, 35))
    self.map_v_drake_to_mujoco = np.zeros((22, 32))

    self.generate_matrices()

    # for pos in self.pos_map:
    #   print(pos + str(self.pos_map[pos]))

    self.world = self.plant.world_frame()
    self.context = self.plant.CreateDefaultContext()
    self.ik_solver = InverseKinematics(self.plant, self.context, with_joint_limits=True)
    self.left_thigh_rod = LeftRodOnThigh(self.plant)
    self.right_thigh_rod = RightRodOnThigh(self.plant)
    self.left_heel_rod = LeftRodOnHeel(self.plant)
    self.right_heel_rod = RightRodOnHeel(self.plant)

    left_foot_crank_offset = np.array([0.058, -0.034, 0.02275])
    right_foot_crank_offset = np.array([0.058, -0.034, -0.02275])
    toe_joint_offset = np.array([0.35012, 0, 0])
    self.left_foot_crank = (left_foot_crank_offset, self.plant.GetBodyByName('tarsus_left').body_frame())
    self.right_foot_crank = (right_foot_crank_offset, self.plant.GetBodyByName('tarsus_right').body_frame())
    self.left_toe_joint = (toe_joint_offset, self.plant.GetBodyByName('toe_left').body_frame())
    self.right_toe_joint = (toe_joint_offset, self.plant.GetBodyByName('toe_right').body_frame())

    self.left_achilles_frame = np.zeros((3,3))
    self.left_achilles_frame[:, 0] = np.array([0.7922,  -0.60599, -0.072096])
    self.left_achilles_frame[:, 1] = np.array([0.60349,  0.79547, -0.054922])
    self.left_achilles_frame[:, 2] = np.cross(self.left_achilles_frame[:, 0], self.left_achilles_frame[:, 1])
    self.right_achilles_frame = np.zeros((3,3))
    self.right_achilles_frame[:, 0] = np.array([0.7922, -0.60599,  0.072096])
    self.right_achilles_frame[:, 1] = np.array([0.60349, 0.79547,  0.054922])
    self.right_achilles_frame[:, 2] = np.cross(self.right_achilles_frame[:, 0], self.right_achilles_frame[:, 1])


  def convert_to_drake(self, q, v):
    q_drake = np.zeros(23)
    v_drake = np.zeros(22)
    q_drake = self.map_q_drake_to_mujoco.T @ q

    # x_drake =

    return q_drake, v_drake


  def generate_matrices(self):
    self.map_q_drake_to_mujoco = np.zeros((35, 23))
    self.map_q_drake_to_mujoco[0:3, 4:7] = np.eye(3)
    self.map_q_drake_to_mujoco[3:7, 0:4] = np.eye(4)
    self.map_q_drake_to_mujoco[3:7, 0:4] = np.eye(4)
    self.map_q_drake_to_mujoco[7, 7] = 1
    self.map_q_drake_to_mujoco[8, 9] = 1
    self.map_q_drake_to_mujoco[9, 11] = 1
    self.map_q_drake_to_mujoco[14, 13] = 1
    self.map_q_drake_to_mujoco[15, 15] = 1
    self.map_q_drake_to_mujoco[16, 17] = 1
    self.map_q_drake_to_mujoco[17, 19] = 1
    self.map_q_drake_to_mujoco[20, 20] = 1
    self.map_q_drake_to_mujoco[21, 8] = 1
    self.map_q_drake_to_mujoco[22, 10] = 1
    self.map_q_drake_to_mujoco[23, 12] = 1
    self.map_q_drake_to_mujoco[28, 14] = 1
    self.map_q_drake_to_mujoco[29, 16] = 1
    self.map_q_drake_to_mujoco[30, 18] = 1
    self.map_q_drake_to_mujoco[31, 21] = 1
    self.map_q_drake_to_mujoco[34, 22] = 1

  def convert_to_mujoco(self, x):
    # q_full = np.zeros(35)
    v_full = np.zeros(32)
    q = x[0:23]
    v = x[23:45]
    q_missing, v_missing = self.solve_IK(x)
    q_copy = self.map_q_drake_to_mujoco @ q
    q_full = q_missing + q_copy

    # q_full[0:3] = q[4:7]
    # q_full[3:7] = q[0:4]
    # q_full[7] = q[7]
    # q_full[8] = q[9]
    # q_full[9] = q[11]
    # # left achilles rod quat
    # # q_full[10]  = q
    # # q_full[11]  = q
    # # q_full[12]  = q
    # # q_full[13]  = q
    # q_full[14] = q[13]
    # q_full[15] = q[15]
    # q_full[16] = q[17]
    # q_full[17] = q[19]
    # # left foot crank + left planar rod
    # # q_full[18]  = q
    # # q_full[19]  = q
    # q_full[20] = q[20]
    # q_full[21] = q[8]
    # q_full[22] = q[10]
    # q_full[23] = q[12]
    # # right achilles rod quat
    # # q_full[24]  = q
    # # q_full[25]  = q
    # # q_full[26]  = q
    # # q_full[27]  = q
    # q_full[28] = q[14]
    # q_full[29] = q[16]
    # q_full[30] = q[18]
    # q_full[31] = q[21]
    # # right foot crank + right planar rod
    # # q_full[32]  = q
    # # q_full[33]  = q
    # q_full[34] = q[22]

    v_full[:9] = v[:9]
    rot = R.from_quat([q[6], q[3], q[4], q[5]])
    v_full[3:6] = rot.as_dcm().T @ v[3:6]
    # left achilles omega
    # v_full[9] =  v
    # v_full[10] = v
    # v_full[11] = v
    v_full[12] = v[9]
    v_full[13] = v[11]
    v_full[14] = v[12]
    v_full[15] = v[13]
    # left foot crank + left planar rod
    # v_full[16] = v
    # v_full[17] = v
    v_full[18] = v[10]
    v_full[19] = v[14]
    v_full[20] = v[15]
    v_full[21] = v[16]
    # right achilles omega
    # v_full[22] = v
    # v_full[23] = v
    # v_full[24] = v
    v_full[25] = v[17]
    v_full[26] = v[19]
    v_full[27] = v[20]
    v_full[28] = v[21]
    # right foot crank + right planar rod
    # v_full[29] = v
    # v_full[30] = v
    v_full[31] = v[18]

    # x_full = np.hstack((q_full, v_full))
    return q_full, v_full

  # def solve_foot_angles(self):

  def solve_IK(self, x):
    self.plant.SetPositionsAndVelocities(self.context, x)

    # self.plant.CalcRelativeTransform(self.context, self.world)
    left_thigh = self.plant.CalcPointsPositions(self.context, self.left_thigh_rod[1], self.left_thigh_rod[0],
                                                self.world)
    right_thigh = self.plant.CalcPointsPositions(self.context, self.right_thigh_rod[1], self.right_thigh_rod[0],
                                                 self.world)
    left_heel = self.plant.CalcPointsPositions(self.context, self.left_heel_rod[1], self.left_heel_rod[0], self.world)
    right_heel = self.plant.CalcPointsPositions(self.context, self.right_heel_rod[1], self.right_heel_rod[0],
                                                self.world)

    left_foot_crank = self.plant.CalcPointsPositions(self.context, self.left_foot_crank[1], self.left_foot_crank[0],
                                                     self.world)
    right_foot_crank = self.plant.CalcPointsPositions(self.context, self.right_foot_crank[1], self.right_foot_crank[0],
                                                      self.world)
    left_toe_joint = self.plant.CalcPointsPositions(self.context, self.left_toe_joint[1], self.left_toe_joint[0],
                                                    self.world)
    right_toe_joint = self.plant.CalcPointsPositions(self.context, self.right_toe_joint[1], self.right_toe_joint[0],
                                                     self.world)

    world_frame = np.eye(3)
    l_x_vec = (left_heel - left_thigh)[:, 0]
    l_x_vec *= 1 / np.linalg.norm(l_x_vec)
    l_y_vec = np.cross(l_x_vec, world_frame[0])
    l_y_vec *= 1 / np.linalg.norm(l_y_vec)
    l_z_vec = np.cross(l_x_vec, l_y_vec)
    r_x_vec = (right_heel - right_thigh)[:, 0]
    r_x_vec *= 1 / np.linalg.norm(r_x_vec)
    r_y_vec = np.cross(r_x_vec, world_frame[0])
    r_y_vec *= 1 / np.linalg.norm(r_y_vec)
    r_z_vec = np.cross(r_x_vec, r_y_vec)
    l_thigh_frame = self.plant.CalcRelativeTransform(self.context, self.left_thigh_rod[1], self.world)
    l_bar_frame = np.vstack((l_x_vec, l_y_vec, l_z_vec)).T
    l_bar_quat = R.from_dcm(self.left_achilles_frame.T @ l_thigh_frame.rotation().matrix() @ l_bar_frame).as_quat()
    print(l_bar_quat)
    # l_bar_quat = R.from_dcm(l_thigh_frame.rotation().matrix().T @ l_bar_frame).as_quat()
    # print(l_bar_quat)
    l_bar_quat = R.from_dcm(self.left_achilles_frame.T @ l_bar_frame).as_quat()
    # l_bar_quat = R.from_dcm(self.left_achilles_frame @ l_thigh_frame.rotation().matrix() @ l_bar_frame).as_quat()
    # print(l_bar_quat)
    # l_bar_quat = R.from_dcm(np.vstack((l_x_vec, l_y_vec, l_z_vec))).as_quat()
    l_bar_quat = np.hstack((l_bar_quat[3], l_bar_quat[0:3]))
    r_thigh_frame = self.plant.CalcRelativeTransform(self.context, self.right_thigh_rod[1], self.world)
    r_bar_quat = R.from_dcm(r_thigh_frame.rotation() @ np.vstack((r_x_vec, r_y_vec, r_z_vec))).as_quat()

    l_plantar_x_vec = (left_foot_crank - left_toe_joint)[:, 0]
    # l_plantar_x_vec *= 1 / np.linalg.norm(l_plantar_x_vec)
    # l_plantar_y_vec = np.cross(l_plantar_x_vec, world_frame[0])
    # l_plantar_z_vec = np.cross(l_plantar_x_vec, l_plantar_y_vec)
    r_plantar_x_vec = (left_foot_crank - left_toe_joint)[:, 0]
    # r_plantar_x_vec *= 1 / np.linalg.norm(r_plantar_x_vec)
    # r_plantar_y_vec = np.cross(r_plantar_x_vec, world_frame[0])
    # r_plantar_z_vec = np.cross(r_plantar_x_vec, r_plantar_y_vec)

    l_foot_crank_frame = self.plant.CalcRelativeTransform(self.context, self.left_foot_crank[1], self.world)
    r_foot_crank_frame = self.plant.CalcRelativeTransform(self.context, self.right_foot_crank[1], self.world)
    l_toe_frame = self.plant.CalcRelativeTransform(self.context, self.left_toe_joint[1], self.world)
    r_toe_frame = self.plant.CalcRelativeTransform(self.context, self.right_toe_joint[1], self.world)

    left_foot_crank_ang = math.acos(np.dot(l_plantar_x_vec, l_foot_crank_frame.rotation().matrix()[0]))
    right_foot_crank_ang = math.acos(np.dot(r_plantar_x_vec, r_foot_crank_frame.rotation().matrix()[0]))
    left_plantar_rod = math.acos(np.dot(l_plantar_x_vec, l_toe_frame.rotation().matrix()[0]))
    right_plantar_rod = math.acos(np.dot(r_plantar_x_vec, r_toe_frame.rotation().matrix()[0]))


    q_missing = np.zeros(35)
    v_missing = np.zeros(32)

    # import pdb; pdb.set_trace()
    print(l_bar_quat)
    # import pdb; pdb.set_trace()
    l_bar_quat = np.array([[ 0.97850359, -0.0164015, 0.01785328, -0.20479984]])
    r_bar_quat = np.array([[ 0.97850359, -0.0164015, 0.01785328, -0.20479984]])
    # r_bar_quat = np.array([1, 0, 0, 0])
    left_foot_crank_ang = -1.5212260470160093
    left_plantar_rod = 1.5190997625212834
    right_foot_crank_ang = -1.5212260470160093
    right_plantar_rod = 1.5190997625212834
    q_missing[10:14] = l_bar_quat
    q_missing[18] = left_foot_crank_ang
    q_missing[19] = left_plantar_rod
    q_missing[24:28] = r_bar_quat
    q_missing[32] = right_foot_crank_ang
    q_missing[33] = right_plantar_rod
    #
    # v_missing[9:12] = left_omega
    # v_missing[16] = left_foot_crank_dot
    # v_missing[17] = left_plantar_rod_dot
    # v_missing[22:25] = right_omega
    # v_missing[29] = right_foot_crank_dot
    # v_missing[30] = right_plantar_rod_dot

    return q_missing, v_missing
