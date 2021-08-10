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


class DrakeToMujocoConverter():

  def __init__(self, drake_sim_dt=5e-5):
    self.builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
    addCassieMultibody(self.plant, self.scene_graph, True,
                       "examples/Cassie/urdf/cassie_v2.urdf", False, False)
    self.plant.Finalize()
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

  def convert_to_drake(self, q, v):
    q_drake = np.zeros(23)
    v_drake = np.zeros(22)
    q_drake[0:10] = x[0:10]

    x_drake =

    return

  def convert_to_mujoco(self, x):
    # q_full = np.zeros(35)
    # v_full = np.zeros(32)
    q = x[0:23]
    v = x[23:45]
    q_full, v_full = self.solve_IK(x)

    q_full[:10] = q[:10]
    # left achilles rod quat
    # q_full[10]  = q
    # q_full[11]  = q
    # q_full[12]  = q
    # q_full[13]  = q
    q_full[14] = q[10]
    q_full[15] = q[12]
    q_full[16] = q[13]
    q_full[17] = q[14]
    # left foot crank + left planar rod
    # q_full[18]  = q
    # q_full[19]  = q
    q_full[20] = q[11]
    q_full[21] = q[15]
    q_full[22] = q[16]
    q_full[23] = q[17]
    # right achilles rod quat
    # q_full[24]  = q
    # q_full[25]  = q
    # q_full[26]  = q
    # q_full[27]  = q
    q_full[28] = q[18]
    q_full[29] = q[20]
    q_full[30] = q[21]
    q_full[31] = q[22]
    # right foot crank + right planar rod
    # q_full[32]  = q
    # q_full[33]  = q
    q_full[34] = q[19]

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
    l_x_vec = (left_thigh - left_heel)[:, 0]
    l_x_vec *= 1 / np.linalg.norm(l_x_vec)
    l_y_vec = np.cross(l_x_vec, world_frame[0])
    l_z_vec = np.cross(l_x_vec, l_y_vec)
    r_x_vec = (right_thigh - right_heel)[:, 0]
    r_x_vec *= 1 / np.linalg.norm(r_x_vec)
    r_y_vec = np.cross(r_x_vec, world_frame[0])
    r_z_vec = np.cross(r_x_vec, r_y_vec)
    l_thigh_frame = self.plant.CalcRelativeTransform(self.context, self.left_thigh_rod[1], self.world)
    l_bar_quat = R.from_dcm(l_thigh_frame @ np.vstack((l_x_vec, l_y_vec, l_z_vec))).as_quat()
    r_thigh_frame = self.plant.CalcRelativeTransform(self.context, self.right_thigh_rod[1], self.world)
    r_bar_quat = R.from_dcm(r_thigh_frame @ np.vstack((r_x_vec, r_y_vec, r_z_vec))).as_quat()

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

    import pdb; pdb.set_trace()

    q_missing = np.zeros(35)
    v_missing = np.zeros(32)

    # import pdb; pdb.set_trace()

    # left_quat =
    #
    q_missing[10:14] = l_bar_quat
    q_missing[18] = left_foot_crank_ang
    # q_missing[19] = left_plantar_rod
    q_missing[24:28] = r_bar_quat
    q_missing[32] = right_foot_crank_ang
    # q_missing[33] = right_plantar_rod
    #
    # v_missing[9:12] = left_omega
    # v_missing[16] = left_foot_crank_dot
    # v_missing[17] = left_plantar_rod_dot
    # v_missing[22:25] = right_omega
    # v_missing[29] = right_foot_crank_dot
    # v_missing[30] = right_plantar_rod_dot

    return q_missing, v_missing
