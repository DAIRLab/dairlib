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
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve

class DrakeToMujocoConverter():

  def __init__(self, drake_sim_dt=5e-5):
    self.builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
    addCassieMultibody(self.plant, self.scene_graph, True,
                       "examples/Cassie/urdf/cassie_v2.urdf", False, False)
    self.plant.Finalize()

    self.foot_crank_plant = MultibodyPlant(drake_sim_dt)
    Parser(self.foot_crank_plant).AddModelFromFile(FindResourceOrThrow('examples/Cassie/urdf/cassie_foot_crank.urdf'))
    self.foot_crank_plant.Finalize()

    temp = makeNameToPositionsMap(self.foot_crank_plant)

    self.pos_map = makeNameToPositionsMap(self.plant)
    self.vel_map = makeNameToVelocitiesMap(self.plant)
    self.act_map = makeNameToActuatorsMap(self.plant)

    self.map_q_drake_to_mujoco = np.zeros((23, 35))
    self.map_v_drake_to_mujoco = np.zeros((22, 32))

    # for vel in self.vel_map:
    #   print(vel + str(self.vel_map[vel]))
    # for pos in temp:
    #   print(pos + str(temp[pos]))
    self.generate_matrices()

    self.world = self.plant.world_frame()
    self.context = self.plant.CreateDefaultContext()
    self.foot_crank_context = self.foot_crank_plant.CreateDefaultContext()
    self.ik_solver = InverseKinematics(self.foot_crank_plant, self.foot_crank_context, with_joint_limits=True)
    self.left_thigh_rod = LeftRodOnThigh(self.plant)
    self.right_thigh_rod = RightRodOnThigh(self.plant)
    self.left_heel_rod = LeftRodOnHeel(self.plant)
    self.right_heel_rod = RightRodOnHeel(self.plant)

    # left_plantar_rod_pos = np.array([0.35012, 0, 0])
    # right_plantar_rod_pos =  np.array([0.35012, 0, 0])
    # toe_joint_offset = np.array([0.35012, 0, 0])
    # self.left_foot_crank = (left_plantar_rod_pos, self.foot_crank_plant.GetBodyByName('plantar_rod_left').body_frame())
    # self.right_foot_crank = (right_plantar_rod_pos, self.foot_crank_plant.GetBodyByName('plantar_rod_left').body_frame())
    # self.left_toe_joint = (toe_joint_offset, self.plant.GetBodyByName('toe_left').body_frame())
    # self.right_toe_joint = (toe_joint_offset, self.plant.GetBodyByName('toe_right').body_frame())

    self.left_achilles_frame = np.zeros((3, 3))
    self.left_achilles_frame[:, 0] = np.array([0.7922, -0.60599, -0.072096])
    self.left_achilles_frame[:, 1] = np.array([0.60349, 0.79547, -0.054922])
    self.left_achilles_frame[:, 2] = np.cross(self.left_achilles_frame[:, 0], self.left_achilles_frame[:, 1])
    self.right_achilles_frame = np.zeros((3, 3))
    self.right_achilles_frame[:, 0] = np.array([0.7922, -0.60599, 0.072096])
    self.right_achilles_frame[:, 1] = np.array([0.60349, 0.79547, 0.054922])
    self.right_achilles_frame[:, 2] = np.cross(self.right_achilles_frame[:, 0], self.right_achilles_frame[:, 1])

    self.plantar_rod_frame = self.foot_crank_plant.GetBodyByName('plantar_rod_left').body_frame()
    self.toe_left_frame = self.foot_crank_plant.GetBodyByName('toe_left').body_frame()
    self.plantar_rod_anchor_point = np.array([0.35012, 0, 0])
    self.toe_left_anchor_point = np.array([0.04885482, 0.00394248, 0.01484])

    self.ik_solver.AddPositionConstraint(self.plantar_rod_frame, self.plantar_rod_anchor_point, self.toe_left_frame,
                                         self.toe_left_anchor_point - 2e-3*np.ones(3), self.toe_left_anchor_point + 2e-3*np.ones(3))
    self.toe_angle_constraint = self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[7], 0)
    self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[0:7], np.array([1, 0, 0, 0, 0, 0, 0]))

  def convert_to_drake(self, q, v):
    q_drake = np.zeros(23)
    v_drake = np.zeros(22)
    q_drake = self.map_q_drake_to_mujoco.T @ q

    return q_drake, v_drake

  def convert_to_mujoco(self, x):
    # q_full = np.zeros(35)
    # v_full = np.zeros(32)
    q = x[0:23]
    v = x[23:45]
    q_missing, v_missing = self.solve_IK(x)
    q_copy = self.map_q_drake_to_mujoco @ q
    v_copy = self.map_v_drake_to_mujoco @ v
    q_full = q_missing + q_copy

    v_full = v_missing + v_copy
    return q_full, v_full

  # def solve_foot_angles(self):

  def solve_IK(self, x):
    self.plant.SetPositionsAndVelocities(self.context, x)

    # left_foot_crank_state = np.array([1, 0, 0, 0, 0, 0, 0, -1.5968, -1.5244, 1.5244])
    # left_foot_crank_state = np.array([1, 0, 0, 0, 0, 0, 0, -1.5968, -1.5244, 1.5244])
    # -1.7439499847110957, 1.7254888878481534, -1.8498456916286636
    # left_foot_crank_state = np.array([1, 0, 0, 0, 0, 0, 0, -1.5244, 1.5244, -1.5968])
    # self.foot_crank_plant.SetPositions(self.foot_crank_context, left_foot_crank_state)
    # toe_left_anchor_point = self.foot_crank_plant.CalcPointsPositions(self.foot_crank_context, self.plantar_rod_frame,
    #                                                                   self.plantar_rod_anchor_point,
    #                                                                   self.toe_left_frame)
    # print(toe_left_anchor_point)
    self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[x[self.pos_map['toe_left']]])
    result = Solve(self.ik_solver.prog())
    left_foot_crank_state = result.GetSolution()
    print(result.get_solution_result())
    self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[x[self.pos_map['toe_right']]])
    result = Solve(self.ik_solver.prog())
    right_foot_crank_state = result.GetSolution()
    print(result.get_solution_result())
    # import pdb; pdb.set_trace()
    # self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[7], -1.5968)
    # self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[7], -1.84984)

    # self.plant.CalcRelativeTransform(self.context, self.world)
    left_thigh = self.plant.CalcPointsPositions(self.context, self.left_thigh_rod[1], self.left_thigh_rod[0],
                                                self.world)
    right_thigh = self.plant.CalcPointsPositions(self.context, self.right_thigh_rod[1], self.right_thigh_rod[0],
                                                 self.world)
    left_heel = self.plant.CalcPointsPositions(self.context, self.left_heel_rod[1], self.left_heel_rod[0], self.world)
    right_heel = self.plant.CalcPointsPositions(self.context, self.right_heel_rod[1], self.right_heel_rod[0],
                                                self.world)

    # left_foot_crank = self.plant.CalcPointsPositions(self.context, self.left_foot_crank[1], self.left_foot_crank[0],
    #                                                  self.world)
    # right_foot_crank = self.plant.CalcPointsPositions(self.context, self.right_foot_crank[1], self.right_foot_crank[0],
    #                                                   self.world)
    # left_toe_joint = self.plant.CalcPointsPositions(self.context, self.left_toe_joint[1], self.left_toe_joint[0],
    #                                                 self.world)
    # right_toe_joint = self.plant.CalcPointsPositions(self.context, self.right_toe_joint[1], self.right_toe_joint[0],
    #                                                  self.world)

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
    l_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.left_thigh_rod[1])
    r_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.right_thigh_rod[1])
    l_bar_frame = np.vstack((l_x_vec, l_y_vec, l_z_vec)).T
    r_bar_frame = np.vstack((r_x_vec, r_y_vec, r_z_vec)).T
    l_bar_quat = R.from_dcm(
      self.left_achilles_frame.T @ l_hip_pitch_frame.rotation().matrix().T @ l_bar_frame).as_quat()
    r_bar_quat = R.from_dcm(
      self.right_achilles_frame.T @ r_hip_pitch_frame.rotation().matrix().T @ r_bar_frame).as_quat()
    # l_bar_quat = R.from_dcm(l_bar_frame).as_quat()
    # print(l_bar_quat)
    # l_bar_quat = R.from_dcm(self.left_achilles_frame.T @ l_bar_frame).as_quat()
    # l_bar_quat = R.from_dcm(self.left_achilles_frame @ l_hip_pitch_frame.rotation().matrix() @ l_bar_frame).as_quat()
    # print(l_bar_quat)
    # l_bar_quat = R.from_dcm(np.vstack((l_x_vec, l_y_vec, l_z_vec))).as_quat()
    l_bar_quat = np.hstack((l_bar_quat[3], l_bar_quat[0:3]))
    r_bar_quat = np.hstack((r_bar_quat[3], r_bar_quat[0:3]))

    # l_plantar_x_vec = (left_foot_crank - left_toe_joint)[:, 0]
    # l_plantar_x_vec *= 1 / np.linalg.norm(l_plantar_x_vec)
    # l_plantar_y_vec = np.cross(l_plantar_x_vec, world_frame[0])
    # l_plantar_z_vec = np.cross(l_plantar_x_vec, l_plantar_y_vec)
    # r_plantar_x_vec = (left_foot_crank - left_toe_joint)[:, 0]
    # r_plantar_x_vec *= 1 / np.linalg.norm(r_plantar_x_vec)
    # r_plantar_y_vec = np.cross(r_plantar_x_vec, world_frame[0])
    # r_plantar_z_vec = np.cross(r_plantar_x_vec, r_plantar_y_vec)

    # l_foot_crank_frame = self.plant.CalcRelativeTransform(self.context, self.left_foot_crank[1], self.world)
    # r_foot_crank_frame = self.plant.CalcRelativeTransform(self.context, self.right_foot_crank[1], self.world)
    # l_toe_frame = self.plant.CalcRelativeTransform(self.context, self.left_toe_joint[1], self.world)
    # r_toe_frame = self.plant.CalcRelativeTransform(self.context, self.right_toe_joint[1], self.world)

    # left_foot_crank_ang = math.acos(np.dot(l_plantar_x_vec, l_foot_crank_frame.rotation().matrix()[0]))
    # right_foot_crank_ang = math.acos(np.dot(r_plantar_x_vec, r_foot_crank_frame.rotation().matrix()[0]))
    # left_plantar_rod = math.acos(np.dot(l_plantar_x_vec, l_toe_frame.rotation().matrix()[0]))
    # right_plantar_rod = math.acos(np.dot(r_plantar_x_vec, r_toe_frame.rotation().matrix()[0]))

    q_missing = np.zeros(35)
    v_missing = np.zeros(32)

    # import pdb; pdb.set_trace()
    # l_bar_quat = np.array([[ 0.97850359, -0.0164015, 0.01785328, -0.20479984]])
    # r_bar_quat = np.array([[ 0.97850359, -0.0164015, 0.01785328, -0.20479984]])
    # r_bar_quat = np.array([1, 0, 0, 0])
    left_foot_crank_ang = left_foot_crank_state[8]
    left_plantar_rod = left_foot_crank_state[9]
    right_foot_crank_ang = right_foot_crank_state[8]
    right_plantar_rod = right_foot_crank_state[9]
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

  def generate_matrices(self):
    self.map_q_drake_to_mujoco = np.zeros((35, 23))
    self.map_v_drake_to_mujoco = np.zeros((32, 22))
    self.map_q_drake_to_mujoco[0:3, 4:7] = np.eye(3)
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

    # rot = R.from_quat([q[6], q[3], q[4], q[5]])
    self.map_v_drake_to_mujoco[0:3, 3:6] = np.eye(3)
    self.map_v_drake_to_mujoco[3:6, 0:3] = np.eye(3)
    self.map_v_drake_to_mujoco[6, 6] = 1
    self.map_v_drake_to_mujoco[7, 8] = 1
    self.map_v_drake_to_mujoco[8, 10] = 1
    self.map_v_drake_to_mujoco[12, 12] = 1
    self.map_v_drake_to_mujoco[13, 14] = 1
    self.map_v_drake_to_mujoco[14, 16] = 1
    self.map_v_drake_to_mujoco[15, 18] = 1
    self.map_v_drake_to_mujoco[18, 19] = 1
    self.map_v_drake_to_mujoco[19, 7] = 1
    self.map_v_drake_to_mujoco[20, 9] = 1
    self.map_v_drake_to_mujoco[21, 11] = 1
    self.map_v_drake_to_mujoco[25, 13] = 1
    self.map_v_drake_to_mujoco[26, 16] = 1
    self.map_v_drake_to_mujoco[27, 17] = 1
    self.map_v_drake_to_mujoco[28, 20] = 1
    self.map_v_drake_to_mujoco[31, 21] = 1
