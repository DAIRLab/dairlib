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


class DrakeToMujocoConverter():

    def __init__(self, drake_sim_dt=5e-5):
        self.builder = DiagramBuilder()
        self.drake_sim_dt = drake_sim_dt
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
        addCassieMultibody(self.plant, self.scene_graph, True,
                           "examples/Cassie/urdf/cassie_v2.urdf", False, False)
        self.plant.Finalize()

        self.foot_crank_builder = DiagramBuilder()
        self.foot_crank_plant, self.foot_crank_scene_graph = AddMultibodyPlantSceneGraph(self.foot_crank_builder,
                                                                                         drake_sim_dt)

        Parser(self.foot_crank_plant).AddModelFromFile(
            FindResourceOrThrow('examples/Cassie/urdf/cassie_foot_crank.urdf'))
        self.foot_crank_plant.Finalize()

        self.knee_linkage_builder = DiagramBuilder()
        self.knee_linkage_plant, self.knee_linkage_scene_graph = AddMultibodyPlantSceneGraph(self.knee_linkage_builder,
                                                                                             drake_sim_dt)

        Parser(self.knee_linkage_plant).AddModelFromFile(
            FindResourceOrThrow('examples/Cassie/urdf/cassie_knee_linkage.urdf'))
        self.knee_linkage_plant.Finalize()

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()

        # Print the indices for the newly constructed plants
        # self.print_pos_indices(self.knee_linkage_plant)
        # self.print_pos_indices(self.plant)

        self.pos_map = makeNameToPositionsMap(self.plant)
        self.vel_map = makeNameToVelocitiesMap(self.plant)
        self.act_map = makeNameToActuatorsMap(self.plant)

        self.map_q_drake_to_mujoco = np.zeros((23, 35))
        self.map_v_drake_to_mujoco = np.zeros((22, 32))

        self.generate_matrices()

        self.world = self.plant.world_frame()
        self.context = self.plant.CreateDefaultContext()
        self.foot_crank_context = self.foot_crank_plant.CreateDefaultContext()
        self.ik_solver = InverseKinematics(self.foot_crank_plant, self.foot_crank_context, with_joint_limits=True)
        self.left_thigh_rod = LeftRodOnThigh(self.plant)
        self.right_thigh_rod = RightRodOnThigh(self.plant)
        self.left_heel_rod = LeftRodOnHeel(self.plant)
        self.right_heel_rod = RightRodOnHeel(self.plant)

        self.left_achilles_frame = np.zeros((3, 3))
        self.left_achilles_frame[:, 0] = np.array([0.7922, -0.60599, -0.072096])
        self.left_achilles_frame[:, 1] = -np.array([0.60349, 0.79547, -0.054922])
        self.left_achilles_frame[:, 2] = np.cross(self.left_achilles_frame[:, 0], self.left_achilles_frame[:, 1])
        self.right_achilles_frame = np.zeros((3, 3))
        self.right_achilles_frame[:, 0] = np.array([0.7922, -0.60599, 0.072096])
        self.right_achilles_frame[:, 1] = -np.array([0.60349, 0.79547, 0.054922])
        self.right_achilles_frame[:, 2] = np.cross(self.right_achilles_frame[:, 0], self.right_achilles_frame[:, 1])

        self.plantar_rod_frame = self.foot_crank_plant.GetBodyByName('plantar_rod_left').body_frame()
        self.toe_left_frame = self.foot_crank_plant.GetBodyByName('toe_left').body_frame()
        self.plantar_rod_anchor_point = np.array([0.35012, 0, 0])
        self.toe_left_anchor_point = np.array([0.04885482, 0.00394248, 0.01484])

        self.ik_solver.AddPositionConstraint(self.plantar_rod_frame, self.plantar_rod_anchor_point, self.toe_left_frame,
                                             self.toe_left_anchor_point,
                                             self.toe_left_anchor_point)
        self.toe_angle_constraint = self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[7], 0)
        self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[0:7], np.array([1, 0, 0, 0, 0, 0, 0]))

    def print_pos_indices(self, plant):
        name_map = makeNameToPositionsMap(plant)
        for name in name_map:
            print(name + ': ' + str(name_map[name]))

    def print_vel_indices(self, plant):
        name_map = makeNameToVelocitiesMap(plant)
        for name in name_map:
            print(name + ': ' + str(name_map[name]))

    def convert_to_drake(self, q, v):
        q_drake = np.zeros(23)
        v_drake = np.zeros(22)
        q_drake = self.map_q_drake_to_mujoco.T @ q
        v_drake = self.map_v_drake_to_mujoco.T @ v

        return q_drake, v_drake

    def convert_to_mujoco(self, x_i):
        q = x_i[0:23]
        v = x_i[23:45]
        q_missing, v_missing = self.solve_IK(x_i)

        q_copy = self.map_q_drake_to_mujoco @ q
        v_copy = self.map_v_drake_to_mujoco @ v
        q_full = q_missing + q_copy
        v_full = v_missing + v_copy
        return q_full, v_full

    def visualize_entire_leg(self, x):
        self.plant.SetPositionsAndVelocities(self.context, x)
        q_missing, _ = self.solve_IK(x)
        quat = q_missing[10:14]

        rot = R.from_quat(np.hstack((quat[1:4], quat[0])))
        euler_vec = rot.as_euler('xyz')

        toe_left_ang = x[self.pos_map['toe_left']]
        self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_left_ang])
        self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                              np.array(
                                                  [1, 0, 0, 0, 0, 0, 0, toe_left_ang, toe_left_ang, -toe_left_ang]))
        result = Solve(self.ik_solver.prog())
        left_foot_crank_state = result.GetSolution()

        builder = DiagramBuilder()

        plant = MultibodyPlant(self.drake_sim_dt)
        scene_graph = builder.AddSystem(SceneGraph())

        Parser(plant).AddModelFromFile(FindResourceOrThrow('examples/Cassie/urdf/cassie_left_leg.urdf'))
        plant.Finalize()
        self.print_pos_indices(plant)

        plant_context = plant.CreateDefaultContext()
        print(plant.num_positions())
        fb_state = np.array([1, 0, 0, 0, 0, 0, 0])
        left_leg_state = np.zeros(17)
        left_leg_state[0:7] = fb_state
        left_leg_state[7:10] = euler_vec
        left_leg_state[10] = x[self.pos_map["knee_left"]]
        left_leg_state[11] = x[self.pos_map["knee_joint_left"]]
        left_leg_state[12] = x[self.pos_map["ankle_joint_left"]]
        left_leg_state[13] = x[self.pos_map["ankle_spring_joint_left"]]
        left_leg_state[14:17] = left_foot_crank_state[-3:]
        plant.SetPositions(plant_context, left_leg_state)

        visualizer = MultiposeVisualizer('examples/Cassie/urdf/cassie_left_leg.urdf', 1, '')
        visualizer.DrawPoses(left_leg_state)

    def visualize_state_lower(self, x):
        self.plant.SetPositionsAndVelocities(self.context, x)
        toe_left_ang = x[self.pos_map['toe_left']]
        self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_left_ang])
        self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                              np.array(
                                                  [1, 0, 0, 0, 0, 0, 0, toe_left_ang, toe_left_ang, -toe_left_ang]))
        result = Solve(self.ik_solver.prog())
        left_foot_crank_state = result.GetSolution()
        visualizer = MultiposeVisualizer('examples/Cassie/urdf/cassie_foot_crank.urdf', 1, '')
        visualizer.DrawPoses(left_foot_crank_state)

    def visualize_state_upper(self, x):
        q_missing, _ = self.solve_IK(x)
        quat = q_missing[10:14]

        rot = R.from_quat(np.hstack((quat[1:4], quat[0])))
        euler_vec = rot.as_euler('xyz')

        fb_state = np.array([1, 0, 0, 0, 0, 0, 0])
        knee_linkage_state = np.zeros(14)
        knee_linkage_state[0:7] = fb_state
        knee_linkage_state[7:10] = euler_vec
        knee_linkage_state[10] = x[self.pos_map["knee_left"]]
        knee_linkage_state[11] = x[self.pos_map["knee_joint_left"]]
        knee_linkage_state[12] = x[self.pos_map["ankle_joint_left"]]
        knee_linkage_state[13] = x[self.pos_map["ankle_spring_joint_left"]]

        visualizer = MultiposeVisualizer('examples/Cassie/urdf/cassie_knee_linkage.urdf', 1, '')
        visualizer.DrawPoses(knee_linkage_state)

    def estimate_omega_bar(self, quat1, quat2, dt):
        R1 = R.from_quat(np.hstack((quat1[1:4], quat1[0])))
        R2 = R.from_quat(np.hstack((quat2[1:4], quat2[0])))
        R_rel = R1.inv() * R2
        omega = R_rel.as_rotvec() / dt
        return R1.apply(omega)

    def solve_for_achilles_rod_quats(self, q):
        self.plant.SetPositions(self.context, q)
        left_thigh = self.plant.CalcPointsPositions(self.context, self.left_thigh_rod[1], self.left_thigh_rod[0],
                                                    self.world)
        right_thigh = self.plant.CalcPointsPositions(self.context, self.right_thigh_rod[1], self.right_thigh_rod[0],
                                                     self.world)
        left_heel = self.plant.CalcPointsPositions(self.context, self.left_heel_rod[1], self.left_heel_rod[0],
                                                   self.world)
        right_heel = self.plant.CalcPointsPositions(self.context, self.right_heel_rod[1], self.right_heel_rod[0],
                                                    self.world)
        l_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.left_thigh_rod[1])
        r_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.right_thigh_rod[1])

        world_frame = np.eye(3)
        l_x_vec = (left_heel - left_thigh)[:, 0]
        l_x_vec *= 1 / np.linalg.norm(l_x_vec)
        l_y_vec = np.cross(l_x_vec, -world_frame[1])
        l_y_vec *= 1 / np.linalg.norm(l_y_vec)
        l_z_vec = np.cross(l_x_vec, l_y_vec)
        l_z_vec *= 1 / np.linalg.norm(l_z_vec)

        r_x_vec = (right_heel - right_thigh)[:, 0]
        r_x_vec *= 1 / np.linalg.norm(r_x_vec)
        r_y_vec = np.cross(r_x_vec, -world_frame[1])
        r_y_vec *= 1 / np.linalg.norm(r_y_vec)
        r_z_vec = np.cross(r_x_vec, r_y_vec)
        r_z_vec *= 1 / np.linalg.norm(r_z_vec)

        l_bar_frame = R.from_matrix(np.vstack((l_x_vec, l_y_vec, l_z_vec)).T)
        r_bar_frame = R.from_matrix(np.vstack((r_x_vec, r_y_vec, r_z_vec)).T)
        l_bar_euler = R.from_matrix(
            self.left_achilles_frame @ l_hip_pitch_frame.rotation().matrix().T @ l_bar_frame.as_matrix()).as_euler(
            'xyz')
        r_bar_euler = R.from_matrix(
            self.right_achilles_frame @ r_hip_pitch_frame.rotation().matrix().T @ r_bar_frame.as_matrix()).as_euler(
            'xyz')

        l_bar_quat = R.from_euler('xyz', -l_bar_euler).as_quat()
        r_bar_quat = R.from_euler('xyz', -r_bar_euler).as_quat()
        l_bar_quat = np.hstack((l_bar_quat[3], l_bar_quat[0:3]))
        r_bar_quat = np.hstack((r_bar_quat[3], r_bar_quat[0:3]))

        return l_bar_quat, r_bar_quat, l_hip_pitch_frame, r_hip_pitch_frame

    def solve_IK(self, x):
        toe_ang = x[self.pos_map['toe_left']]
        self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_ang])
        self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                              np.array([1, 0, 0, 0, 0, 0, 0, toe_ang, toe_ang, -toe_ang]))
        result = Solve(self.ik_solver.prog())
        left_foot_crank_state = result.GetSolution()
        toe_ang = x[self.pos_map['toe_right']]
        self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_ang])
        self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                              np.array([1, 0, 0, 0, 0, 0, 0, toe_ang, toe_ang, -toe_ang]))
        result = Solve(self.ik_solver.prog())
        right_foot_crank_state = result.GetSolution()

        q = x[:self.plant.num_positions()]
        v = x[-self.plant.num_velocities():]
        l_bar_quat, r_bar_quat, l_hip_pitch_frame, r_hip_pitch_frame = self.solve_for_achilles_rod_quats(q)
        q_dt = q + self.plant.MapVelocityToQDot(self.context, v) * self.drake_sim_dt
        l_bar_quat_dt, r_bar_quat_dt, _, _ = self.solve_for_achilles_rod_quats(q_dt)

        l_bar_omega = self.estimate_omega_bar(l_bar_quat, l_bar_quat_dt, self.drake_sim_dt)
        r_bar_omega = self.estimate_omega_bar(r_bar_quat, r_bar_quat_dt, self.drake_sim_dt)

        l_bar_quat_gt = np.array([0.9785, -0.0164, 0.01787, -0.2049])
        r_bar_quat_gt = np.array([0.9786, 0.00386, -0.01524, -0.2051])

        q_missing = np.zeros(35)
        v_missing = np.zeros(32)

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

        v_missing[16] = x[23 + self.vel_map['toe_leftdot']]
        v_missing[17] = -x[23 + self.vel_map['toe_leftdot']]
        v_missing[29] = x[23 + self.vel_map['toe_rightdot']]
        v_missing[30] = -x[23 + self.vel_map['toe_rightdot']]

        v_missing[9:12] = l_bar_omega
        v_missing[22:25] = r_bar_omega

        return q_missing, v_missing

    def generate_matrices(self):
        self.map_q_drake_to_mujoco = np.zeros((35, 23))
        self.map_v_drake_to_mujoco = np.zeros((32, 22))
        self.map_q_drake_to_mujoco[0:3, 4:7] = np.eye(3)
        self.map_q_drake_to_mujoco[3:7, 0:4] = np.eye(4)
        self.map_q_drake_to_mujoco[7, self.pos_map["hip_roll_left"]] = 1
        self.map_q_drake_to_mujoco[8, self.pos_map["hip_yaw_left"]] = 1
        self.map_q_drake_to_mujoco[9, self.pos_map["hip_pitch_left"]] = 1
        self.map_q_drake_to_mujoco[14, self.pos_map["knee_left"]] = 1
        self.map_q_drake_to_mujoco[15, self.pos_map["knee_joint_left"]] = 1
        self.map_q_drake_to_mujoco[16, self.pos_map["ankle_joint_left"]] = 1
        self.map_q_drake_to_mujoco[17, self.pos_map["ankle_spring_joint_left"]] = 1
        self.map_q_drake_to_mujoco[20, self.pos_map["toe_left"]] = 1
        self.map_q_drake_to_mujoco[21, self.pos_map["hip_roll_right"]] = 1
        self.map_q_drake_to_mujoco[22, self.pos_map["hip_yaw_right"]] = 1
        self.map_q_drake_to_mujoco[23, self.pos_map["hip_pitch_right"]] = 1
        self.map_q_drake_to_mujoco[28, self.pos_map["knee_right"]] = 1
        self.map_q_drake_to_mujoco[29, self.pos_map["knee_joint_right"]] = 1
        self.map_q_drake_to_mujoco[30, self.pos_map["ankle_joint_right"]] = 1
        self.map_q_drake_to_mujoco[31, self.pos_map["ankle_spring_joint_right"]] = 1
        self.map_q_drake_to_mujoco[34, self.pos_map["toe_right"]] = 1

        self.map_v_drake_to_mujoco[0:3, 3:6] = np.eye(3)
        self.map_v_drake_to_mujoco[3:6, 0:3] = np.eye(3)
        self.map_v_drake_to_mujoco[6, self.vel_map["hip_roll_leftdot"]] = 1
        self.map_v_drake_to_mujoco[7, self.vel_map["hip_yaw_leftdot"]] = 1
        self.map_v_drake_to_mujoco[8, self.vel_map["hip_pitch_leftdot"]] = 1
        self.map_v_drake_to_mujoco[12, self.vel_map["knee_leftdot"]] = 1
        self.map_v_drake_to_mujoco[13, self.vel_map["knee_joint_leftdot"]] = 1
        self.map_v_drake_to_mujoco[14, self.vel_map["ankle_joint_leftdot"]] = 1
        self.map_v_drake_to_mujoco[15, self.vel_map["ankle_spring_joint_leftdot"]] = 1
        self.map_v_drake_to_mujoco[18, self.vel_map["toe_leftdot"]] = 1
        self.map_v_drake_to_mujoco[19, self.vel_map["hip_roll_rightdot"]] = 1
        self.map_v_drake_to_mujoco[20, self.vel_map["hip_yaw_rightdot"]] = 1
        self.map_v_drake_to_mujoco[21, self.vel_map["hip_pitch_rightdot"]] = 1
        self.map_v_drake_to_mujoco[25, self.vel_map["knee_rightdot"]] = 1
        self.map_v_drake_to_mujoco[26, self.vel_map["knee_joint_rightdot"]] = 1
        self.map_v_drake_to_mujoco[27, self.vel_map["ankle_joint_rightdot"]] = 1
        self.map_v_drake_to_mujoco[28, self.vel_map["ankle_spring_joint_rightdot"]] = 1
        self.map_v_drake_to_mujoco[31, self.vel_map["toe_rightdot"]] = 1
