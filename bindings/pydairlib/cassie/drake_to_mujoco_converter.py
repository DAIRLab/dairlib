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
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource


class DrakeToMujocoConverter():

  def __init__(self, drake_sim_dt=5e-5):
    self.builder = DiagramBuilder()
    self.drake_sim_dt = drake_sim_dt
    # Add a cube as MultibodyPlant
    self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, drake_sim_dt)
    addCassieMultibody(self.plant, self.scene_graph, True,
                       "examples/Cassie/urdf/cassie_v2.urdf", False, False)
    self.plant.Finalize()

    # self.foot_crank_plant = MultibodyPlant(drake_sim_dt)
    self.foot_crank_builder = DiagramBuilder()
    self.foot_crank_plant, self.foot_crank_scene_graph = AddMultibodyPlantSceneGraph(self.foot_crank_builder,
                                                                                     drake_sim_dt)
    # DrakeVisualizer.AddToBuilder(self.foot_crank_builder, self.foot_crank_scene_graph)

    Parser(self.foot_crank_plant).AddModelFromFile(FindResourceOrThrow('examples/Cassie/urdf/cassie_foot_crank.urdf'))
    self.foot_crank_plant.Finalize()

    self.knee_linkage_builder = DiagramBuilder()
    self.knee_linkage_plant, self.knee_linkage_scene_graph = AddMultibodyPlantSceneGraph(self.knee_linkage_builder,
                                                                                         drake_sim_dt)
    # DrakeVisualizer.AddToBuilder(self.foot_crank_builder, self.foot_crank_scene_graph)

    Parser(self.knee_linkage_plant).AddModelFromFile(
      FindResourceOrThrow('examples/Cassie/urdf/cassie_knee_linkage.urdf'))
    self.knee_linkage_plant.Finalize()

    self.diagram = self.builder.Build()
    self.diagram_context = self.diagram.CreateDefaultContext()

    # Print the indices for the newly constructed plants
    # self.print_pos_indices(self.knee_linkage_plant)

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
    self.left_achilles_frame[:, 1] = -np.array([0.60349, 0.79547, -0.054922])
    self.left_achilles_frame[:, 2] = np.cross(self.left_achilles_frame[:, 0], self.left_achilles_frame[:, 1])
    self.right_achilles_frame = np.zeros((3, 3))
    self.right_achilles_frame[:, 0] = np.array([0.7922, -0.60599, 0.072096])
    self.right_achilles_frame[:, 1] = -np.array([0.60349, 0.79547, 0.054922])
    self.right_achilles_frame[:, 2] = np.cross(self.right_achilles_frame[:, 0], self.right_achilles_frame[:, 1])

    self.plantar_rod_frame = self.foot_crank_plant.GetBodyByName('plantar_rod_left').body_frame()
    self.toe_left_frame = self.foot_crank_plant.GetBodyByName('toe_left').body_frame()
    self.plantar_rod_anchor_point = np.array([0.35012, 0, 0])
    # self.toe_left_anchor_point = np.array([0.04885482, 0.00394248, 0.01484])
    self.toe_left_anchor_point = np.array([0.05485482, 0, 0.01484])
    # self.toe_left_anchor_point = np.array([0.05512, 0.006, 0.01484])

    # self.ik_solver.AddPositionConstraint(self.plantar_rod_frame, self.plantar_rod_anchor_point, self.toe_left_frame,
    #                                      self.toe_left_anchor_point - 2e-3 * np.ones(3),
    #                                      self.toe_left_anchor_point + 2e-3 * np.ones(3))
    self.ik_solver.AddPositionConstraint(self.plantar_rod_frame, self.plantar_rod_anchor_point, self.toe_left_frame,
                                         self.toe_left_anchor_point,
                                         self.toe_left_anchor_point)
    self.toe_angle_constraint = self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[7], 0)
    self.ik_solver.prog().AddLinearEqualityConstraint(self.ik_solver.q()[0:7], np.array([1, 0, 0, 0, 0, 0, 0]))

  def print_pos_indices(self, plant):
    name_map = makeNameToPositionsMap(plant)
    for name in name_map:
      print(name + ': ' + str(name_map[name]))

  def convert_to_drake(self, q, v):
    q_drake = np.zeros(23)
    v_drake = np.zeros(22)
    q_drake = self.map_q_drake_to_mujoco.T @ q

    return q_drake, v_drake

  def convert_to_mujoco(self, x_i):
    q = x_i[0:23]
    v = x_i[23:45]
    q_missing, v_missing = self.solve_IK(x_i)
    # if x_ip1 is not None:
    #   q_missing_ip1, v_missing_ip1 = self.solve_IK(x_ip1)
    #   v_missing[9:12] = self.estimate_omega_bar(q_missing[10:14], q_missing_ip1[10:14], dt)
    #   v_missing[22:25] = self.estimate_omega_bar(q_missing[24:28], q_missing_ip1[24:28], dt)
    q_copy = self.map_q_drake_to_mujoco @ q
    v_copy = self.map_v_drake_to_mujoco @ v
    q_full = q_missing + q_copy
    v_full = v_missing + v_copy
    # import pdb; pdb.set_trace()
    return q_full, v_full

  def visualize_state_lower(self, x):
    self.plant.SetPositionsAndVelocities(self.context, x)
    # toe_left_ang = x[self.pos_map['toe_left']]
    toe_left_ang = -1.5968
    self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_left_ang])
    self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                          np.array([1, 0, 0, 0, 0, 0, 0, toe_left_ang, toe_left_ang, -toe_left_ang]))
    result = Solve(self.ik_solver.prog())
    left_foot_crank_state = result.GetSolution()
    # print(left_foot_crank_state)
    print(result.get_solution_result())
    builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    plant = MultibodyPlant(self.drake_sim_dt)
    scene_graph = builder.AddSystem(SceneGraph())
    plant_id = plant.RegisterAsSourceForSceneGraph(scene_graph)

    Parser(plant).AddModelFromFile(FindResourceOrThrow('examples/Cassie/urdf/cassie_foot_crank.urdf'))
    # plant.RegisterVisualGeometry(plant.world_body(), X_WG, Box(10, 10, 0.001), "visual", terrain_color)
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    pp_traj = PiecewisePolynomial(left_foot_crank_state)

    traj_source = builder.AddSystem(TrajectorySource(pp_traj))
    q_to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(traj_source.get_output_port(), q_to_pose.get_input_port())
    builder.Connect(q_to_pose.get_output_port(), scene_graph.get_source_pose_port(plant_id))

    # self.builder.Connect(self.foot_crank_plant.get_geometry_poses_output_port(),
    #                      self.foot_crank_scene_graph.get_source_pose_port(self.foot_crank_plant.get_source_id().value()))

    # self.q_to_pose = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))
    # self.builder.Connect(self.q_to_pose.get_output_port(), self.foot_crank_scene_graph.get_source_pose_port(plant_id))
    DrakeVisualizer.AddToBuilder(builder, scene_graph)
    diagram = builder.Build()
    sim = Simulator(diagram)
    sim.set_publish_every_time_step(True)
    sim.get_mutable_context().SetTime(0.0)
    plant.SetPositions(plant_context, left_foot_crank_state)
    sim.Initialize()
    sim.AdvanceTo(0.1)

  def visualize_state_upper(self, x):

    q_missing, _ = self.solve_IK(x)
    quat = q_missing[10:14]

    # print(quat)

    rot = R.from_quat(np.hstack((quat[1:4], quat[0])))
    euler_vec = rot.as_euler('xyz')

    # fb_state = np.array([-0.14644661,  0.35355339,  0.35355339,  0.85355339, 0, 0, 1])
    fb_state = np.array([1, 0, 0, 0, 0, 0, 0])
    # knee_linkage_state = np.array([-0.5,  0.5,  0.5,  0.5, 0, 0, 1, -0.03944253, 0.02825365, -0.41339635, -1.1997, 0, 1.4267, 0])
    knee_linkage_state = np.hstack((fb_state, np.array([-0.03944253, 0.02825365, -0.41339635, -1.1997, 0, 1.4267, 0])))
    knee_linkage_state[7:10] = euler_vec
    knee_linkage_state[10] = x[13]
    knee_linkage_state[11] = x[15]
    knee_linkage_state[12] = x[17]
    knee_linkage_state[13] = x[19]
    # knee_linkage_state = np.array([1, 0, 0, 0, 0, 0, 0, -0.03944253,  0.02825365, -0.41339635, -1.1997, 0, 1.4267, 0])
    # knee_linkage_state = np.array([1, 0, 0, 0, 0, 0, 0, -0.03944253,  0.02825365,  -0.41339635, -1.1997, 0, 1.4267, 0])

    builder = DiagramBuilder()

    # Add a cube as MultibodyPlant
    plant = MultibodyPlant(self.drake_sim_dt)
    scene_graph = builder.AddSystem(SceneGraph())
    plant_id = plant.RegisterAsSourceForSceneGraph(scene_graph)

    Parser(plant).AddModelFromFile(FindResourceOrThrow('examples/Cassie/urdf/cassie_knee_linkage.urdf'))
    # plant.RegisterVisualGeometry(plant.world_body(), X_WG, Box(10, 10, 0.001), "visual", terrain_color)
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    plant.SetPositions(plant_context, knee_linkage_state)

    # offset = plant.CalcPointsPositions(plant_context, plant.GetBodyByName('thigh_left').body_frame(), np.array([0, 0, 0.045]), plant.GetBodyByName('heel_spring_left').body_frame())
    hip_mount = plant.CalcPointsPositions(plant_context, plant.GetBodyByName('thigh_left').body_frame(),
                                          np.array([0, 0, 0.045]), plant.world_frame())
    heel_spring_mount = plant.CalcPointsPositions(plant_context, plant.GetBodyByName('heel_spring_left').body_frame(),
                                                  np.array([.11877, -.01, 0.0]), plant.world_frame())
    achilles_end_point = plant.CalcPointsPositions(plant_context, plant.GetBodyByName('achilles_rod_left').body_frame(),
                                                   np.array([0.5012, 0, 0]), plant.world_frame())
    # print(achilles_end_point)
    # print(heel_spring_mount)
    print(np.linalg.norm(hip_mount - heel_spring_mount))
    # print(np.linalg.norm(achilles_end_point - heel_spring_mount))

    pp_traj = PiecewisePolynomial(knee_linkage_state)

    traj_source = builder.AddSystem(TrajectorySource(pp_traj))
    q_to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(traj_source.get_output_port(), q_to_pose.get_input_port())
    builder.Connect(q_to_pose.get_output_port(), scene_graph.get_source_pose_port(plant_id))

    # self.builder.Connect(self.foot_crank_plant.get_geometry_poses_output_port(),
    #                      self.foot_crank_scene_graph.get_source_pose_port(self.foot_crank_plant.get_source_id().value()))

    # self.q_to_pose = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))
    # self.builder.Connect(self.q_to_pose.get_output_port(), self.foot_crank_scene_graph.get_source_pose_port(plant_id))
    DrakeVisualizer.AddToBuilder(builder, scene_graph)
    diagram = builder.Build()
    sim = Simulator(diagram)
    sim.set_publish_every_time_step(True)
    sim.get_mutable_context().SetTime(0.0)
    # plant.SetPositions(plant_context, knee_linkage_state)
    sim.Initialize()
    sim.AdvanceTo(0.1)

    # import pdb; pdb.set_trace()

  def estimate_omega_bar(self, quat1, quat2, dt):
    R1 = R.from_quat(np.hstack((quat1[1:4], quat1[0])))
    R2 = R.from_quat(np.hstack((quat2[1:4], quat2[0])))
    R_rel = R1.inv() * R2
    omega = R_rel.as_rotvec() / dt
    return omega


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

    world_frame = np.eye(3)
    l_x_vec = (left_heel - left_thigh)[:, 0]
    l_x_vec *= 1 / np.linalg.norm(l_x_vec)
    l_y_vec = np.cross(l_x_vec, world_frame[1])
    l_y_vec *= 1 / np.linalg.norm(l_y_vec)
    l_z_vec = np.cross(l_x_vec, l_y_vec)
    l_z_vec *= 1 / np.linalg.norm(l_z_vec)

    r_x_vec = (right_heel - right_thigh)[:, 0]
    r_x_vec *= 1 / np.linalg.norm(r_x_vec)
    r_y_vec = np.cross(r_x_vec, world_frame[1])
    r_y_vec *= 1 / np.linalg.norm(r_y_vec)
    r_z_vec = np.cross(r_x_vec, r_y_vec)
    r_z_vec *= 1 / np.linalg.norm(r_z_vec)

    l_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.left_thigh_rod[1])
    r_hip_pitch_frame = self.plant.CalcRelativeTransform(self.context, self.world, self.right_thigh_rod[1])
    l_bar_frame = R.from_matrix(np.vstack((l_x_vec, l_y_vec, l_z_vec)).T)
    r_bar_frame = R.from_matrix(np.vstack((r_x_vec, r_y_vec, r_z_vec)).T)
    l_bar_euler = R.from_matrix(
      self.left_achilles_frame @ l_hip_pitch_frame.rotation().matrix().T @ l_bar_frame.as_matrix()).as_euler('xyz')
    r_bar_euler = R.from_matrix(
      self.right_achilles_frame @ r_hip_pitch_frame.rotation().matrix().T @ r_bar_frame.as_matrix()).as_euler('xyz')

    # l_rot = R.from_quat(l_bar_quat)
    # r_rot = R.from_quat(l_bar_quat)
    # l_euler_vec = l_rot.as_euler('xyz')
    l_bar_quat = R.from_euler('xyz', -l_bar_euler).as_quat()
    r_bar_quat = R.from_euler('xyz', -r_bar_euler).as_quat()
    # r_bar_quat = l_rot.as_quat()
    l_bar_quat = np.hstack((l_bar_quat[3], l_bar_quat[0:3]))
    r_bar_quat = np.hstack((r_bar_quat[3], r_bar_quat[0:3]))

    return l_bar_quat, r_bar_quat

  def solve_IK(self, x):
    toe_ang = x[self.pos_map['toe_left']]
    self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_ang])
    self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                          np.array([1, 0, 0, 0, 0, 0, 0, toe_ang, toe_ang, -toe_ang]))
    result = Solve(self.ik_solver.prog())
    left_foot_crank_state = result.GetSolution()
    # print(result.get_solution_result())
    toe_ang = x[self.pos_map['toe_right']]
    self.toe_angle_constraint.evaluator().UpdateCoefficients(Aeq=[[1]], beq=[toe_ang])
    self.ik_solver.prog().SetInitialGuess(self.ik_solver.q(),
                                          np.array([1, 0, 0, 0, 0, 0, 0, toe_ang, toe_ang, -toe_ang]))
    result = Solve(self.ik_solver.prog())
    right_foot_crank_state = result.GetSolution()
    # print(result.get_solution_result())

    q = x[:self.plant.num_positions()]
    v = x[-self.plant.num_velocities():]
    l_bar_quat, r_bar_quat = self.solve_for_achilles_rod_quats(q)
    q_dt = q + self.plant.MapVelocityToQDot(self.context, v) * self.drake_sim_dt
    l_bar_quat_dt, r_bar_quat_dt = self.solve_for_achilles_rod_quats(q_dt)

    l_bar_omega = self.estimate_omega_bar(l_bar_quat, l_bar_quat_dt, self.drake_sim_dt)
    r_bar_omega = self.estimate_omega_bar(r_bar_quat, r_bar_quat_dt, self.drake_sim_dt)

    # l_bar_quat = np.array([0.9785, -0.0164, 0.01787, -0.2049])
    # r_bar_quat = np.array([0.9786, 0.00386, -0.01524, -0.2051])

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
