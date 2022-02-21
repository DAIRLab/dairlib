"""
Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

(TODO) figure out licensing before submitting to IROS
"""

import numpy as np

from cassie_sim_data.cassie_traj import *
from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.drake_to_isaac_converter import DrakeToIsaacConverter
from pydairlib.cassie.isaac_spring_constraint import IsaacSpringConstraint

from isaacgym import gymutil, gymapi, gymtorch
from isaacgym.torch_utils import *


class IsaacCassieSim:

    def __init__(self, visualize=False):
        # changing the sim_dt from 5e-5 to 5e-4 does not change the trajectory but greatly improves the runtime
        self.sim_dt = 5e-5
        self.dt = 5e-4
        self.substeps = 10
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 0.05
        self.traj = CassieSimTraj()
        self.hardware_traj = None
        self.box_size = 2.0
        self.default_params = {"mu": 0.1,
                               "stiffness": 100.0,
                               "restitution": 0.0}

        self.armatures = np.array(
            [0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225,
             0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225])

        self.efforts_map = np.zeros((16, 10))
        self.efforts_map[0:4, 0:4] = np.eye(4)
        self.efforts_map[7, 4] = 1
        self.efforts_map[8:12, 5:9] = np.eye(4)
        self.efforts_map[15, 9] = 1

        self.kCassieAchillesLength = 0.5012
        self.achilles_stiffness = 1e6
        self.achilles_damping = 7e2

        self.state_converter = DrakeToIsaacConverter()

        self.gym = gymapi.acquire_gym()

        # Set simulator parameters
        sim_params = gymapi.SimParams()
        sim_params.dt = self.sim_dt
        sim_params.substeps = self.substeps
        sim_params.use_gpu_pipeline = False
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        sim_params.physx.bounce_threshold_velocity = 2 * 9.81 * self.sim_dt / sim_params.substeps
        sim_params.physx.solver_type = 1  # O: PGS, 1: TGS
        sim_params.physx.num_position_iterations = 10  # [1, 255]
        sim_params.physx.num_velocity_iterations = 10  # [1, 255]
        sim_params.physx.num_threads = 0
        sim_params.physx.use_gpu = 0
        # sim_params.physx.contact_collection = 2  # 0: never, 1: last sub-step, 2: all sub-steps (default=2)
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # add ground plane
        # plane_params = gymapi.PlaneParams()
        # plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
        # plane_params.distance = 0
        # plane_params.static_friction = params['mu']
        # plane_params.dynamic_friction = params['mu']
        # plane_params.restitution = params['restitution']
        # self.gym.add_ground(self.sim, plane_params)

        # add visualization
        if self.visualize:
            self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
            if self.viewer is None:
                print("*** Failed to create viewer")
                quit()

        # create cassie plant
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_root = "examples/Cassie/"
        asset_file = "urdf/cassie_v2.urdf"
        asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

        spacing = 1.0
        lower = gymapi.Vec3(-spacing, -spacing, -spacing)
        upper = gymapi.Vec3(spacing, spacing, 2 * spacing)
        self.env = self.gym.create_env(self.sim, lower, upper, 1)

        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.5)
        pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)

        self.cassie_handle = self.gym.create_actor(self.env, asset, pose, "cassie_v2", 0, 1)
        self.body_indices = self.gym.get_actor_rigid_body_dict(self.env, self.cassie_handle)

        self.left_loop_closure = IsaacSpringConstraint(self.body_indices['thigh_left'],
                                                       self.body_indices['heel_spring_left'],
                                                       gymapi.Vec3(0.0, 0.0, 0.045), gymapi.Vec3(.11877, -.01, 0.0),
                                                       self.kCassieAchillesLength, self.achilles_stiffness,
                                                       self.achilles_damping)
        self.right_loop_closure = IsaacSpringConstraint(self.body_indices['thigh_right'],
                                                        self.body_indices['heel_spring_right'],
                                                        gymapi.Vec3(0.0, 0.0, -0.045), gymapi.Vec3(.11877, -.01, 0.0),
                                                        self.kCassieAchillesLength, self.achilles_stiffness,
                                                        self.achilles_damping)

        self.cassie_dof_props = self.gym.get_actor_dof_properties(self.env, self.cassie_handle)
        self.cassie_dof_props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
        self.cassie_dof_props["driveMode"][4:7] = np.full(3, gymapi.DOF_MODE_NONE)
        self.cassie_dof_props["driveMode"][12:15] = np.full(3, gymapi.DOF_MODE_NONE)
        self.cassie_dof_props['armature'] = self.armatures
        self.cassie_dof_props['stiffness'][4] = 1500
        self.cassie_dof_props['stiffness'][6] = 1250
        self.cassie_dof_props['stiffness'][12] = 1500
        self.cassie_dof_props['stiffness'][14] = 1250
        self.gym.set_actor_dof_properties(self.env, self.cassie_handle, self.cassie_dof_props)

        plane_pose = gymapi.Transform()
        plane_pose.p = gymapi.Vec3(0.0, 0.0, -0.05)
        plane_pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        plane = self.gym.create_box(self.sim, self.box_size, self.box_size, 0.1, asset_options)
        self.plane_handle = self.gym.create_actor(self.env, plane, plane_pose, 'plane', 0, 0)


    def make(self, params, hardware_traj_num, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        shape_props = self.gym.get_actor_rigid_shape_properties(self.env, self.plane_handle)
        shape_props[0].friction = params['mu']
        shape_props[0].compliance = params['stiffness']  # compliance
        shape_props[0].restitution = params['restitution']  # damping equivalent
        success = self.gym.set_actor_rigid_shape_properties(self.env, self.plane_handle, shape_props)
        self.actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)
        self.root_states = gymtorch.wrap_tensor(self.actor_root_state)

        if self.visualize:
            cam_pos = gymapi.Vec3(2, 0, 2.0)
            cam_target = gymapi.Vec3(1, 0, -2.5)
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.joint_states = np.copy(self.gym.get_actor_dof_states(self.env, self.cassie_handle, gymapi.STATE_ALL))
        self.current_time = 0.0
        self.reset(hardware_traj_num)

    def reset(self, hardware_traj_num):
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.traj = CassieSimTraj()

        x_init_drake = self.hardware_traj.get_initial_state()
        qjointpos_init_drake = self.state_converter.map_drake_pos_to_isaac_joint_pos(
            x_init_drake[:CASSIE_NQ])
        qjointvel_init_drake = self.state_converter.map_drake_vel_to_isaac_joint_vel(
            x_init_drake[-CASSIE_NV:])
        qfbpos_init_drake = x_init_drake[CASSIE_FB_POSITION_SLICE]
        qfbvel_init_drake = x_init_drake[CASSIE_FB_VELOCITY_SLICE]
        qfbquat_init_drake = x_init_drake[CASSIE_QUATERNION_SLICE]
        qfbomega_init_drake = x_init_drake[CASSIE_OMEGA_SLICE]

        self.root_states[self.cassie_handle, :3] = to_torch(qfbpos_init_drake)
        self.root_states[self.cassie_handle, 3:7] = to_torch(
            [qfbquat_init_drake[1], qfbquat_init_drake[2], qfbquat_init_drake[3], qfbquat_init_drake[0]])
        self.root_states[self.cassie_handle, 7:10] = to_torch(qfbvel_init_drake)
        self.root_states[self.cassie_handle, 10:13] = to_torch(qfbomega_init_drake)

        self.root_states[self.plane_handle, :3] = to_torch([0., 0., -0.05])
        self.root_states[self.plane_handle, 3:7] = to_torch([0., 0., 0., 1.])
        self.root_states[self.plane_handle, 7:13] = to_torch([0., 0., 0., 0., 0., 0.])

        self.joint_states['pos'] = qjointpos_init_drake
        self.joint_states['vel'] = qjointvel_init_drake
        # self.gym.set_actor_rigid_body_states(self.env, self.actor_handle, self.full_state, gymapi.STATE_ALL)
        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))
        self.gym.set_actor_dof_states(self.env, self.cassie_handle, self.joint_states, gymapi.STATE_ALL)

        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(),
                         self.hardware_traj.get_action(self.start_time))
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while self.current_time < time:
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.current_time + self.dt
        action = self.hardware_traj.get_action(next_timestep)
        efforts = np.array(self.state_converter.map_drake_effort_to_isaac(action), dtype=np.float32)
        for i in range(int(self.dt / self.sim_dt)):
            self.gym.apply_actor_dof_efforts(self.env, self.cassie_handle, efforts)
            self.left_loop_closure.CalcAndAddForceContribution(self.gym, self.env)
            self.right_loop_closure.CalcAndAddForceContribution(self.gym, self.env)
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)

        if self.visualize:
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, True)

        joint_state = self.gym.get_actor_dof_states(self.env, self.cassie_handle, gymapi.STATE_ALL)
        rigid_body_state = self.gym.get_actor_rigid_body_states(self.env, self.cassie_handle, gymapi.STATE_ALL)
        cassie_state = self.convert_isaac_state_to_drake(joint_state, rigid_body_state)
        self.current_time = next_timestep
        self.traj.update(next_timestep, cassie_state, action)
        return cassie_state

    def get_traj(self):
        return self.traj

    def free_sim(self):
        pass
        # Freeing sim doesn't do anything
        # https://forums.developer.nvidia.com/t/not-all-vram-is-freed-after-the-simulation-is-destroyed/188329/2
        # if self.visualize:
        #     self.gym.destroy_viewer(self.viewer)
        # self.gym.destroy_sim(self.sim)

    def convert_isaac_state_to_drake(self, joint_state, rigid_body_state):
        cassie_state = np.zeros(45)
        cassie_state[:CASSIE_NQ] = self.state_converter.map_isaac_joint_pos_to_drake_pos(
            joint_state['pos'])
        cassie_state[-CASSIE_NV:] = self.state_converter.map_isaac_joint_vel_to_drake_vel(
            joint_state['vel'])
        pelvis_pose = rigid_body_state['pose']['p'][0]
        pelvis_quat = rigid_body_state['pose']['r'][0]
        pelvis_linear_vel = rigid_body_state['vel']['linear'][0]
        pelvis_angular_vel = rigid_body_state['vel']['angular'][0]
        cassie_state[CASSIE_FB_POSITION_SLICE] = np.array([pelvis_pose[0], pelvis_pose[1], pelvis_pose[2]])
        cassie_state[CASSIE_QUATERNION_SLICE] = np.array(
            [pelvis_quat[3], pelvis_quat[0], pelvis_quat[1], pelvis_quat[2]])
        cassie_state[CASSIE_FB_VELOCITY_SLICE] = np.array(
            [pelvis_linear_vel[0], pelvis_linear_vel[1], pelvis_linear_vel[2]])
        cassie_state[CASSIE_OMEGA_SLICE] = np.array(
            [pelvis_angular_vel[0], pelvis_angular_vel[1], pelvis_angular_vel[2]])
        return cassie_state
