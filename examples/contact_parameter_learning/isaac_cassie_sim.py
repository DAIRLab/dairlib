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

from isaacgym import gymutil
from isaacgym import gymapi


class IsaacCassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 0.05
        self.traj = CassieSimTraj()
        self.valid_ground_truth_trajs = np.arange(0, 29)
        self.hardware_traj = None
        self.default_params = {"mu": 0.8,
                               "restitution": 0.0,
                               "dissipation": 0.5}
        self.armatures = np.array(
            [0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225,
             0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225])

        self.efforts_map = np.zeros((16, 10))
        self.efforts_map[0:4, 0:4] = np.eye(4)
        self.efforts_map[7, 4] = 1
        self.efforts_map[8:12, 5:9] = np.eye(4)
        self.efforts_map[15, 9] = 1

        self.state_converter = DrakeToIsaacConverter()

    def make(self, params, hardware_traj_num, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.gym = gymapi.acquire_gym()

        # Set simulator parameters
        sim_params = gymapi.SimParams()
        sim_params.physx.bounce_threshold_velocity = 2 * 9.81 * self.sim_dt / sim_params.substeps
        sim_params.physx.solver_type = 1  # O: PGS, 1: TGS
        sim_params.physx.num_position_iterations = 4  # [1, 255]
        sim_params.physx.num_velocity_iterations = 1  # [1, 255]
        sim_params.physx.num_threads = 0
        sim_params.physx.use_gpu = 0

        sim_params.dt = self.sim_dt
        sim_params.substeps = 10
        sim_params.use_gpu_pipeline = False
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # add ground plane
        plane_params = gymapi.PlaneParams()

        plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
        plane_params.distance = 0
        plane_params.static_friction = params['mu']
        plane_params.dynamic_friction = params['mu']
        plane_params.restitution = params['restitution']

        self.gym.add_ground(self.sim, plane_params)

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
        np.random.seed(17)

        spacing = 1.0
        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, 2 * spacing)
        self.env = self.gym.create_env(self.sim, lower, upper, 1)

        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.5)
        pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)

        self.actor_handle = self.gym.create_actor(self.env, asset, pose, "cassie_v2", 0, 1)

        #
        self.cassie_dof_props = self.gym.get_actor_dof_properties(self.env, self.actor_handle)
        self.cassie_dof_props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
        self.cassie_dof_props["driveMode"][4:7] = 0
        self.cassie_dof_props["driveMode"][12:15] = 0
        self.cassie_dof_props['armature'] = self.armatures
        self.cassie_dof_props['stiffness'][4] = 1500
        self.cassie_dof_props['stiffness'][6] = 1250
        self.cassie_dof_props['stiffness'][12] = 1500
        self.cassie_dof_props['stiffness'][14] = 1250
        self.gym.set_actor_dof_properties(self.env, self.actor_handle, self.cassie_dof_props)

        if self.visualize:
            cam_pos = gymapi.Vec3(2, 0, 2.0)
            cam_target = gymapi.Vec3(1, 0, -2.5)
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.full_state = np.copy(self.gym.get_actor_rigid_body_states(self.env, self.actor_handle, gymapi.STATE_ALL))
        self.joint_states = self.gym.get_actor_dof_states(self.env, self.actor_handle, gymapi.STATE_ALL)

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

        self.full_state['pose']['p']['x'][0] = qfbpos_init_drake[0]
        self.full_state['pose']['p']['y'][0] = qfbpos_init_drake[1]
        self.full_state['pose']['p']['z'][0] = qfbpos_init_drake[2]
        self.full_state['pose']['r']['w'][0] = qfbquat_init_drake[0]
        self.full_state['pose']['r']['x'][0] = qfbquat_init_drake[1]
        self.full_state['pose']['r']['y'][0] = qfbquat_init_drake[2]
        self.full_state['pose']['r']['z'][0] = qfbquat_init_drake[3]
        self.full_state['vel']['linear']['x'][0] = qfbvel_init_drake[0]
        self.full_state['vel']['linear']['y'][0] = qfbvel_init_drake[1]
        self.full_state['vel']['linear']['z'][0] = qfbvel_init_drake[2]
        self.full_state['vel']['angular']['x'][0] = qfbomega_init_drake[0]
        self.full_state['vel']['angular']['y'][0] = qfbomega_init_drake[1]
        self.full_state['vel']['angular']['z'][0] = qfbomega_init_drake[2]

        self.joint_states['pos'] = qjointpos_init_drake
        self.joint_states['vel'] = qjointvel_init_drake
        self.gym.set_actor_rigid_body_states(self.env, self.actor_handle, self.full_state, gymapi.STATE_ALL)
        self.gym.set_actor_dof_states(self.env, self.actor_handle, self.joint_states, gymapi.STATE_POS)

        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(),
                         self.hardware_traj.get_action(self.start_time))
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while (self.current_time < time):
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.current_time + self.sim_dt
        action = self.hardware_traj.get_action(self.current_time)
        efforts = self.convert_action_to_full_efforts(action)
        self.gym.apply_actor_dof_efforts(self.env, self.actor_handle, efforts)

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        if self.visualize:
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, True)

        joint_state = self.gym.get_actor_dof_states(self.env, self.actor_handle, gymapi.STATE_ALL)
        rigid_body_state = self.gym.get_actor_rigid_body_states(self.env, self.actor_handle, gymapi.STATE_ALL)
        cassie_state = self.convert_isaac_state_to_drake(joint_state, rigid_body_state)

        self.current_time = next_timestep
        self.traj.update(next_timestep, cassie_state, action)
        return cassie_state

    def get_traj(self):
        return self.traj

    def free_sim(self):
        if self.visualize:
            self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)

    def convert_isaac_state_to_drake(self, joint_state, rigid_body_state):
        cassie_state = np.zeros(45)
        cassie_state[CASSIE_JOINT_POSITION_SLICE] = joint_state['pos']
        cassie_state[CASSIE_JOINT_VELOCITY_SLICE] = joint_state['vel']
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

    def convert_action_to_full_efforts(self, action):
        return np.array(self.efforts_map @ action, dtype=np.float32)
