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

from isaacgym import gymutil, gymapi, gymtorch
from isaacgym.torch_utils import *

from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, \
    CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, \
    CUBE_DATA_DT, load_cube_toss

default_isaac_contact_params = {"mu": 0.1,
                                "stiffness": 100.0,
                                "restitution": 0.0}


class IsaacCubeSim(CubeSim):

    def __init__(self, visualize=False, substeps=1):
        self.visualize = visualize
        self.sim_dt = CUBE_DATA_DT
        self.dt = CUBE_DATA_DT
        self.box_size = 1.0
        self.gym = gymapi.acquire_gym()
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
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        if self.visualize:
            self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
            if self.viewer is None:
                print("*** Failed to create viewer")
                quit()

        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_root = "examples/contact_parameter_learning"
        asset_file = "urdf/cube.urdf"
        asset = self.gym.load_asset(self.sim, asset_root,
                                    asset_file, asset_options)

        spacing = 1.0
        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, 2 * spacing)
        self.env = self.gym.create_env(self.sim, lower, upper, 1)

        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.5)
        pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)

        self.cube_handle = self.gym.create_actor(
            self.env, asset, pose, "cube", 0, 1)

        plane_pose = gymapi.Transform()
        plane_pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
        plane_pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        plane = self.gym.create_box(
            self.sim, self.box_size, self.box_size, 0.0, asset_options)
        self.plane_handle = self.gym.create_actor(
            self.env, plane, plane_pose, 'plane', 0, 0)

    def init_sim(self, params):
        shape_props = self.gym.get_actor_rigid_shape_properties(self.env, self.plane_handle)
        shape_props[0].friction = params['mu']
        shape_props[0].compliance = params['stiffness']  # compliance
        shape_props[0].restitution = params['restitution']  # damping equivalent
        self.gym.set_actor_rigid_shape_properties(self.env, self.plane_handle, shape_props)
        self.actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)
        self.root_states = gymtorch.wrap_tensor(self.actor_root_state)

        if self.visualize:
            cam_pos = gymapi.Vec3(2, 0, 2.0)
            cam_target = gymapi.Vec3(1, 0, -2.5)
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.root_states[self.plane_handle, :3] = to_torch([0., 0., 0.])
        self.root_states[self.plane_handle, 3:7] = to_torch([0., 0., 0., 1.])
        self.root_states[self.plane_handle, 7:13] = to_torch([0., 0., 0., 0., 0., 0.])

    def set_initial_condition(self, state):
        self.root_states[self.cube_handle, :3] = \
            to_torch(state[CUBE_DATA_POSITION_SLICE])