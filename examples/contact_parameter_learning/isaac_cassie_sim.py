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

from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *

from pydairlib.common import FindResourceOrThrow

from isaacgym import gymutil
from isaacgym import gymapi

from cassie_sim_data.cassie_traj import CASSIE_JOINT_POSITION_SLICE
from cassie_sim_data.cassie_traj import CASSIE_JOINT_VELOCITY_SLICE


class IsaacCassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 5e-4
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
                               "stiffness": 4e4,
                               "dissipation": 0.5}
        self.armatures = np.array(
            [0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225,
             0.038125, 0.038125, 0.09344, 0.09344, 0, 0, 0, 0.01225])

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
        asset_root = "../../assets"
        asset_file = "urdf/cassie_description/urdf/cassie_v2.urdf"
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

        if self.visualize:
            cam_pos = gymapi.Vec3(2, 0, 2.0)
            cam_target = gymapi.Vec3(1, 0, -2.5)
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.full_state = np.copy(self.gym.get_actors_rigid_body_states(self.env, self.actor_handle, gymapi.STATE_ALL))

        self.joint_states = self.gym.get_actor_dof_states(self.env, self.actor_handle, gymapi.STATE_ALL)

        self.current_time = 0.0
        self.reset(hardware_traj_num)

    def reset(self, hardware_traj_num):
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.traj = CassieSimTraj()

        qjointpos_init_drake = self.hardware_traj.get_initial_state()[CASSIE_JOINT_POSITION_SLICE]
        qjointvel_init_drake = self.hardware_traj.get_initial_state()[CASSIE_JOINT_VELOCITY_SLICE]

        # for i in range(cassie_num_dofs):
        self.joint_states['pos'] = qjointpos_init_drake
        self.joint_states['vel'] = qjointvel_init_drake
        self.gym.set_actor_dof_states(self.env, self.actor_handle, self.joint_states, gymapi.STATE_POS)

        self.sim.get_mutable_context().SetTime(self.start_time)
        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(),
                         self.hardware_traj.get_action(self.start_time))
        self.sim.Initialize()
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while not self.gym.query_viewer_has_closed(self.viewer):
            # step the physics
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)

            # update the viewer
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, True)

            # Wait for dt to elapse in real time.
            # This synchronizes the physics simulation with the rendering rate.
            self.gym.sync_frame_time(self.sim)

        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)
        while (self.current_time < time):
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.sim.get_context().get_time() + self.dt
        action = self.hardware_traj.get_action(self.sim.get_context().get_time())
        # self.plant.get_actuation_input_port().FixValue(self.plant_context, action)
        efforts = self.convert_action_to_full_efforts(action)
        self.gym.apply_actor_dof_efforts(self.env, self.actor_handle, efforts)
        # self.plant.get_actuation_input_port().FixValue(self.plant_context, np.zeros(10))
        self.sim.AdvanceTo(next_timestep)

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        cassie_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))
        self.current_time = next_timestep
        self.traj.update(next_timestep, cassie_state, action)
        return cassie_state

    def get_traj(self):
        return self.traj

    def free_sim(self):
        if self.visualize:
            self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)

    def convert_action_to_full_efforts(self, action):
        efforts_map = np.zeros((16, 10))
        efforts = efforts_map * action


### Set the joint armatures
cassie_dof_props = gym.get_actor_dof_properties(env, actor_handle)
cassie_num_dofs = len(cassie_dof_props)

import pdb;

pdb.set_trace()

### (WORK IN PROGRESS, figure out how to set the initial state including velocities)

# FULL STATE OF ALL THE RIGID BODIES
initial_state = np.copy(gym.get_sim_rigid_body_states(sim, gymapi.STATE_ALL))
# initial_state['pose'][0]['p']['z'] = 1.0 # HOW TO SET A SINGLE STATE


# STATE OF THE ROBOT DOFs
dof_states = gym.get_actor_dof_states(env, actor_handle, gymapi.STATE_ALL)

# pelvis_pos = gymapi.Vec3(0.0, 0.0, 1.0)
# initial_state['pose'][0]['p'] = gymapi.Vec3(0.0, 0.0, 1.0)
# gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)

# gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)
# initial_pos = np.copy(gym.get_sim_rigid_body_states(sim, gymapi.STATE_POS))

# gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)

qjointpos_init_drake = np.array([0.0045, 0, 0.4973, -1.1997, 0, 1.4267, 0, -1.5968,
                                 0.0045, 0, 0.4973, -1.1997, 0, 1.4267, 0, -1.5968])
qjointvel_init_drake = np.zeros(16)

for i in range(cassie_num_dofs):
    dof_states['pos'][i] = qjointpos_init_drake[i]
    dof_states['vel'][i] = qjointvel_init_drake[i]

gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_POS)

while not gym.query_viewer_has_closed(viewer):

    # Get input actions from the viewer and handle them appropriately
    for evt in gym.query_viewer_action_events(viewer):
        if evt.action == "reset" and evt.value > 0:
            gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
