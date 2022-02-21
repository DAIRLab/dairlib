import numpy as np
import xml.etree.ElementTree as ET

from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *
# from cassie_sim_data.cassie_traj import reexpress_state_local_to_global_omega, reexpress_state_global_to_local_omega

from cassiemujoco import *
from mujoco_lcm_utils import *
from drake_to_mujoco_converter import DrakeToMujocoConverter


class MuJoCoCassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.dt = 5e-4
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.end_time = 0.05
        self.sample_period = 2e-3

        # Will read in the default mujoco model and write a new model with the updated contact parameters
        self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
        self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
        self.tree = ET.parse(self.default_model_file)

        self.traj = CassieSimTraj()
        self.valid_ground_truth_trajs = np.arange(0, 29)
        self.hardware_traj = None
        self.drake_to_mujoco_converter = DrakeToMujocoConverter(self.sim_dt)
        self.cassie_in = cassie_user_in_t()
        self.default_params = {"stiffness": 2000,
                               "damping": 36.02,
                               "mu_tangent": 0.18}

        self.actuator_index_map = {'hip_roll_left_motor': 0,
                                   'hip_yaw_left_motor': 1,
                                   'hip_pitch_left_motor': 2,
                                   'knee_left_motor': 3,
                                   'toe_left_motor': 4,
                                   'hip_roll_right_motor': 5,
                                   'hip_yaw_right_motor': 6,
                                   'hip_pitch_right_motor': 7,
                                   'knee_right_motor': 8,
                                   'toe_right_motor': 9}

    def make(self, params, hardware_traj_num, xml='cassie_new_params.xml'):

        stiffness = params['stiffness']
        damping = params['damping']
        mu_tangent = params['mu_tangent']
        self.tree.getroot().find('default').find('geom').set('solref', '%.5f %.5f' % (-stiffness, -damping))
        self.tree.getroot().find('default').find('geom').set('friction',
                                                             '%.5f %.5f %.5f' % (mu_tangent, .001, .001))
        self.tree.write(self.default_model_directory + xml)
        self.cassie_env = CassieSim(self.default_model_directory + xml)
        if self.visualize:
            self.cassie_vis = CassieVis(self.cassie_env)
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.reset(hardware_traj_num)

    def reset(self, hardware_traj_num):
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.traj = CassieSimTraj()
        x_init = self.hardware_traj.get_initial_state()

        q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(
            reexpress_state_global_to_local_omega(x_init))
        mujoco_state = self.cassie_env.get_state()
        mujoco_state.set_qpos(q_mujoco)
        mujoco_state.set_qvel(v_mujoco)
        mujoco_state.set_time(self.start_time)
        self.cassie_env.set_state(mujoco_state)

        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(),
                         self.hardware_traj.get_action(self.start_time))
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while (self.current_time < time):
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.cassie_env.time() + self.dt
        action = self.hardware_traj.get_action(next_timestep)
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, action)
        while self.cassie_env.time() < next_timestep:
            self.cassie_env.step(cassie_in)
        if self.visualize:
            self.cassie_vis.draw(self.cassie_env)
        # get current state
        t = self.cassie_env.time()
        q = self.cassie_env.qpos()
        v = self.cassie_env.qvel()
        q, v = self.drake_to_mujoco_converter.convert_to_drake(q, v)
        self.current_time = t
        cassie_state = np.hstack((q, v))
        cassie_state = reexpress_state_local_to_global_omega(cassie_state)
        self.traj.update(t, cassie_state, action)
        # print(cassie_state)
        return cassie_state

    def get_traj(self):
        return self.traj

    def pack_input(self, cassie_in, u_drake):
        act_map = self.drake_to_mujoco_converter.act_map
        # Set control parameters
        u_mujoco = np.zeros(10)
        for u_name in act_map:
            cassie_in.torque[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
            u_mujoco[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
        return cassie_in, u_mujoco

    def free_sim(self):
        del self.cassie_env
