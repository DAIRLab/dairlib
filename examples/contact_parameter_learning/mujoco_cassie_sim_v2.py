import numpy as np
from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *

from cassiemujoco import *
from mujoco_lcm_utils import *
from drake_to_mujoco_converter import DrakeToMujocoConverter

class MuJoCoCassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.end_time = 0.05
        self.sample_period = 2e-3
        self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
        # self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
        self.traj = CassieSimTraj()
        self.valid_ground_truth_trajs = np.arange(0, 24)
        self.hardware_traj = None
        self.drake_to_mujoco_converter = DrakeToMujocoConverter(self.sim_dt)
        self.cassie_in = cassie_user_in_t()
        self.default_params = {"mu": 0.8,
                               "stiffness": 4e4,
                               "dissipation": 0.5}

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

    def make(self, params, hardware_traj_num, xml='/home/yangwill/workspace/cassie-mujoco-sim/model/cassie_new_params.xml'):
        self.cassie_env = CassieSim(xml)
        self.dt = 5e-4
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.reset()


    def reset(self):
        self.traj = CassieSimTraj()
        q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(self.hardware_traj.get_initial_state())
        mujoco_state = self.cassie_env.get_state()
        mujoco_state.set_qpos(q_mujoco)
        mujoco_state.set_qvel(v_mujoco)
        self.cassie_env.set_state(mujoco_state)

        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(), self.hardware_traj.get_action(self.start_time))
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
        print("outer step")
        while self.cassie_env.time() < next_timestep:
            print("inner step")
            self.cassie_env.step(cassie_in)
        t = self.cassie_env.time()
        q = self.cassie_env.qpos()
        v = self.cassie_env.qvel()
        q, v = self.drake_to_mujoco_converter.convert_to_drake(q, v)
        self.current_time = t
        cassie_state = np.hstack((q, v))
        self.traj.update(t, np.hstack((q, v)), action)
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
