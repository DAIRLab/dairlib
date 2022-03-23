import numpy as np

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
# from dairlib import lcmt_radio_out
from pydairlib.cassie.gym_envs.cassie_env_state import CassieEnvState, quat_to_rotation, \
    reexpress_state_local_to_global_omega, reexpress_state_global_to_local_omega
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter
# from drake_to_mujoco_converter import DrakeToMujocoConverter

from pydairlib.cassie.mujoco.cassiemujoco import *
from pydairlib.cassie.mujoco.mujoco_lcm_utils import *
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter


class MuJoCoCassieGym():
    def __init__(self, reward_func, visualize=False):
        self.sim_dt = 8e-5
        self.gym_dt = 1e-3
        self.visualize = visualize
        self.reward_func = reward_func
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 7.5

        self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
        self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'

        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
             -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.prev_cassie_state = None
        self.controller = None
        self.terminated = False
        self.initialized = False
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

    def make(self, controller, model_xml='cassie.xml'):
        self.builder = DiagramBuilder()
        self.dt = 8e-5
        self.plant = MultibodyPlant(self.dt)
        self.controller = controller
        self.simulator = CassieSim(self.default_model_directory + model_xml)
        if self.visualize:
            self.cassie_vis = CassieVis(self.simulator)
        # self.simulator = CassieSimDiagram(self.plant, urdf, self.visualize, 0.8, 1e4, 1e2)
        # self.new_plant = self.simulator.get_plant()
        # self.sensor_aggregator = self.simulator.get_sensor_aggregator()
        self.builder.AddSystem(self.controller)
        # self.builder.AddSystem(self.simulator)

        # self.builder.Connect(self.controller.get_control_output_port(), self.simulator.get_actuation_input_port())
        # self.builder.Connect(self.simulator.get_state_output_port(), self.controller.get_state_input_port())
        # self.builder.Connect(self.simulator.get_cassie_out_output_port_index(),
        #                      self.controller.get_cassie_out_input_port())
        # self.builder.Connect(self.controller, self.simulator.get_radio_input_port())
        self.drake_to_mujoco_converter = DrakeToMujocoConverter(self.sim_dt)

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        # self.simulator_context = self.diagram.GetMutableSubsystemContext(self.simulator, self.sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(self.controller,
                                                                          self.sim.get_mutable_context())
        self.controller_state_input_port = self.controller.get_state_input_port()
        self.controller_output_port = self.controller.get_torque_output_port()
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        # self.traj = CassieTraj()
        q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(
            reexpress_state_global_to_local_omega(self.x_init))
        mujoco_state = self.simulator.get_state()
        mujoco_state.set_qpos(q_mujoco)
        mujoco_state.set_qvel(v_mujoco)
        mujoco_state.set_time(self.start_time)
        self.simulator.set_state(mujoco_state)
        self.sim.get_mutable_context().SetTime(self.start_time)
        u = np.zeros(10)
        self.sim.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        self.cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        self.terminated = False
        return

    def advance_to(self, time):
        while self.current_time < time and not self.terminated:
            self.step()
        return

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4

    def step(self, action=np.zeros(18)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")

        next_timestep = self.sim.get_context().get_time() + self.gym_dt
        import pdb; pdb.set_trace()
        self.controller_state_input_port.FixValue(self.controller_context, self.cassie_state.x)
        u = self.controller_output_port.Eval(self.controller_context)[:-1]  # remove the timestamp
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, u)

        self.sim.AdvanceTo(np.around(next_timestep, decimals=3))
        while self.simulator.time() < next_timestep:
            self.simulator.step(cassie_in)
        if self.visualize:
            self.cassie_vis.draw(self.simulator)

        self.current_time = next_timestep
        t = self.simulator.time()
        q = self.simulator.qpos()
        v = self.simulator.qvel()
        q, v = self.drake_to_mujoco_converter.convert_to_drake(q, v)
        self.current_time = t
        x = np.hstack((q, v))
        x = reexpress_state_local_to_global_omega(x)
        self.cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(self.cassie_state, self.prev_cassie_state)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state, reward

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
