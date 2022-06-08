import numpy as np
import gym   # OpenAI gym

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
from pydairlib.systems.robot_lcm_systems import RobotOutputSender
from dairlib import lcmt_radio_out
from pydairlib.cassie.simulators import CassieSimDiagram
from cassie_env_state import CassieEnvState, CASSIE_NU


class DrakeCassieGym(gym.Env):
    def __init__(self, reward_func, visualize=False):
        self.visualize = visualize
        self.reward_func = reward_func
        self.start_time = 0.00
        self.current_time = 0.00
        self.sim_dt = 1e-2
        self.plant_dt = 8e-5
        self.default_action = np.zeros(18)
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742,
             1.90918, -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588,
             -0.0457885, 1.90919, -0.0382424, -1.82321, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.controller = None
        self.terminated = False
        self.initialized = False
        self.cassie_state = None
        self.prev_cassie_state = None
        self.cumulative_reward = 0
        self.traj = None

        # Simulation objects
        self.builder = None
        self.plant = None
        self.cassie_sim = None
        self.sim_plant = None
        self.diagram = None
        self.drake_simulator = None
        self.cassie_sim_context = None
        self.controller_context = None
        self.controller_output_port = None

        self.observation_space = \
            gym.spaces.Box(high=1000, low=-1000, shape=(45,), dtype=float)
        self.action_space = \
            gym.spaces.Box(high=1, low=-1, shape=(18,), dtype=float)

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.builder = DiagramBuilder()
        # Add Controller
        self.controller = controller
        self.builder.AddSystem(self.controller)

        # Add Cassie Simulation
        self.plant = MultibodyPlant(self.plant_dt)
        self.cassie_sim = CassieSimDiagram(self.plant, urdf, self.visualize, 0.8)
        self.sim_plant = self.cassie_sim.get_plant()
        self.builder.AddSystem(self.cassie_sim)

        # Connect ports
        self.builder.Connect(self.controller.get_control_output_port(),
                             self.cassie_sim.get_actuation_input_port())
        self.builder.Connect(self.cassie_sim.get_state_output_port(),
                             self.controller.get_state_input_port())

        # Build Drake block diagram
        self.diagram = self.builder.Build()
        self.drake_simulator = Simulator(self.diagram)

        # grab relevant contexts
        self.cassie_sim_context = self.diagram.GetMutableSubsystemContext(
            self.cassie_sim, self.drake_simulator.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(
            self.controller, self.drake_simulator.get_mutable_context())
        self.controller_output_port = self.controller.get_torque_output_port()
        self.drake_simulator.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        # Reset plant state
        self.sim_plant.SetPositionsAndVelocities(
            self.sim_plant.GetMyMutableContextFromRoot(
                self.drake_simulator.get_mutable_context()), self.x_init)
        # Rewind simulator to 0
        self.drake_simulator.get_mutable_context().SetTime(self.start_time)
        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.drake_simulator.get_context()))
        u = np.zeros(CASSIE_NU)
        self.drake_simulator.Initialize()
        self.current_time = self.start_time
        self.cassie_state = \
            CassieEnvState(self.current_time, x, u, self.default_action)
        self.prev_cassie_state = \
            CassieEnvState(self.current_time, x, u, self.default_action)
        self.cumulative_reward = 0
        self.terminated = False
        return np.array(self.cassie_state.x)

    def advance_to(self, time):
        while self.current_time < time and not self.terminated:
            x, r, t, _ = self.step()
            # print(x.shape)
        return

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4

    def step(self, action=None):
        if not self.initialized:
            print("Call make() before calling step() or advance()")
        if action is None:
            action = self.default_action
            
        next_timestep = self.drake_simulator.get_context().get_time() + self.sim_dt
        self.cassie_sim.get_radio_input_port().FixValue(self.cassie_sim_context, action)
        self.controller.get_radio_input_port().FixValue(self.controller_context, action)
        self.drake_simulator.AdvanceTo(next_timestep)
        self.current_time = self.drake_simulator.get_context().get_time()

        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.drake_simulator.get_context()))
        u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
        self.cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(self.sim_dt, self.cassie_state, self.prev_cassie_state)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        self.cumulative_reward += reward
        return np.array(self.cassie_state.x), reward, bool(self.terminated), {}

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
