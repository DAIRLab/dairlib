import numpy as np
from cassie_traj import *

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
from pydairlib.cassie.gym_envs.cassie_env_state import CassieEnvState


class CassieGym():
    def __init__(self, reward_func, visualize=False):
        self.sim_dt = 1e-3
        self.visualize = visualize
        self.reward_func = reward_func
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 0.05
        self.traj = CassieTraj()
        self.hardware_traj = None
        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
             -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.default_contact_params = {"mu": 0.8,
                                       "stiffness": 4e4,
                                       "dissipation": 0.5}
        self.prev_cassie_state = None
        self.controller = None

    def make(self, controller, urdf):
        self.builder = DiagramBuilder()
        self.dt = 8e-5
        self.plant = MultibodyPlant(self.dt)
        self.controller = controller
        self.simulator = CassieSimDiagram(self.plant, urdf, 0.8, 1e4, 1e2)
        self.new_plant = self.simulator.get_plant()
        # self.sensor_aggregator = self.simulator.get_sensor_aggregator()
        self.builder.AddSystem(self.controller)
        self.builder.AddSystem(self.simulator)

        self.builder.Connect(self.controller.get_control_output_port(), self.simulator.get_actuation_input_port())
        self.builder.Connect(self.simulator.get_state_output_port(), self.controller.get_state_input_port())
        self.builder.Connect(self.simulator.get_cassie_out_output_port_index(),
                             self.controller.get_cassie_out_input_port())
        # self.builder.Connect(self.controller, self.simulator.get_radio_input_port())

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.simulator_context = self.diagram.GetMutableSubsystemContext(self.simulator, self.sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(self.controller, self.sim.get_mutable_context())
        self.controller_output_port = self.controller.get_torque_output_port()
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()

    def reset(self):
        self.traj = CassieTraj()
        self.new_plant.SetPositionsAndVelocities(self.new_plant.GetMyMutableContextFromRoot(
            self.sim.get_mutable_context()), self.x_init)
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.traj.update(self.start_time, self.x_init,
                         np.zeros(self.action_dim))
        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        u = np.zeros(10)
        self.sim.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, x, u, np.zeros(18))
        return

    def advance_to(self, time):
        while self.current_time < time:
            self.step()
        return self.traj

    def step(self, action=np.zeros(18)):
        # next_timestep = self.sim.get_context().get_time() + self.dt
        next_timestep = self.sim.get_context().get_time() + self.sim_dt
        self.simulator.get_radio_input_port().FixValue(self.simulator_context, action)
        self.sim.AdvanceTo(np.around(next_timestep, decimals=3))
        self.current_time = next_timestep

        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        # print(next_timestep)
        u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
        # u = self.controller_output_port.Eval(self.sim.get_context())[:-1] # remove the timestamp
        # print(u)
        # u = np.zeros(10)
        cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(cassie_state, self.prev_cassie_state)
        # reward = 0
        self.prev_cassie_state = cassie_state
        # self.traj.update(next_timestep, cassie_state, action[:10])
        return cassie_state, reward

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
