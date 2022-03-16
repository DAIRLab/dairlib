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


class CassieGym():
    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 0.05
        self.traj = CassieTraj()
        self.hardware_traj = None
        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array([1, 0, 0, 0, 0, 0, 1,
                                -0.0304885, 0, 0.466767, -1.15602, -0.037542, 1.45243, -0.0257992, -1.59913,
                                0.0304885, 0, 0.466767, -1.15602, -0.0374859, 1.45244, -0.0259075, -1.59919,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.default_contact_params = {"mu": 0.8,
                                       "stiffness": 4e4,
                                       "dissipation": 0.5}
        self.controller = None

    def make(self, controller, urdf):
        self.builder = DiagramBuilder()
        self.dt = 5e-4
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
        self.plant_context = self.diagram.GetMutableSubsystemContext(self.new_plant, self.sim.get_mutable_context())
        self.simulator_context = self.diagram.GetMutableSubsystemContext(self.simulator, self.sim.get_mutable_context())
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()

    def reset(self):
        self.traj = CassieTraj()
        self.new_plant.SetPositionsAndVelocities(self.new_plant.GetMyMutableContextFromRoot(
            self.sim.get_mutable_context()), self.x_init)
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.traj.update(self.start_time, self.x_init,
                         np.zeros(self.action_dim))
        self.sim.Initialize()
        cassie_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))
        import pdb;
        pdb.set_trace()
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while self.current_time < time:
            self.step()
        return self.traj

    def get_state(self):
        return self.traj.get_positions()[-1]

    def step(self, action=np.ones(18)):
        next_timestep = self.sim.get_context().get_time() + self.dt
        # action = lcmt_radio_out
        # self.simulator.get_radio_input_port().FixValue(self.simulator_context, AbstractValue.Make(action))

        self.simulator.get_radio_input_port().FixValue(self.simulator_context, action)
        self.sim.AdvanceTo(next_timestep)

        cassie_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))
        self.current_time = next_timestep
        self.traj.update(next_timestep, cassie_state, action)
        return cassie_state

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
