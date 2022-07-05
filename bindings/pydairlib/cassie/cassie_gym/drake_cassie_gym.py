import numpy as np

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator
from pydrake.systems.sensors import ImageToLcmImageArrayT, PixelType
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.lcm import DrakeLcm
from drake import lcmt_image_array

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
from pydairlib.systems.robot_lcm_systems import RobotOutputSender
from dairlib import lcmt_radio_out
from pydairlib.cassie.simulators import CassieVisionSimDiagram
from pydairlib.cassie.cassie_gym.cassie_env_state import CassieEnvState


class DrakeCassieGym():
    def __init__(self, visualize=False):
        self.sim_dt = 1e-2
        self.visualize = visualize
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 0.05
        self.hardware_traj = None
        self.action_dim = 10
        self.state_dim = 45
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
        self.plant_context = None
        self.controller_output_port = None

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.builder = DiagramBuilder()
        self.plant = MultibodyPlant(8e-5)
        self.controller = controller
        self.cassie_sim = CassieVisionSimDiagram(
            plant=self.plant,
            urdf=urdf,
            visualize=self.visualize,
            mu=0.8,
            normal=np.array([0.1, -0.1, 1]))
        self.sim_plant = self.cassie_sim.get_plant()
        self.builder.AddSystem(self.controller)
        self.builder.AddSystem(self.cassie_sim)

        if self.visualize:
            self.lcm = DrakeLcm()
            self.image_array_sender = self.builder.AddSystem(
                ImageToLcmImageArrayT())
            self.image_array_sender.DeclareImageInputPort[PixelType.kDepth32F]("depth")
            self.image_array_publisher = self.builder.AddSystem(
                LcmPublisherSystem.Make(
                    "DRAKE_RGBD_CAMERA_IMAGES",
                    lcm_type=lcmt_image_array,
                    lcm=self.lcm,
                    publish_period=0.1,
                    use_cpp_serializer=True))

            self.builder.Connect(self.cassie_sim.get_camera_out_output_port(),
                                 self.image_array_sender.get_input_port())
            self.builder.Connect(self.image_array_sender.get_output_port(),
                                 self.image_array_publisher.get_input_port())

        self.builder.Connect(self.controller.get_control_output_port(),
                             self.cassie_sim.get_actuation_input_port())
        self.builder.Connect(self.cassie_sim.get_state_output_port(),
                             self.controller.get_state_input_port())

        self.diagram = self.builder.Build()
        self.drake_simulator = Simulator(self.diagram)
        self.cassie_sim_context = \
            self.diagram.GetMutableSubsystemContext(
                self.cassie_sim,
                self.drake_simulator.get_mutable_context())
        self.controller_context = \
            self.diagram.GetMutableSubsystemContext(
                self.controller,
                self.drake_simulator.get_mutable_context())
        self.plant_context = \
            self.diagram.GetMutableSubsystemContext(
                self.sim_plant,
                self.drake_simulator.get_mutable_context())
        self.controller_output_port = self.controller.get_torque_output_port()
        self.drake_simulator.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        self.sim_plant.SetPositionsAndVelocities(
            self.plant_context, self.x_init)
        self.drake_simulator.get_mutable_context().SetTime(self.start_time)
        x = self.sim_plant.GetPositionsAndVelocities(self.plant_context)
        u = np.zeros(10)
        self.drake_simulator.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, x, u)
        self.cassie_state = CassieEnvState(self.current_time, x, u)
        self.terminated = False
        return

    def advance_to(self, time):
        while self.current_time < time and not self.terminated:
            self.step()
        return

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4

    def step(self, radio=np.zeros(18)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")

        # Calculate next timestep
        next_timestep = self.drake_simulator.get_context().get_time() + self.sim_dt

        # Set simulator inputs and advance simulator
        self.cassie_sim.get_radio_input_port().FixValue(
            context=self.cassie_sim_context,
            value=radio)
        self.controller.get_radio_input_port().FixValue(
            context=self.controller_context,
            value=radio)
        self.drake_simulator.AdvanceTo(next_timestep)
        self.current_time = self.drake_simulator.get_context().get_time()

        # Get the state
        x = self.sim_plant.GetPositionsAndVelocities(self.plant_context)
        u = self.controller_output_port.Eval(
            self.controller_context)[:-1]  # remove the timestamp
        self.cassie_state = CassieEnvState(self.current_time, x, u)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
