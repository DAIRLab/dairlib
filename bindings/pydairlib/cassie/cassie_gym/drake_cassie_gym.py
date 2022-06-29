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
from cassie_env_state import CassieEnvState


class DrakeCassieGym():
    def __init__(self, visualize=False):
        self.sim_dt = 1e-3
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
        self.prev_cassie_state = None
        self.controller = None
        self.terminated = False
        self.initialized = False

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.builder = DiagramBuilder()
        self.dt = 8e-5
        self.plant = MultibodyPlant(self.dt)
        self.controller = controller
        self.simulator = \
            CassieVisionSimDiagram(self.plant, urdf, self.visualize,
                                   0.8, np.array([0.1, -0.1, 1]))
        self.sim_plant = self.simulator.get_plant()
        self.builder.AddSystem(self.controller)
        self.builder.AddSystem(self.simulator)

        if (self.visualize):
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

            self.builder.Connect(self.simulator.get_camera_out_output_port(),
                                 self.image_array_sender.get_input_port())
            self.builder.Connect(self.image_array_sender.get_output_port(),
                                 self.image_array_publisher.get_input_port())

        self.builder.Connect(self.controller.get_control_output_port(),
                             self.simulator.get_actuation_input_port())
        self.builder.Connect(self.simulator.get_state_output_port(),
                             self.controller.get_state_input_port())

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.simulator_context = self.diagram.GetMutableSubsystemContext(
            self.simulator, self.sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(
            self.controller, self.sim.get_mutable_context())
        self.controller_output_port = self.controller.get_torque_output_port()
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        self.sim_plant.SetPositionsAndVelocities(
            self.sim_plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()), self.x_init)
        self.sim.get_mutable_context().SetTime(self.start_time)
        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        u = np.zeros(10)
        self.sim.Initialize()
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
        next_timestep = self.sim.get_context().get_time() + self.sim_dt
        self.simulator.get_radio_input_port().FixValue(self.simulator_context, radio)
        self.controller.get_radio_input_port().FixValue(self.controller_context, radio)
        self.sim.AdvanceTo(next_timestep)
        self.current_time = self.sim.get_context().get_time()

        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
        self.cassie_state = CassieEnvState(self.current_time, x, u)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
