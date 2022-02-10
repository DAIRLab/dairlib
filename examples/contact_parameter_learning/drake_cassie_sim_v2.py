import numpy as np
from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *
# import CassieSim

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *


class DrakeCassieSim():

    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.visualize = visualize
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.end_time = 0.05
        self.sample_period = 2e-3
        self.traj = CassieSimTraj()
        self.valid_ground_truth_trajs = np.arange(0, 24)
        self.hardware_traj = None

        self.default_params = {"mu": 0.8,
                               "stiffness": 4e4,
                               "dissipation": 0.5}

    def make(self, params, hardware_traj_num, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.builder = DiagramBuilder()
        self.dt = 5e-4
        self.params = params

        terrain_normal = np.array([0.0, 0.0, 1.0])
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.sim_dt)
        AddCassieMultibody(self.plant, self.scene_graph, True, urdf, True, True)
        AddFlatTerrain(self.plant, self.scene_graph, params['mu'], params['mu'], terrain_normal, params['stiffness'],
                       params['dissipation'])

        self.plant.Finalize()
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        controller_inputs = self.builder.AddSystem(
            TrajectoryPlayback(self.hardware_traj.generate_input_traj(), self.plant.num_actuators(), 0.0))
        passthrough = self.builder.AddSystem(SubvectorPassThrough(11, 0, 10))
        self.builder.Connect(controller_inputs.get_command_output_port(), passthrough.get_input_port())
        self.builder.Connect(passthrough.get_output_port(), self.plant.get_actuation_input_port())
        # self.input_port = self.plant.get_actuation_input_port()
        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        # self.diagram_context.EnableCaching()
        # self.diagram.SetDefaultContext(self.diagram_context)
        # self.plant_context = self.diagram.GetMutableSubsystemContext(self.plant, self.diagram_context)
        self.diagram_context.SetTime(self.start_time)
        self.sim = Simulator(self.diagram)

        self.reset()
        # self.sim.Initialize()
        # self.current_time = 0.0

    def reset(self):
        self.traj = CassieSimTraj()
        self.plant.SetPositionsAndVelocities(self.plant.GetMyMutableContextFromRoot(
            self.sim.get_mutable_context()), self.hardware_traj.get_initial_state())
        self.diagram_context.SetTime(self.start_time)
        self.traj.update(self.start_time, self.hardware_traj.get_initial_state(), self.hardware_traj.get_action(self.start_time))
        self.sim.Initialize()
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while (self.current_time < time):
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.sim.get_mutable_context().get_time() + self.dt
        action = self.hardware_traj.get_action(next_timestep)
        # self.plant.get_actuation_input_port().FixValue(self.plant_context, action)
        self.sim.AdvanceTo(next_timestep)

        cassie_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))
        self.current_time = next_timestep
        self.traj.update(next_timestep, cassie_state, action)
        return cassie_state

    def get_traj(self):
        return self.traj
