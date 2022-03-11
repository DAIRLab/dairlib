import numpy as np
from cassie_actor import *

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *


class CassieRunningController(CassieActor):
    def __init__(self, visualize=False):
        self.sim_dt = 5e-5
        self.action_dim = 10
        self.state_dim = 45

    def make(self, params):
        self.builder = DiagramBuilder()
        self.dt = 5e-4
        self.params = params

        terrain_normal = np.array([0.0, 0.0, 1.0])
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.sim_dt)
        AddCassieMultibody(self.plant, self.scene_graph, True, urdf, True, True)
        AddFlatTerrain(self.plant, self.scene_graph, params['mu'], params['mu'], terrain_normal, params['stiffness'],
                       params['dissipation'])

        self.plant.Finalize()

        self.builder.Connect(self.controller_inputs.get_command_output_port(), self.passthrough.get_input_port())
        self.builder.Connect(self.passthrough.get_output_port(), self.plant.get_actuation_input_port())
        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)

        self.plant_context = self.diagram.GetMutableSubsystemContext(self.plant, self.sim.get_mutable_context())
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()

    def reset(self):
        self.traj = CassieTraj()
        self.plant.SetPositionsAndVelocities(self.plant.GetMyMutableContextFromRoot(
            self.sim.get_mutable_context()), self.x_init)
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.traj.update(self.start_time, self.x_init,
                         np.zeros(self.action_dim))
        self.sim.Initialize()
        self.current_time = self.start_time
        return

    def advance_to(self, time):
        while self.current_time < time:
            self.step()
        return self.traj

    def get_state(self):
        return self.traj.get_positions()[-1]

    def step(self, action=np.zeros(10)):
        next_timestep = self.sim.get_context().get_time() + self.dt
        if self.controller:
            action = self.controller.eval(self.get_state())
        self.plant.get_actuation_input_port().FixValue(self.plant_context, action)
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
