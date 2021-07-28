from cube_sim import *
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import DrakeVisualizer
from pydairlib.multibody import makeNameToPositionsMap, makeNameToVelocitiesMap, addFlatTerrain
from pydairlib.common import FindResourceOrThrow

default_drake_contact_params = {
    "mu_static": 0.8,
    "mu_kinetic": 0.8,
    "pen_allow": 1e-5, 
    "stiction_tol": 1e-3 }

class DrakeCubeSim(CubeSim):

    def __init__(self, drake_sim_dt=1e-5, visualize=False):
        if (not type(visualize) == bool) : 
            raise TypeError('visualize argument must be set to a boolean value')
        self.drake_sim_dt = drake_sim_dt
        self.visualize=visualize

    def init_sim(self, params):
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = \
            AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_model_verification/urdf/cube.urdf"))
        addFlatTerrain(
            self.plant, self.scene_graph, params["mu_static"], params["mu_kinetic"])
        self.plant.Finalize()
        self.plant.set_penetration_allowance(params["pen_allow"])
        self.plant.set_stiction_tolerance(params["stiction_tol"])
        self.pos_map = makeNameToPositionsMap(self.plant)
        self.vel_map = makeNameToVelocitiesMap(self.plant)
        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.diagram_context)
        self.sim = Simulator(self.diagram)
        self.sim.Initialize()

    def sim_step(self):
        data_arr = np.zeros((1,13))
        cube_state = self.plant.GetPositionsAndVelocities(self.plant_context)
        data_arr

    def set_initial_condition(self, intial_state):
        q = np.zeros((self.plant.num_positions()))
        v = np.zeros((self.plant.num_velocities()))
        self.sim.get_mutable_context().SetTime(.0)
        self.plant.Set