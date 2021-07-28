from cube_sim import *
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import DrakeVisualizer


class DrakeCubeSim(CubeSim):

    def __init__(self, drake_sim_dt=1e-5, visualize=False):
        if (not type(visualize) == bool) : 
            raise TypeError('visualize argument must be set to a boolean value')
        self.drake_sim_dt = drake_sim_dt
        self.visualize=visualize

    def init_sim(self, params):
        builder = DiagramBuilder()
        self.plant, self.scene_graph = \
            AddMultibodyPlantSceneGraph(builder, self.drake_sim_dt)
        Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_model_verification/urdf/cube.urdf"))
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.sim = Simulator(diagram)
    
