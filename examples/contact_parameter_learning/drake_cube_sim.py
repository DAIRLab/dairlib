from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, CUBE_DATA_DT
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer, ConnectMeshcatVisualizer
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.math import RigidTransform
from pydrake.geometry import DrakeVisualizer, HalfSpace, Box
from pydairlib.common import FindResourceOrThrow
from pydairlib.multibody import makeNameToPositionsMap


# Note that the mu_ratio defines the ratio 
# mu_kinetic/mu_static = [0 ... 1] to enforce that mu_kinetic <= mu_static
default_drake_contact_params = {
    "mu_static": 0.8,
    "mu_ratio": 1.0,
    "pen_allow": 1e-5, 
    "stiction_tol": 1e-3 }

class DrakeCubeSim(CubeSim):

    def __init__(self, drake_sim_dt=1e-4, visualize=False):
        if (not type(visualize) == bool) : 
            raise TypeError('visualize argument must be set to a boolean value')
        self.drake_sim_dt = drake_sim_dt
        self.visualize=visualize

    def init_sim(self, params):
        ''' Here we build a diagram for the drake simulation'''
        self.builder = DiagramBuilder()
        
        # Add a cube as MultibodyPlant
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)

        terrain_normal=np.array([0.0, 0.0, 1.0])
        terrain_point=np.zeros((3,))
        terrain_color=np.array([0.8, 0.8, 0.8, 1.0])
        friction=CoulombFriction(params["mu_static"], params["mu_ratio"]*params["mu_static"])
        X_WG = RigidTransform(HalfSpace.MakePose(terrain_normal, terrain_point))

        Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_parameter_learning/urdf/cube.urdf"))

        self.plant.RegisterCollisionGeometry(self.plant.world_body(), X_WG, HalfSpace(), "collision", friction)
        self.plant.RegisterVisualGeometry(self.plant.world_body(), X_WG, Box(1, 1, 0.001), "visual", terrain_color)
        

        self.plant.Finalize()
        self.plant.set_penetration_allowance(params["pen_allow"])
        self.plant.set_stiction_tolerance(params["stiction_tol"])

        # Add visualization if visualizing the simlation
        if (self.visualize):
            DrakeVisualizer.AddToBuilder(self.builder, self.scene_graph)
            self.meshcat_vis = ConnectMeshcatVisualizer(self.builder, self.scene_graph, zmq_url="new", open_browser=True)

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        
        self.sim = Simulator(self.diagram)
        self.sim.Initialize()

    def sim_step(self, dt):
        data_arr = np.zeros((1,13))

        cube_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))
        
        data_arr[0,CUBE_DATA_QUATERNION_SLICE] = cube_state[0:4]
        data_arr[0,CUBE_DATA_POSITION_SLICE] = cube_state[4:7]
        data_arr[0,CUBE_DATA_OMEGA_SLICE] = cube_state[7:10]
        data_arr[0,CUBE_DATA_VELOCITY_SLICE] = cube_state[10:]
        
        new_time = self.sim.get_mutable_context().get_time() + dt
        self.sim.AdvanceTo(new_time)

        if (self.visualize):
            self.meshcat_vis.vis.render_static()

        return data_arr

    def set_initial_condition(self, initial_state):
        q = np.zeros((self.plant.num_positions(),))
        v = np.zeros((self.plant.num_velocities(),))

        q[0:4] = initial_state[CUBE_DATA_QUATERNION_SLICE]
        q[4:] = initial_state[CUBE_DATA_POSITION_SLICE]
        v[0:3] = initial_state[CUBE_DATA_OMEGA_SLICE]
        v[3:] = initial_state[CUBE_DATA_VELOCITY_SLICE]

        self.sim.get_mutable_context().SetTime(0.0)
        self.sim.Initialize()

        self.plant.SetPositions(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()), q)
        self.plant.SetVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()), v)
