from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, CUBE_DATA_DT, load_cube_toss
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer, ConnectMeshcatVisualizer
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.systems.primitives import TrajectorySource
from pydrake.math import RigidTransform
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box, ProximityProperties
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import FindResourceOrThrow
# from ctypes import c_double

# Note that the mu_ratio defines the ratio 
# mu_kinetic/mu_static = [0 ... 1] to enforce that mu_kinetic <= mu_static
default_drake_contact_params = {
    "mu": 0.18,
    "stiffness": 1.0e4, 
    "stiction_tol": 1e-3, 
    "dissipation":0.5 }

class DrakeCubeSim(CubeSim):

    def __init__(self, visualize=False, substeps=1):
        if (not type(visualize) == bool) : 
            raise TypeError('visualize argument must be set to a boolean value')
        self.drake_sim_dt = CUBE_DATA_DT / substeps
        self.visualize=visualize

    def init_sim(self, params):
        ''' Here we build a diagram for the drake simulation'''
        self.builder = DiagramBuilder()
        
        # Add a cube as MultibodyPlant
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)       
        self.add_plant_and_terrain(params)

        # Add visualization if visualizing the simlation
        if (self.visualize):
            DrakeVisualizer.AddToBuilder(self.builder, self.scene_graph)

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        
        self.sim = Simulator(self.diagram)

        if (self.visualize):
            self.sim.set_target_realtime_rate(1.0)

        self.sim.Initialize()

    def add_plant_and_terrain(self, params):
        terrain_normal=np.array([0.0, 0.0, 1.0])
        terrain_point=np.zeros((3,))
        terrain_color=np.array([0.8, 0.8, 0.8, 1.0])
        friction = CoulombFriction(params['mu'], params['mu'])
        props = ProximityProperties()
        props.AddProperty("material", "point_contact_stiffness", params['stiffness'])
        props.AddProperty("material", "hunt_crossley_dissipation", params['dissipation'])
        props.AddProperty("material", "coulomb_friction", friction)

        X_WG = RigidTransform(HalfSpace.MakePose(terrain_normal, terrain_point))

        Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_parameter_learning/urdf/cube.urdf"))

        self.plant.RegisterCollisionGeometry(self.plant.world_body(), X_WG, HalfSpace(), "collision", props)
        self.plant.RegisterVisualGeometry(self.plant.world_body(), X_WG, Box(1, 1, 0.001), "visual", terrain_color)
        self.plant.Finalize()
        self.plant.set_stiction_tolerance(params["stiction_tol"])

    def init_playback_sim(self, traj_to_play_back):
        self.builder = DiagramBuilder()
        
        # Add a cube as MultibodyPlant
        self.plant = MultibodyPlant(self.drake_sim_dt)
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        plant_id = self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)

        terrain_normal=np.array([0.0, 0.0, 1.0])
        terrain_point=np.zeros((3,))
        terrain_color=np.array([0.8, 0.8, 0.8, 1.0])
        X_WG = RigidTransform(HalfSpace.MakePose(terrain_normal, terrain_point))
        Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_parameter_learning/urdf/cube.urdf"))
        self.plant.RegisterVisualGeometry(self.plant.world_body(), X_WG, Box(10, 10, 0.001), "visual", terrain_color)
        self.plant.Finalize()
        
        # Setup trajectory source
        t_traj = self.make_traj_timestamps(traj_to_play_back)
        traj_converted = np.zeros((7,traj_to_play_back.shape[0]))
        traj_converted[0:4,:] = traj_to_play_back[:,CUBE_DATA_QUATERNION_SLICE].T
        traj_converted[4:,:] = traj_to_play_back[:,CUBE_DATA_POSITION_SLICE].T

        pp_traj = PiecewisePolynomial.FirstOrderHold(t_traj, traj_converted)
        
        # Wire up the simulation
        self.traj_source = self.builder.AddSystem(TrajectorySource(pp_traj))
        self.q_to_pose = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))
        self.builder.Connect(self.traj_source.get_output_port(), self.q_to_pose.get_input_port())
        self.builder.Connect(self.q_to_pose.get_output_port(), self.scene_graph.get_source_pose_port(plant_id))        
        
        DrakeVisualizer.AddToBuilder(self.builder, self.scene_graph)
        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.sim.set_publish_every_time_step(True)

    def visualize_two_cubes(self, cube_data, sim_data, realtime_rate):
        self.builder = DiagramBuilder()
        
        # Add a cube as MultibodyPlant
        self.plant = MultibodyPlant(self.drake_sim_dt)
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        plant_id = self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)

        terrain_normal=np.array([0.0, 0.0, 1.0])
        terrain_point=np.zeros((3,))
        terrain_color=np.array([0.8, 0.8, 0.8, 1.0])
        X_WG = RigidTransform(HalfSpace.MakePose(terrain_normal, terrain_point))
        red_cube = Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_parameter_learning/urdf/cube.urdf"), model_name="red_cube")
        blue_cube = Parser(self.plant).AddModelFromFile(
            FindResourceOrThrow(
                "examples/contact_parameter_learning/urdf/cube_blue.urdf"), model_name="blue_cube")
        self.plant.RegisterVisualGeometry(self.plant.world_body(), X_WG, Box(10, 10, 0.001), "visual", terrain_color)
        self.plant.Finalize()
        
        # Setup trajectory source
        t_traj = self.make_traj_timestamps(cube_data)[:cube_data.shape[0]]
        cube_data_converted = np.zeros((14,cube_data.shape[0]))
        cube_data_converted[0:4,:] = cube_data[:,CUBE_DATA_QUATERNION_SLICE].T
        cube_data_converted[4:7,:] = cube_data[:,CUBE_DATA_POSITION_SLICE].T
        cube_data_converted[7:11,:] = sim_data[:,CUBE_DATA_QUATERNION_SLICE].T
        cube_data_converted[11:,:] = sim_data[:,CUBE_DATA_POSITION_SLICE].T

        pp_traj = PiecewisePolynomial.FirstOrderHold(t_traj, cube_data_converted)
        
        # Wire up the simulation
        self.traj_source = self.builder.AddSystem(TrajectorySource(pp_traj))
        self.q_to_pose = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))
        self.builder.Connect(self.traj_source.get_output_port(), self.q_to_pose.get_input_port())
        self.builder.Connect(self.q_to_pose.get_output_port(), self.scene_graph.get_source_pose_port(plant_id))        
        
        DrakeVisualizer.AddToBuilder(self.builder, self.scene_graph)
        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.sim.set_publish_every_time_step(True)

        t_end = CUBE_DATA_DT * cube_data.shape[0]
        self.sim.set_target_realtime_rate(realtime_rate)
        
        while(True):
            self.sim.get_mutable_context().SetTime(0.0)
            self.sim.Initialize()
            self.sim.AdvanceTo(t_end)

        

    def sim_step(self, dt):
        data_arr = np.zeros((1,13))

        cube_state = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_mutable_context()))

        data_arr[0, CUBE_DATA_QUATERNION_SLICE] = cube_state[0:4]
        data_arr[0, CUBE_DATA_POSITION_SLICE] = cube_state[4:7]
        data_arr[0, CUBE_DATA_OMEGA_SLICE] = cube_state[7:10]
        data_arr[0, CUBE_DATA_VELOCITY_SLICE] = cube_state[10:]
        
        new_time = self.sim.get_mutable_context().get_time() + dt
        self.sim.AdvanceTo(new_time)

        data_arr[0] = self.reexpress_state_global_to_local_omega(data_arr[0])
        return data_arr

    def set_initial_condition(self, state):
        q = np.zeros((self.plant.num_positions(),))
        v = np.zeros((self.plant.num_velocities(),))

        initial_state = self.reexpress_state_local_to_global_omega(state)

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


    def visualize_data_file(self, data_folder, toss_id):
        data_file = data_folder + str(toss_id) + '.pt'
        toss = load_cube_toss(data_file)
        self.visualize=True
        self.init_sim(default_drake_contact_params)
        self.visualize_data_rollout(toss)

    def visualize_data_rollout(self, data):
        self.init_playback_sim(data)
        t_end = CUBE_DATA_DT * data.shape[0]
        self.sim.set_target_realtime_rate(1.0)
        
        while(True):
            self.sim.get_mutable_context().SetTime(0.0)
            self.sim.Initialize()
            self.sim.AdvanceTo(t_end)
            

    
    def visualize_sim_rollout(self, params, initial_state, steps):
        self.visualize=True
        self.init_sim(params)
        data = self.get_sim_traj_initial_state(initial_state, steps, 0)
        self.visualize_data_rollout(data)
