from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.systems.primitives import TrajectorySource
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box,\
    ProximityProperties, SourceId

'''
    Constructs a DrakeVisualizer and wraps it in a drake simulation to play back
    trajectories for the plant passed to the constructor
'''
class PlantTrajVisualizer:

    # Import an unfinalized plant
    def __init__(self, plant, scene_graph, traj):

        self.plant = plant
        self.traj = traj
        self.builder = DiagramBuilder()
        self.scene_graph = self.builder.AddSystem(scene_graph)
        self.plant.Finalize()

        self.traj_source = self.builder.AddSystem(TrajectorySource(traj))
        self.position_to_pose_source = self.builder.AddSystem(
            MultibodyPositionToGeometryPose(self.plant)
        )
        self.builder.Connect(
            self.traj_source.get_output_port(),
            self.position_to_pose_source.get_input_port()
        )
        self.builder.Connect(
            self.position_to_pose_source.get_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        )
        DrakeVisualizer.AddToBuilder(self.builder, self.scene_graph)

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.sim.set_publish_every_time_step(True)
        self.sim.set_target_realtime_rate(0.5)
        self.sim.Initialize()

    def play(self):
        self.sim.get_mutable_context().SetTime(0.0)
        self.sim.Initialize()
        self.sim.AdvanceTo(self.traj.end_time())
