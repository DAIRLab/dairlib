"""Script to visualize the costs of samples.  Requires the drake_env virtual
environment located at dairlib/drake_env, i.e.:

source ~/workspace/dairlib/drake_env/bin/activate

Visualizes the costs and configurations exported by lcm_log_loader.cc when the
DO_SAMPLE_VISUALIZATIONS is #defined.  That log loading script writes files to
the test/tmp directory.  This script loads those files and visualizes the
information in meshcat.
"""

import matplotlib.pyplot as plt
import numpy as np
import os.path as op

from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, \
    SceneGraph, StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.perception import BaseField, Fields, PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization._triad import AddFrameTriadIllustration


DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '../../..'))
LOAD_DIR = op.join(DAIRLIB_DIR, 'examples/sampling_c3/jacktoy/test/tmp')

# Load the data exported by the LCM log loader.
x_lcs_samples = np.loadtxt(op.join(LOAD_DIR, 'x_lcs_samples.txt'))
costs = np.loadtxt(op.join(LOAD_DIR, 'costs.txt'))
x_lcs_desired = np.loadtxt(op.join(LOAD_DIR, 'x_lcs_desired.txt'))
p_world_to_franka = np.loadtxt(op.join(LOAD_DIR, 'p_world_to_franka.txt'))
p_franka_to_ground = np.loadtxt(op.join(LOAD_DIR, 'p_franka_to_ground.txt'))

# Center the jack at the origin for faster panning/viewing in meshcat.
jack_xy = x_lcs_samples[0, 7:9].copy()
x_lcs_samples[:, :2] -= jack_xy
x_lcs_samples[:, 7:9] -= jack_xy
x_lcs_desired[:2] -= jack_xy
x_lcs_desired[7:9] -= jack_xy


# ==== Paste below the output from LCM log loader ==== #
ee_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/end_effector_simple_model.urdf')
jack_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/jack.sdf')
# ==== Paste above the output from LCM log loader ==== #


def costs_to_colors(costs):
    """Returns (3, N) array of RGB colors based on costs."""
    # Try doing a map that will make the good samples more obvious.
    normalized_costs = (costs - costs.min()) / (costs.max() - costs.min())
    mapped = np.arctan(normalized_costs * 100) / (np.pi/2)
    mapped /= mapped.max()
    colormap = plt.cm.RdYlGn.reversed()
    rgba_colors = colormap(mapped)
    return rgba_colors[:, :3].T * 255


# Start building the scene for visualization in meshcat.
builder = DiagramBuilder()
plant = builder.AddSystem(MultibodyPlant(time_step=0.0))
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterAsSourceForSceneGraph(scene_graph)

# Add the models.
urdf_path = jack_urdf
Parser(plant).AddModels(ee_urdf)
Parser(plant).AddModels(jack_urdf)
Parser(plant).AddModels(jack_urdf.replace('jack.sdf', 'ground.urdf'))

p_world_to_ground = p_world_to_franka + p_franka_to_ground
X_W_Ground = RigidTransform(RotationMatrix(), p_world_to_ground)
quaternion_input = Quaternion(x_lcs_desired[3:7]/np.linalg.norm(x_lcs_desired[3:7]))
X_WGoal = RigidTransform(
    quaternion=quaternion_input, p=x_lcs_desired[7:10])
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base_link"), RigidTransform())
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("ground"), X_W_Ground)
plant.AddFrame(
    FixedOffsetFrame(name="goal", P=plant.world_frame(), X_PF=X_WGoal))

# Add some triads for the goal and current configuration.
AddFrameTriadIllustration(
    plant=plant, scene_graph=scene_graph, body=plant.GetBodyByName("capsule_1"),
    length=0.1, radius=0.005, opacity=1.0)
AddFrameTriadIllustration(
    plant=plant, scene_graph=scene_graph, frame=plant.GetFrameByName("goal"),
    length=0.1, radius=0.005, opacity=1.0)
plant.Finalize()

builder.Connect(plant.get_geometry_pose_output_port(),
                scene_graph.get_source_pose_port(plant.get_source_id()))
builder.Connect(scene_graph.get_query_output_port(),
                plant.get_geometry_query_input_port())

# Add a point cloud to represent the samples and their associated costs.
fields = Fields(BaseField.kXYZs | BaseField.kRGBs)
point_cloud = PointCloud(new_size=costs.shape[0], fields=fields)
point_cloud.mutable_xyzs()[:] = x_lcs_samples[:, :3].T
point_cloud.mutable_rgbs()[:] = costs_to_colors(costs)

# Add meshcat visualization.
meshcat = StartMeshcat()
visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat, MeshcatVisualizerParams()
)
print(f"Meshcat is running at: {meshcat.web_url()}")

meshcat.SetObject(path="point_cloud", cloud=point_cloud, point_size=0.005)

# Build the diagram.
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Start a simulator.
sim = Simulator(diagram)
sim.set_publish_at_initialization(True)
sim.set_publish_every_time_step(True)
sim.Initialize()

# Set the pose of the model.
plant_context = plant.GetMyMutableContextFromRoot(sim.get_mutable_context())
plant.SetPositionsAndVelocities(plant_context, x_lcs_samples[0])
sim.AdvanceTo(0)


breakpoint()
