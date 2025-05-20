"""Script to visualize the costs of samples.  Requires the drake_env virtual
environment located at workspace/drake_env, i.e.:

source ~/workspace/drake_env/bin/activate

Visualizes the costs and configurations exported by lcm_log_loader.cc when the
DO_SAMPLE_VISUALIZATIONS is #defined.  That log loading script writes files to
the test/tmp directory.  This script loads those files and visualizes the
information in meshcat.

Run this script with the following command:
python examples/sampling_c3/test/cost_visualization.py jacktoy
"""
import argparse
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


def costs_to_colors(costs):
    """Returns (3, N) array of RGB colors based on costs."""
    # Try doing a map that will make the good samples more obvious.
    normalized_costs = (costs - costs.min()) / (costs.max() - costs.min())
    mapped = np.arctan(normalized_costs * 100) / (np.pi/2)
    mapped /= mapped.max()
    colormap = plt.cm.RdYlGn.reversed()
    rgba_colors = colormap(mapped)
    return rgba_colors[:, :3].T * 255


def visualize(demo_name):

    DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '../../../'))
    print(f"DAIRLIB_DIR: {DAIRLIB_DIR}")
    LOAD_DIR = op.join(DAIRLIB_DIR, f'examples/sampling_c3/{demo_name}/test/tmp')

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

    # Start building the scene for visualization in meshcat.
    builder = DiagramBuilder()
    plant = builder.AddSystem(MultibodyPlant(time_step=0.0))
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    ee_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/end_effector_simple_model.urdf')
    ground_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/ground.urdf')
    if demo_name == "jacktoy":
        jack_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/jack.sdf')
        triad_body_name = "capsule_1"
        quaternion_input = Quaternion(x_lcs_desired[3:7]/np.linalg.norm(x_lcs_desired[3:7]))
    elif demo_name == "push_t":
        jack_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/T_vertical.sdf')
        triad_body_name = "vertical_link"
        quaternion_input = Quaternion(x_lcs_desired[3:7]/np.linalg.norm(x_lcs_desired[3:7]))
    elif demo_name == "ball_rolling":
        jack_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/sphere.urdf')
        triad_body_name = "sphere"
        quaternion_input = Quaternion(x_lcs_desired[3:7]/np.linalg.norm(x_lcs_desired[3:7]))
    elif demo_name == "box_topple":
        jack_urdf = op.join(DAIRLIB_DIR, 'examples/sampling_c3/urdf/box.sdf')
        triad_body_name = "box"
        quaternion_input = Quaternion(x_lcs_desired[3:7]/np.linalg.norm(x_lcs_desired[3:7]))
    else:
        raise ValueError(f"Unknown demo_name: {demo_name}")
    
    # Add the models.
    urdf_path = jack_urdf
    Parser(plant).AddModels(ee_urdf)
    Parser(plant).AddModels(jack_urdf)
    Parser(plant).AddModels(ground_urdf)

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
        plant=plant, scene_graph=scene_graph, body=plant.GetBodyByName(triad_body_name),
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
    meshcat.Delete()
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize costs of samples in meshcat.")
    parser.add_argument("demo_name", choices=["jacktoy","ball_rolling", "box_topple", "push_t"],
                         help="Name of the demo to visualize")
    args = parser.parse_args()
    visualize(args.demo_name)
