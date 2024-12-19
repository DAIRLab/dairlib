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
from scipy.spatial.transform import Rotation as Rot

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
LOAD_DIR = op.join(DAIRLIB_DIR, 'examples/jacktoy/test/tmp')

p_world_to_franka = np.loadtxt(op.join(LOAD_DIR, 'p_world_to_franka.txt'))
p_franka_to_ground = np.loadtxt(op.join(LOAD_DIR, 'p_franka_to_ground.txt'))

jack_urdf = op.join(DAIRLIB_DIR, 'examples/jacktoy/urdf/jack.sdf')
ground_urdf = op.join(DAIRLIB_DIR, 'examples/jacktoy/urdf/ground.urdf')


# Compute the 8 quaternions.
partial_rot_1 = Rot.from_euler('y', -np.pi/4)
partial_rot_2 = Rot.from_euler('x', np.arctan(np.sqrt(2)/2))

rot_x = Rot.from_euler('x', np.pi/2)
rot_y = Rot.from_euler('y', np.pi/2)
rot_z = Rot.from_euler('z', np.pi/2)

rot_all_up = partial_rot_2 * partial_rot_1
rot_red_down = rot_all_up * rot_y
rot_blue_up = rot_red_down * rot_z.inv()
rot_all_down = rot_blue_up * rot_x.inv()
rot_green_up = rot_all_down * rot_z
rot_blue_down = rot_green_up * rot_z
rot_red_up = rot_blue_down * rot_z
rot_green_down = rot_red_up * rot_x

rotations = {'all_up': rot_all_up, 'red_down': rot_red_down,
             'blue_up': rot_blue_up, 'all_down': rot_all_down,
             'green_up': rot_green_up, 'blue_down': rot_blue_down,
             'red_up': rot_red_up, 'green_down': rot_green_down}


# Start building the scene for visualization in meshcat.
builder = DiagramBuilder()
plant = builder.AddSystem(MultibodyPlant(time_step=0.0))
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterAsSourceForSceneGraph(scene_graph)

# Add the models.
urdf_path = jack_urdf
Parser(plant).AddModels(jack_urdf)
Parser(plant).AddModels(ground_urdf)

p_world_to_ground = p_world_to_franka + p_franka_to_ground
X_W_Ground = RigidTransform(RotationMatrix(), p_world_to_ground)
quat_tfs = []
for i, (name, rot) in enumerate(rotations.items()):
    quat_tfs.append(RigidTransform(
        quaternion=Quaternion(rot.as_quat(scalar_first=True)),
        p=np.array([0.1*(i+1), 0, 0.1])
    ))
    plant.AddFrame(
        FixedOffsetFrame(name=name, P=plant.world_frame(), X_PF=quat_tfs[-1])
    )
    AddFrameTriadIllustration(
        plant=plant, scene_graph=scene_graph,
        frame=plant.GetFrameByName(name),
        length=0.1, radius=0.005, opacity=1.0)

# Add some triads for the goal and current configuration.
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("ground"), X_W_Ground)
AddFrameTriadIllustration(
    plant=plant, scene_graph=scene_graph, body=plant.GetBodyByName("capsule_1"),
    length=0.1, radius=0.005, opacity=1.0)
plant.Finalize()

builder.Connect(plant.get_geometry_pose_output_port(),
                scene_graph.get_source_pose_port(plant.get_source_id()))
builder.Connect(scene_graph.get_query_output_port(),
                plant.get_geometry_query_input_port())

# Add meshcat visualization.
meshcat = StartMeshcat()
visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat, MeshcatVisualizerParams()
)
print(f"Meshcat is running at: {meshcat.web_url()}")

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
plant.SetPositions(plant_context, np.array([1, 0, 0, 0, 0, 0, 0.1]))
sim.AdvanceTo(0)

for i, (name, rot) in enumerate(rotations.items()):
    wxyz = rot.as_quat(scalar_first=True)
    print(f'#define QUAT_{name.upper()} Eigen::Quaterniond({wxyz[0]}, {wxyz[1]}, {wxyz[2]}, {wxyz[3]})')


breakpoint()
