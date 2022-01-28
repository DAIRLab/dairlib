from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
from pydrake.all import *
from pydairlib.common import FindResourceOrThrow

# Load the URDF and the cube
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-4)
addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0, mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
parser.package_map().Add("robot_properties_fingers", "examples/trifinger/robot_properties_fingers")
parser.AddModelFromFile(FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/urdf/trifinger.urdf"))
parser.AddModelFromFile(FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf"))

# Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI)
plant.Finalize()

# Constuct the simulator and visualizer
DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)
diagram = builder.Build()
simulator = Simulator(diagram)
simulator.Initialize()
simulator.set_target_realtime_rate(1)

plant_context = diagram.GetMutableSubsystemContext(
    plant, simulator.get_mutable_context())

# Set a dummy controller with open-loop, constant torques
plant.get_actuation_input_port().FixValue(
    plant_context, -.03*np.ones(plant.num_actuators()))

# Set the initial state
q = np.zeros(plant.num_positions())
q_map = makeNameToPositionsMap(plant)
q[q_map['finger_base_to_upper_joint_0']] = 0
q[q_map['finger_upper_to_middle_joint_0']] = -1
q[q_map['finger_middle_to_lower_joint_0']] = -1.5
q[q_map['finger_base_to_upper_joint_0']] = 0
q[q_map['finger_upper_to_middle_joint_120']] = -1
q[q_map['finger_middle_to_lower_joint_120']] = -1.5
q[q_map['finger_base_to_upper_joint_240']] = 0
q[q_map['finger_upper_to_middle_joint_240']] = -1
q[q_map['finger_middle_to_lower_joint_240']] = -1.5
q[q_map['base_qw']] = 1
q[q_map['base_qx']] = 0
q[q_map['base_qz']] = 0
q[q_map['base_x']] = 0
q[q_map['base_y']] = 0
q[q_map['base_z']] = .05
plant.SetPositions(plant_context, q)

# Simulate for 1 second
simulator.AdvanceTo(1)
