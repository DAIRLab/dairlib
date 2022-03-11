from pydrake.all import *

from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
from pydairlib.systems import AddActuationRecieverAndStateSenderLcm
import pydairlib.common

# Load the URDF and the cube
builder = DiagramBuilder()
sim_dt = 1e-4
output_dt = 5e-4

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_dt)
addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0,
               mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
parser.package_map().Add("robot_properties_fingers",
                         "examples/trifinger/robot_properties_fingers")
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/urdf/trifinger_minimal_collision.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf"))

# Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI)
plant.Finalize()

drake_lcm = DrakeLcm()
lcm = builder.AddSystem(LcmInterfaceSystem(drake_lcm));


passthrough = AddActuationRecieverAndStateSenderLcm(
    builder=builder, plant=plant, lcm=lcm, actuator_channel="TRIFINGER_INPUT",
    state_channel="TRIFINGER_OUTPUT", publish_rate=1/output_dt,
    publish_efforts=True, actuator_delay=0.0)
# Constuct the simulator and visualizer
DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

# Data logging [x;u]
nq = plant.num_positions()
nv = plant.num_velocities()
nu = plant.num_actuators()
logger = builder.AddSystem(VectorLogSink(nq + nv + nu, output_dt))

# Multiplex state and input for logger
mux = builder.AddSystem(Multiplexer([nq + nv, nu]))

builder.Connect(plant.get_state_output_port(), mux.get_input_port(0))
builder.Connect(passthrough.get_output_port(), mux.get_input_port(1))
builder.Connect(mux.get_output_port(0), logger.get_input_port(0))


diagram = builder.Build()

simulator = Simulator(diagram)

simulator.set_publish_every_time_step(False);
simulator.set_publish_at_initialization(False);

# Change the real-time rate to above 1 to simulate faster
simulator.set_target_realtime_rate(1)

plant_context = diagram.GetMutableSubsystemContext(
    plant, simulator.get_mutable_context())

# Set the initial state
q = np.zeros(nq)
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

simulator.Initialize()
# Simulate for 10 seconds
simulator.AdvanceTo(10)

# numpy array of data (nq+nv+nu) x n_time
data = logger.FindLog(simulator.get_context()).data()
