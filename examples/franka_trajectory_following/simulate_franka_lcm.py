from pydrake.all import *
from pydrake.common.yaml import yaml_load

from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
from pydairlib.systems import AddActuationRecieverAndStateSenderLcm
import pydairlib.common

import pydrake.geometry as mut

import matplotlib.pyplot as plt

# load parameters
param = yaml_load(
    filename="examples/franka_trajectory_following/parameters.yaml")

# Load the URDF and the cube
builder = DiagramBuilder()
sim_dt = param["sim_dt"]
output_dt = param["sim_dt"]

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_dt)
# addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0,
#               mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
# parser.package_map().Add("robot_properties_fingers",
#                          "examples/trajectory_following/robot_properties_fingers")
# parser.AddModelFromFile(FindResourceOrThrow(
#     "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))



#props = mut.ProximityProperties()

#props.AddProperty("material", "point_contact_stiffness", 1000)

# Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
plant.Finalize()

drake_lcm = DrakeLcm()
lcm = builder.AddSystem(LcmInterfaceSystem(drake_lcm))


passthrough = AddActuationRecieverAndStateSenderLcm(
    builder=builder, plant=plant, lcm=lcm, actuator_channel="FRANKA_INPUT",
    state_channel="FRANKA_OUTPUT", publish_rate=1/output_dt,
    publish_efforts=True, actuator_delay=0.0)   #1/output_dt
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

simulator.set_publish_every_time_step(False)
simulator.set_publish_at_initialization(False)

# Change the real-time rate to above 1 to simulate faster
simulator.set_target_realtime_rate(param["realtime_rate"])

plant_context = diagram.GetMutableSubsystemContext(
    plant, simulator.get_mutable_context())

q = np.zeros(nq)
q_map = makeNameToPositionsMap(plant)

# initialize EE close to {0.5, 0, 0.12}[m] in task space
franka_init = param["q_init_franka"]
q[q_map["panda_joint1"]] = franka_init[0]
q[q_map["panda_joint2"]] = franka_init[1]
q[q_map["panda_joint3"]] = franka_init[2]
q[q_map["panda_joint4"]] = franka_init[3]
q[q_map["panda_joint5"]] = franka_init[4]
q[q_map["panda_joint6"]] = franka_init[5]
q[q_map["panda_joint7"]] = franka_init[6]

# initialize ball
ball_init = param["q_init_ball"]
traj_radius = param["traj_radius"]
q[q_map['base_qw']] = ball_init[0]
q[q_map['base_qx']] = ball_init[1]
q[q_map['base_qy']] = ball_init[2]
q[q_map['base_qz']] = ball_init[3]
q[q_map['base_x']] = param["x_c"] + traj_radius * np.sin(math.radians(param["phase"]))
q[q_map['base_y']] = param["y_c"] + traj_radius * np.cos(math.radians(param["phase"]))
q[q_map['base_z']] = param["ball_radius"]

plant.SetPositions(plant_context, q)

v = np.zeros(nv)
plant.SetVelocities(plant_context, v)

simulator.Initialize()
# Simulate for 10 seconds
simulator.AdvanceTo(100)

# numpy array of data (nq+nv+nu) x n_time
data = logger.FindLog(simulator.get_context()).data()
