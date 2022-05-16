from pydrake.all import *

from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
from pydairlib.systems import AddActuationRecieverAndStateSenderLcm
import pydairlib.common

import pydrake.geometry as mut



import matplotlib.pyplot as plt

# Load the URDF and the cube
builder = DiagramBuilder()
sim_dt = 1e-4
output_dt = 1e-4

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_dt)
# addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0,
#               mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
# parser.package_map().Add("robot_properties_fingers",
#                          "examples/trajectory_following/robot_properties_fingers")
parser.AddModelFromFile(FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))


#props = mut.ProximityProperties()

#props.AddProperty("material", "point_contact_stiffness", 1000)

# Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
plant.Finalize()

drake_lcm = DrakeLcm()
lcm = builder.AddSystem(LcmInterfaceSystem(drake_lcm))


passthrough = AddActuationRecieverAndStateSenderLcm(
    builder=builder, plant=plant, lcm=lcm, actuator_channel="TRIFINGER_INPUT",
    state_channel="TRIFINGER_OUTPUT", publish_rate=1/output_dt,
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
simulator.set_target_realtime_rate(1)

plant_context = diagram.GetMutableSubsystemContext(
    plant, simulator.get_mutable_context())

q = 0*np.ones(nq)
plant.SetPositions(plant_context, q)

v = np.zeros(nv)
plant.SetVelocities(plant_context, v)

simulator.Initialize()
# Simulate for 10 seconds
simulator.AdvanceTo(100)

# numpy array of data (nq+nv+nu) x n_time
data = logger.FindLog(simulator.get_context()).data()
