from dairlib import (lcmt_robot_output, lcmt_robot_input, lcmt_c3)

import pydairlib.common
import pydairlib.lcm
from pydairlib.systems import (RobotCommandSender, RobotOutputReceiver, RobotC3Receiver,
                               LcmOutputDrivenLoop, OutputVector,
                               TimestampedVector)

# from pydrake.all import (AbstractValue, DiagramBuilder, DrakeLcm, LeafSystem,
#                           MultibodyPlant, Parser, RigidTransform, Subscriber,
#                           LcmPublisherSystem, TriggerType, AddMultibodyPlantSceneGraph)
#import pydairlib.common
from pydrake.all import *


from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
import pydairlib.common
from pydairlib.systems.controllers import C3Controller
from pydairlib.systems.impedance_controllers import ImpedanceController

import numpy as np

from pydrake.trajectories import PiecewisePolynomial

import math


lcm = DrakeLcm()
builder = DiagramBuilder()

# initialize plant for controller
plant = MultibodyPlant(0.0)

parser = Parser(plant)
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka/robot_properties_fingers/urdf/franka_box.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka/robot_properties_fingers/urdf/sphere.urdf"))

X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
plant.Finalize()

builder = DiagramBuilder()
state_receiver = builder.AddSystem(RobotOutputReceiver(plant))

# intialize plant for contact_geoms
builder_f = DiagramBuilder()
plant_f, scene_graph = AddMultibodyPlantSceneGraph(builder_f, 0.0)

parser = Parser(plant_f)
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka/robot_properties_fingers/urdf/franka_box.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka/robot_properties_fingers/urdf/sphere.urdf"))

X_WI = RigidTransform.Identity()
plant_f.WeldFrames(plant_f.world_frame(), plant_f.GetFrameByName("panda_link0"), X_WI)
plant_f.Finalize()

diagram_f = builder_f.Build()
diagram_context_f = diagram_f.CreateDefaultContext()
context_f = diagram_f.GetMutableSubsystemContext(plant_f, diagram_context_f)

################################################################################################

nq = plant.num_positions()
nv = plant.num_velocities()
nu = plant.num_actuators()

q = np.zeros((nq,1))

context = plant.CreateDefaultContext()

# gains
translational_stiffness = 450 #450
rotational_stiffness = 5
coeff = 1  #1

K = np.zeros((6,6))
B = np.zeros((6,6))
K[0:3, 0:3] = rotational_stiffness * np.identity(3)
K[3:6, 3:6] = translational_stiffness * np.identity(3)
B[0:3, 0:3] = coeff * math.sqrt(rotational_stiffness) * np.identity(3)
B[3:6, 3:6] = coeff * math.sqrt(translational_stiffness) * np.identity(3)

K_null = np.identity(7)
B_null = np.identity(7)
qd = np.array([0, 0.4, 0, -1.57, 0, 1.57, 0])

sphere_geoms = plant_f.GetCollisionGeometriesForBody(plant.GetBodyByName("sphere"))[0]
EE_geoms = plant_f.GetCollisionGeometriesForBody(plant.GetBodyByName("panda_link8"))[0]
contact_geoms = [EE_geoms, sphere_geoms]
num_friction_directions = 2

moving_offset = 0.001
pushing_offset = -0.001

controller = builder.AddSystem(
    ImpedanceController(plant,
                        plant_f,
                        context,
                        context_f,
                        K, B,
                        K_null, B_null, qd,
                        contact_geoms,
                        num_friction_directions,
                        moving_offset,
                        pushing_offset))


# connections

builder.Connect(state_receiver.get_output_port(0), controller.get_input_port(0))

control_sender = builder.AddSystem(RobotCommandSender(plant))
builder.Connect(controller.get_output_port(), control_sender.get_input_port(0))

control_publisher = builder.AddSystem(LcmPublisherSystem.Make(
    channel="FRANKA_INPUT", lcm_type=lcmt_robot_input, lcm=lcm,
    publish_triggers={TriggerType.kForced},
    publish_period=0.0, use_cpp_serializer=True))
builder.Connect(control_sender.get_output_port(),
                control_publisher.get_input_port())



# TODO: check these connections
c3_subscriber = builder.AddSystem(LcmSubscriberSystem.Make(
    channel="CONTROLLER_INPUT", lcm_type=lcmt_c3, lcm=lcm,
    use_cpp_serializer=True))
c3_receiver = builder.AddSystem(RobotC3Receiver(10, 9+3, 6))
builder.Connect(c3_subscriber.get_output_port(0), c3_receiver.get_input_port(0))
builder.Connect(c3_receiver.get_output_port(0), controller.get_input_port(1))



diagram = builder.Build()

context_d = diagram.CreateDefaultContext()
receiver_context = diagram.GetMutableSubsystemContext(state_receiver, context_d)

# set the initial state of c3_subscriber
# msg_data = 25*[0]
# msg_data[0] = 0.5
# msg_data[1] = 0
# msg_data[2] = 0.12

# initial_c3_msg = lcmt_c3()
# initial_c3_msg.utime = 0
# initial_c3_msg.data_size = 25
# initial_c3_msg.data = msg_data

# subscriber_context = diagram.GetMutableSubsystemContext(c3_subscriber, context_d)
# subscriber_context.SetAbstractState(0, initial_c3_msg)

# mutable_state = subscriber_context.get_mutable_abstract_state(0)
# mutable_state = initial_c3_msg

print("Waiting for first c3 lcm message")
c3_subscriber.WaitForMessage(0, timeout = 0.2)

loop = LcmOutputDrivenLoop(drake_lcm=lcm, diagram=diagram,
                           lcm_parser=state_receiver,
                           input_channel="FRANKA_OUTPUT",
                           is_forced_publish=True)

loop.Simulate(100)