from dairlib import (lcmt_robot_output, lcmt_robot_input)

import pydairlib.common
import pydairlib.lcm
from pydairlib.systems import (RobotCommandSender, RobotOutputReceiver,
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
parser.AddModelFromFile(FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))
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
parser.AddModelFromFile(FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))
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
translational_stiffness = 250
rotational_stiffness = 5
coeff = 1

K = np.zeros((6,6))
B = np.zeros((6,6))
K[0:3, 0:3] = rotational_stiffness * np.identity(3)
K[3:6, 3:6] = translational_stiffness * np.identity(3)
B[0:3, 0:3] = coeff * math.sqrt(rotational_stiffness) * np.identity(3)
B[3:6, 3:6] = coeff * math.sqrt(translational_stiffness) * np.identity(3)

sphere_geoms = plant_f.GetCollisionGeometriesForBody(plant.GetBodyByName("sphere"))[0]
EE_geoms = plant_f.GetCollisionGeometriesForBody(plant.GetBodyByName("panda_link8"))[0]
contact_geoms = [EE_geoms, sphere_geoms]
num_friction_directions = 2

controller = builder.AddSystem(
    ImpedanceController(plant,
                        plant_f,
                        context,
                        context_f,
                        K, B,
                        contact_geoms,
                        num_friction_directions))


# connections

builder.Connect(state_receiver.get_output_port(0), controller.get_input_port(0))

control_sender = builder.AddSystem(RobotCommandSender(plant))
builder.Connect(controller.get_output_port(), control_sender.get_input_port(0))

control_publisher = builder.AddSystem(LcmPublisherSystem.Make(
    channel="TRIFINGER_INPUT", lcm_type=lcmt_robot_input, lcm=lcm,
    publish_triggers={TriggerType.kForced},
    publish_period=0.0, use_cpp_serializer=True))
builder.Connect(control_sender.get_output_port(),
                control_publisher.get_input_port())

diagram = builder.Build()

context_d = diagram.CreateDefaultContext()
receiver_context = diagram.GetMutableSubsystemContext(state_receiver, context_d)

loop = LcmOutputDrivenLoop(drake_lcm=lcm, diagram=diagram,
                           lcm_parser=state_receiver,
                           input_channel="TRIFINGER_OUTPUT",
                           is_forced_publish=True)

loop.Simulate(100)