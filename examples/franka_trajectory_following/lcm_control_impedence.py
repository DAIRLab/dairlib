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
from pydairlib.systems.ji_controllers import JIController

import numpy as np

from pydrake.trajectories import PiecewisePolynomial

import math


lcm = DrakeLcm()

plant = MultibodyPlant(0.0)


#The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)

parser.AddModelFromFile(FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))


# Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI)
plant.Finalize()


builder = DiagramBuilder()

state_receiver = builder.AddSystem(RobotOutputReceiver(plant))

#############################################################################################


nq = plant.num_positions()
nv = plant.num_velocities()
nu = plant.num_actuators()


q = np.zeros((nq,1))


context = plant.CreateDefaultContext()

K = np.zeros(1)
B = np.zeros(1)

controller = builder.AddSystem(
    JIController(plant, context, K, B))



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
