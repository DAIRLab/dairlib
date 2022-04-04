from dairlib import (lcmt_robot_output, lcmt_robot_input)

import pydairlib.common
import pydairlib.lcm
from pydairlib.systems import (RobotCommandSender, RobotOutputReceiver,
                               LcmOutputDrivenLoop, OutputVector,
                               TimestampedVector)

from pydrake.all import (AbstractValue, DiagramBuilder, DrakeLcm, LeafSystem,
                         MultibodyPlant, Parser, RigidTransform, Subscriber,
                         LcmPublisherSystem, TriggerType)

import numpy as np

# A demo controller system
class TrifingerDemoController(LeafSystem):
  def __init__(self, plant):
    LeafSystem.__init__(self)

    self.plant = plant

    # Input is state, output is torque (control action)
    self.DeclareVectorInputPort("x, u, t", OutputVector(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
    self.DeclareVectorOutputPort("u", TimestampedVector(plant.num_actuators()),
                                 self.CalcControl)

  def CalcControl(self, context, output):
    q = self.EvalVectorInput(context, 0).GetPositions()
    v = self.EvalVectorInput(context, 0).GetVelocities()

    # use a simple PD controller with constant setpoint
    kp = 8
    kd = 1
    q_des = np.array([.1, 0, -1, .1, -.5, -1, .1, -.5, -1])
    u = kp*(q_des - q[0:9]) - kd * v[0:9]

    output.SetDataVector(u)
    output.set_timestamp(context.get_time())

lcm = DrakeLcm()

plant = MultibodyPlant(0.0)

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

builder = DiagramBuilder()

state_receiver = builder.AddSystem(RobotOutputReceiver(plant))
controller = builder.AddSystem(TrifingerDemoController(plant))
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

context = diagram.CreateDefaultContext()
receiver_context = diagram.GetMutableSubsystemContext(state_receiver, context)

loop = LcmOutputDrivenLoop(drake_lcm=lcm, diagram=diagram,
                          lcm_parser=state_receiver,
                          input_channel="TRIFINGER_OUTPUT",
                          is_forced_publish=True)

loop.Simulate(10)
