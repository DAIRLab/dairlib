from pydrake.all import (AbstractValue, DiagramBuilder, DrakeLcm, LeafSystem, MultibodyPlant,
                         Parser, RigidTransform, Subscriber, LcmPublisherSystem)
from dairlib import (lcmt_robot_output, lcmt_robot_input)

import pydairlib.common
from pydairlib.systems import (RobotCommandSender, RobotOutputReceiver, LcmOutputDrivenLoop)

import pydrake.systems.lcm as mut


# A demo controller system
class TrifingerDemoController(LeafSystem):
  def __init__(self, plant):
    LeafSystem.__init__(self)

    self.plant = plant

    # Input is state, output is torque (control action)
    # Input port is realy a RobotOutputVector, which needs to be bound to Python
    self.DeclareVectorInputPort("x, u, t", plant.num_positions() +
        plant.num_velocities() + plant.num_actuators() + 4)
    self.DeclareVectorOutputPort("u", plant.num_actuators()+1, self.CalcControl)

  def CalcControl(self, context, output):
    x = self.EvalVectorInput(context, 0).get_value()
    # q and v are [fingers; cube]
    # cube position is [quat; xyz] and velocity [ang_vel; xyz]
    q = x[0:self.plant.num_positions()]
    v = x[self.plant.num_positions():self.plant.num_positions() +
                                     self.plant.num_velocities()]
    # u = -.03*np.ones(self.plant.num_actuators())

    # use a simple PD controller with constant setpoint
    kp = 10
    kd = 2
    q_des = np.array([.1, 0, -1, .1, -.5, -1, .1, -.5, -1])
    u = kp*(q_des - q[0:9]) - kd * v[0:9]

    # concatenate time to output (actually a TimestampedVector, which needs to
    # be bound to Python)
    output.SetFromVector(np.append(u, context.time()))

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

# control_serializer = 
control_publisher = builder.AddSystem(LcmPublisherSystem.Make(
    channel="TRIFINGER_INPUT", lcm_type=lcmt_robot_input, lcm=lcm,
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

loop.Simulate(100)

# input_sub = Subscriber(lcm, "TRIFINGER_OUTPUT", lcmt_robot_output)

# while True:
#     lcm.HandleSubscriptions(0)
#     if (input_sub.count > 0):
#         print(input_sub.message)
#         state_receiver.get_input_port(0).FixValue(
#             receiver_context, AbstractValue.Make(input_sub.message))
#         input_sub.clear()




# from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
# import pydairlib.common

# # A demo controller system
# class TrifingerDemoController(LeafSystem):
#   def __init__(self, plant):
#     LeafSystem.__init__(self)

#     self.plant = plant

#     # Input is state, output is torque (control action)
#     self.DeclareVectorInputPort("x", plant.num_positions() +
#         plant.num_velocities())
#     self.DeclareVectorOutputPort("u", plant.num_actuators(), self.CalcControl)

#   def CalcControl(self, context, output):
#     x = self.EvalVectorInput(context, 0).get_value()
#     # q and v are [fingers; cube]
#     # cube position is [quat; xyz] and velocity [ang_vel; xyz]
#     q = x[0:self.plant.num_positions()]
#     v = x[self.plant.num_positions():]
#     # u = -.03*np.ones(self.plant.num_actuators())

#     # use a simple PD controller with constant setpoint
#     kp = 10
#     kd = 2
#     q_des = np.array([.1, 0, -1, .1, -.5, -1, .1, -.5, -1])
#     u = kp*(q_des - q[0:9]) - kd * v[0:9]
#     output.SetFromVector(u)
