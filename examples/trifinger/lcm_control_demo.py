
from dairlib import (lcmt_robot_output, lcmt_robot_input)

import pydairlib.common
import pydairlib.lcm
from pydairlib.systems import (RobotCommandSender, RobotOutputReceiver,
                               LcmOutputDrivenLoop, OutputVector,
                               TimestampedVector)

from pydrake.all import (AbstractValue, DiagramBuilder, DrakeLcm, LeafSystem,
                         MultibodyPlant, Parser, RigidTransform, Subscriber,
                         LcmPublisherSystem, TriggerType, AddMultibodyPlantSceneGraph)
#import pydairlib.common


from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
import pydairlib.common
from pydairlib.systems.controllers import C3Controller

import numpy as np

#A demo controller system
class TrifingerDemoController(LeafSystem):
  def __init__(self, plant):
    LeafSystem.__init__(self)

    self.plant = plant

#Input is state, output is torque(control action)
    self.DeclareVectorInputPort("x, u, t", OutputVector(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
    self.DeclareVectorOutputPort("u", TimestampedVector(plant.num_actuators()),
                                 self.CalcControl)

  def CalcControl(self, context, output):
    q = self.EvalVectorInput(context, 0).GetPositions()
    v = self.EvalVectorInput(context, 0).GetVelocities()

#use a simple PD controller with constant setpoint
    kp = 8
    kd = 1
    q_des = np.array([.1, 0, -1, .1, -.5, -1, .1, -.5, -1])
    u = kp*(q_des - q[0:9]) - kd * v[0:9]

    output.SetDataVector(u)
    output.set_timestamp(context.get_time())


lcm = DrakeLcm()

plant = MultibodyPlant(0.0)


#The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
parser.package_map().Add("robot_properties_fingers",
                         "examples/trifinger/robot_properties_fingers")
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/urdf/trifinger_minimal_collision.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf"))

#Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI)
plant.Finalize()

builder = DiagramBuilder()

state_receiver = builder.AddSystem(RobotOutputReceiver(plant))

#############################################################################################


builder_f = DiagramBuilder()
sim_dt = 2e-4
plant_f, scene_graph = AddMultibodyPlantSceneGraph(builder_f, 0.0)
addFlatTerrain(plant=plant_f, scene_graph=scene_graph, mu_static=1.0,
               mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser_f = Parser(plant_f)
parser_f.package_map().Add("robot_properties_fingers",
                         "examples/trifinger/robot_properties_fingers")
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/urdf/trifinger_minimal_collision.urdf"))
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf"))

# Fix the base of the finger to the world
X_WI_f = RigidTransform.Identity()
plant_f.WeldFrames(plant_f.world_frame(), plant_f.GetFrameByName("base_link"), X_WI_f)
plant_f.Finalize()


context_f = plant_f.CreateDefaultContext()

plant_ad_f = plant_f.ToAutoDiffXd()

context_ad_f = plant_ad_f.CreateDefaultContext()

diagram_f = builder_f.Build()


#############################################################################################



mu = 1.0
Qinit = np.eye(31)
Rinit = np.eye(9)
Ginit = 0.1*np.eye(58)
Uinit = np.eye(58)
xdesiredinit = np.zeros((31,1))

num_friction_directions = 2
N = 10
Q = []
R = []
G = []
U = []
xdesired = []

for i in range(N):
    Q.append(Qinit)
    R.append(Rinit)
    G.append(Ginit)
    U.append(Uinit)
    xdesired.append(xdesiredinit)
Q.append(Qinit)

finger_lower_link_0_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("finger_lower_link_0"))[0]
finger_lower_link_120_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("finger_lower_link_120"))[0]
finger_lower_link_240_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("finger_lower_link_240"))[0]
cube_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("cube"))[0]
contact_geoms = [finger_lower_link_0_geoms, finger_lower_link_120_geoms, finger_lower_link_240_geoms, cube_geoms]


plant_ad = plant.ToAutoDiffXd()

context = plant.CreateDefaultContext()

context_ad = plant_ad.CreateDefaultContext()

controller = builder.AddSystem(
    C3Controller(plant, plant_f, context, context_f, plant_ad, plant_ad_f, context_ad, context_ad_f, scene_graph, diagram_f, contact_geoms, num_friction_directions, mu, Q, R, G, U, xdesired))


#controller = builder.AddSystem(TrifingerDemoController(plant))


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

loop.Simulate(10)
