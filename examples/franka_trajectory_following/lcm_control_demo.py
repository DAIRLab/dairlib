from dairlib import (lcmt_robot_output, lcmt_robot_input, lcmt_c3)

import pydairlib.common
import pydairlib.lcm
from pydairlib.systems import (RobotC3Receiver, 
                               RobotC3Sender,
                               RobotOutputReceiver, RobotOutputSender,
                               LcmOutputDrivenLoop, OutputVector,
                               TimestampedVector,
                               AddActuationRecieverAndStateSenderLcm)

from pydrake.all import (AbstractValue, DiagramBuilder, DrakeLcm, LeafSystem,
                         MultibodyPlant, Parser, RigidTransform, Subscriber,
                         LcmPublisherSystem, TriggerType, AddMultibodyPlantSceneGraph,
                         LcmInterfaceSystem)
#import pydairlib.common

from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap)
import pydairlib.common
from pydairlib.systems.controllers_franka import C3Controller_franka

import numpy as np

from pydrake.trajectories import PiecewisePolynomial

import math


lcm = DrakeLcm()

plant = MultibodyPlant(0.0)


#The package addition here seems necessary due to how the URDF is defined
parser = Parser(plant)
parser.package_map().Add("robot_properties_fingers",
                         "examples/trajectory_following/robot_properties_fingers")
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trajectory_following/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf"))
# parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
#     "examples/trifinger_simple/robot_properties_fingers/cube/cube_v2.urdf"))
parser.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))


#Fix the base of the finger to the world
X_WI = RigidTransform.Identity()
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WI)
plant.Finalize()

builder = DiagramBuilder()

#############################################################################################


builder_f = DiagramBuilder()
sim_dt = 1e-4
plant_f, scene_graph = AddMultibodyPlantSceneGraph(builder_f, 0.0)
# addFlatTerrain(plant=plant_f, scene_graph=scene_graph, mu_static=1.0,
#                mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser_f = Parser(plant_f)
parser_f.package_map().Add("robot_properties_fingers",
                         "examples/trajectory_following/robot_properties_fingers")
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trajectory_following/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf"))
# parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
#     "examples/trifinger_simple/robot_properties_fingers/cube/cube_v2.urdf"))
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))


# Fix the base of the finger to the world
X_WI_f = RigidTransform.Identity()
plant_f.WeldFrames(plant_f.world_frame(), plant_f.GetFrameByName("base_link"), X_WI_f)
plant_f.Finalize()


#context_f = plant_f.CreateDefaultContext()

plant_ad_f = plant_f.ToAutoDiffXd()

context_ad_f = plant_ad_f.CreateDefaultContext()

diagram_f = builder_f.Build()

diagram_context = diagram_f.CreateDefaultContext()

context_f = diagram_f.GetMutableSubsystemContext(plant_f, diagram_context)

#############################################################################################

# builder_franka = DiagramBuilder()
sim_dt = 1e-4
output_dt = 1e-4

plant_franka, scene_graph_franka = AddMultibodyPlantSceneGraph(builder, sim_dt)
# addFlatTerrain(plant=plant, scene_graph=scene_graph, mu_static=1.0,
#               mu_kinetic=1.0)

# The package addition here seems necessary due to how the URDF is defined
parser_franka = Parser(plant_franka)
# parser.package_map().Add("robot_properties_fingers",
#                          "examples/trajectory_following/robot_properties_fingers")
# parser.AddModelFromFile(FindResourceOrThrow(
#     "drake/manipulation/models/franka_description/urdf/panda_arm.urdf"))
parser_franka.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/franka_box.urdf"))
parser_franka.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))



#props = mut.ProximityProperties()

#props.AddProperty("material", "point_contact_stiffness", 1000)

# Fix the base of the finger to the world
X_WI_franka = RigidTransform.Identity()
plant_franka.WeldFrames(plant_franka.world_frame(), plant_franka.GetFrameByName("panda_link0"), X_WI_franka)
plant_franka.Finalize()

context_franka = plant_franka.CreateDefaultContext()

state_receiver = builder.AddSystem(RobotOutputReceiver(plant_franka))



nq = plant.num_positions()
nv = plant.num_velocities()
nu = plant.num_actuators()
nc = 2


q = np.zeros((nq,1))
q_map = makeNameToPositionsMap(plant)
q[0] = -0.05
q[1] = 0.22
q[2] = 0.07 #0.05
q[q_map['base_qw']] = 1
q[q_map['base_qx']] = 0
q[q_map['base_qz']] = 0
q[q_map['base_x']] = 0.05
q[q_map['base_y']] = 0.05
q[q_map['base_z']] = 0

mu = 1.0

Qinit = 0*np.eye(nq+nv)
Qinit[0,0] = 100
Qinit[1,1] = 100
Qinit[2,2] = 100  #100
Qinit[7,7] = 10000 #10000
Qinit[8,8] = 10000   #10000
Qinit[10:10+nv,10:10+nv] = 0.1*np.eye(nv) #velocities
Qinit[10:13,10:13] = 10*np.eye(3) #10
Rinit = 0.01*np.eye(nu) #torques

#admm_params
Ginit = 0.01*np.eye(nq+nv+nu+6*nc)   #0.01
Uinit = 1*np.eye(nq+nv+nu+6*nc)
Uinit[0:nq+nv,0:nq+nv] = 10*np.eye(nq+nv)   #10
Uinit[nq+nv+6*nc:nq+nv+nu+6*nc, nq+nv+6*nc:nq+nv+nu+6*nc] = 1*np.eye(nu)

xdesiredinit = np.zeros((nq+nv,1))
xdesiredinit[:nq] = q


r = 0.2
degree_increment = 20
theta = np.arange(0, 400, degree_increment)
xtraj = []
for i in theta:
    x = r * np.sin(math.radians(i))
    y = r * np.cos(math.radians(i))
    q[q_map['base_x']] = x + 0.5
    q[q_map['base_y']] = y
    xtraj_hold = np.zeros((nq+nv,1))
    xtraj_hold[:nq] = q
    xtraj.append(xtraj_hold)



increment = 2.0
timings = np.arange(0, increment*len(xtraj), increment )

pp = PiecewisePolynomial.ZeroOrderHold(timings, xtraj)


num_friction_directions = 2
N = 5
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

#Qinit[nv-3:nv,nv-3:nv] = 1*np.eye(3) #penalize final velocities
Q.append(Qinit)
xdesired.append(xdesiredinit)

finger_lower_link_0_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("tip_link_1_real"))[0]
#finger_lower_link_120_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("tip_link_2_real"))[0]
#finger_lower_link_240_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("tip_link_3_real"))[0]
#cube_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("cube"))[0]
sphere_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("sphere"))[0]
ground_geoms = plant_f.GetCollisionGeometriesForBody(plant_f.GetBodyByName("box"))[0]

contact_geoms = [finger_lower_link_0_geoms, sphere_geoms, ground_geoms] #finger_lower_link_120_geoms, finger_lower_link_240_geoms,


plant_ad = plant.ToAutoDiffXd()

context = plant.CreateDefaultContext()

context_ad = plant_ad.CreateDefaultContext()

controller = builder.AddSystem(
    C3Controller_franka(plant, plant_f, plant_franka, context, context_f, context_franka, plant_ad, plant_ad_f, context_ad, context_ad_f, scene_graph, diagram_f, contact_geoms, num_friction_directions, mu, Q, R, G, U, xdesired, pp))


builder.Connect(state_receiver.get_output_port(0), controller.get_input_port(0))

state_force_sender = builder.AddSystem(RobotC3Sender(10, 9, 6))
builder.Connect(controller.get_output_port(), state_force_sender.get_input_port(0))

control_publisher = builder.AddSystem(LcmPublisherSystem.Make(
    channel="CONTROLLER_INPUT", lcm_type=lcmt_c3, lcm=lcm,
    publish_triggers={TriggerType.kForced},
    publish_period=0.0, use_cpp_serializer=True))
builder.Connect(state_force_sender.get_output_port(),
    control_publisher.get_input_port())

# TODO: check these connections
lcm_passthrough = builder.AddSystem(LcmInterfaceSystem(lcm))
passthrough = AddActuationRecieverAndStateSenderLcm(
    builder=builder, plant=plant_franka, lcm=lcm_passthrough, actuator_channel="FRANKA_INPUT",
    state_channel="FRANKA_OUTPUT", publish_rate=1/output_dt,
    publish_efforts=True, actuator_delay=0.0)   #1/output_dt

diagram = builder.Build()

context_d = diagram.CreateDefaultContext()
receiver_context = diagram.GetMutableSubsystemContext(state_receiver, context_d)

loop = LcmOutputDrivenLoop(drake_lcm=lcm, diagram=diagram,
                          lcm_parser=state_receiver,
                          input_channel="FRANKA_OUTPUT",
                          is_forced_publish=True)

loop.Simulate(100)
