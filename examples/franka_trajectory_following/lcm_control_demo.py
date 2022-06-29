from pyexpat.model import XML_CQUANT_NONE
from dairlib import (lcmt_robot_output, lcmt_robot_input, lcmt_c3)

from pydrake.common.yaml import yaml_load
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

from pydairlib.multibody import (addFlatTerrain, makeNameToPositionsMap, makeNameToVelocitiesMap)
import pydairlib.common
from pydairlib.systems.controllers_franka import C3Controller_franka

import numpy as np

from pydrake.trajectories import PiecewisePolynomial

import math

# load parameters
param = yaml_load(
    filename="examples/franka_trajectory_following/parameters.yaml")
    
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
                         "examples/franka_trajectory_following/robot_properties_fingers")
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/trifinger_minimal_collision_2.urdf"))
# parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
#     "examples/trifinger_simple/robot_properties_fingers/cube/cube_v2.urdf"))
parser_f.AddModelFromFile(pydairlib.common.FindResourceOrThrow(
    "examples/franka_trajectory_following/robot_properties_fingers/urdf/sphere.urdf"))


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

builder_franka = DiagramBuilder()
sim_dt = 1e-4
output_dt = 1e-4

plant_franka, scene_graph_franka = AddMultibodyPlantSceneGraph(builder_franka, sim_dt)
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
v_map = makeNameToVelocitiesMap(plant)

finger_init = param["q_init_finger"]
q[0] = finger_init[0]
q[1] = finger_init[1]
q[2] = finger_init[2]

ball_init = param["q_init_ball_c3"]
q[q_map['base_qw']] = ball_init[0]
q[q_map['base_qx']] = ball_init[1]
q[q_map['base_qy']] = ball_init[2]
q[q_map['base_qz']] = ball_init[3]
q[q_map['base_x']] = ball_init[4]
q[q_map['base_y']] = ball_init[5]
q[q_map['base_z']] = ball_init[6]

mu = param["mu"]

Qinit = param["Q_default"] * np.eye(nq+nv)
Qinit[0,0] = param["Q_finger"]
Qinit[1,1] = param["Q_finger"]
Qinit[2,2] = param["Q_finger"]
Qinit[7,7] = param["Q_ball_x"]
Qinit[8,8] = param["Q_ball_y"]
Qinit[10:10+nv,10:10+nv] = param["Q_ball_vel"] * np.eye(nv)
Qinit[10:13,10:13] = param["Q_finger_vel"]*np.eye(3) #10
# Qinit[13:16, 13:16] = 0*np.eye(3) # zero out cost on rotational velocity
Rinit = param["R"] * np.eye(nu) #torques

#admm_params
Ginit = param["G"] * np.eye(nq+nv+nu+6*nc)
Uinit = param["U_default"] * np.eye(nq+nv+nu+6*nc)
Uinit[0:nq+nv,0:nq+nv] = param["U_pos_vel"] * np.eye(nq+nv) 
Uinit[nq+nv+6*nc:nq+nv+nu+6*nc, nq+nv+6*nc:nq+nv+nu+6*nc] = param["U_u"] * np.eye(nu)

xdesiredinit = np.zeros((nq+nv,1))
xdesiredinit[:nq] = q


r = param["traj_radius"]
x_c = param["x_c"]
y_c = param["y_c"]

degree_increment = param["degree_increment"]
if param["hold_order"] == 0:
    theta = np.arange(degree_increment, 400 + degree_increment, degree_increment)
elif param["hold_order"] == 1:
    theta = np.arange(0, 400, degree_increment)
xtraj = []
for i in theta:
    x = r * np.sin(math.radians(i+param["phase"]))
    y = r * np.cos(math.radians(i+param["phase"]))
    # x = r * np.sin(math.radians(i+param["phase"]+90)) * np.cos(math.radians(i+param["phase"]+90))
    # y = r * np.sin(math.radians(i+param["phase"]+90))
    q[q_map['base_x']] = x + x_c
    q[q_map['base_y']] = y + y_c
    q[q_map['base_z']] = param["ball_radius"]
    xtraj_hold = np.zeros((nq+nv,1))
    xtraj_hold[:nq] = q
    xtraj.append(xtraj_hold)

time_increment = 1.0*param["time_increment"]  # also try just moving in a straight line maybe
delay = param["stabilize_time1"] + param["move_time"] + param["stabilize_time2"]
timings = np.arange(delay, time_increment*len(xtraj) + delay, time_increment)

if param["hold_order"] == 0:
    pp = PiecewisePolynomial.ZeroOrderHold(timings, xtraj)
elif param["hold_order"] == 1:
    pp = PiecewisePolynomial.FirstOrderHold(timings, xtraj)


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
state_force_sender = builder.AddSystem(RobotC3Sender(10, 9, 6, 9))


builder.Connect(state_receiver.get_output_port(0), controller.get_input_port(0))

builder.Connect(controller.get_output_port(), state_force_sender.get_input_port(0))

control_publisher = builder.AddSystem(LcmPublisherSystem.Make(
    channel="CONTROLLER_INPUT", lcm_type=lcmt_c3, lcm=lcm,
    publish_triggers={TriggerType.kForced},
    publish_period=0.0, use_cpp_serializer=True))
builder.Connect(state_force_sender.get_output_port(),
    control_publisher.get_input_port())

# TODO: check these connections
# lcm_passthrough = builder.AddSystem(LcmInterfaceSystem(lcm))
# passthrough = AddActuationRecieverAndStateSenderLcm(
#     builder=builder, plant=plant_franka, lcm=lcm_passthrough, actuator_channel="FRANKA_INPUT",
#     state_channel="FRANKA_OUTPUT", publish_rate=1/output_dt,
#     publish_efforts=True, actuator_delay=0.0)   #1/output_dt

diagram = builder.Build()

context_d = diagram.CreateDefaultContext()
receiver_context = diagram.GetMutableSubsystemContext(state_receiver, context_d)

loop = LcmOutputDrivenLoop(drake_lcm=lcm, diagram=diagram,
                          lcm_parser=state_receiver,
                          input_channel="FRANKA_OUTPUT",
                          is_forced_publish=True)

loop.Simulate(200)
