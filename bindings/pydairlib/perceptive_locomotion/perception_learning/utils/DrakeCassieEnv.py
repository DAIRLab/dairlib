import pdb
import os
import numpy as np
from copy import deepcopy
from tqdm import tqdm
from typing import Dict, Tuple, List
import argparse
import multiprocessing
import gymnasium as gym
from gymnasium import spaces
import matplotlib.pyplot as plt
from pydrake.geometry import Meshcat
import math

# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.systems.all import (
    Diagram,
    EventStatus,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    ZeroOrderHold,
    VectorLogSink,
    LogVectorOutput,
)

from pydrake.multibody.tree import BodyIndex, JointIndex, RevoluteJoint, PrismaticJoint
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common import RandomGenerator
from pydrake.math import RotationMatrix

from pydairlib.perceptive_locomotion.perception_learning.utils.DrakeGymEnv import (
    DrakeGymEnv
)

from pydairlib.perceptive_locomotion.perception_learning.utils.true_cost_system import (
    CumulativeCost)

from pydairlib.perceptive_locomotion.systems.alip_lqr_rl import (
    AlipFootstepLQROptions,
    AlipFootstepLQR,
    calc_collision_cost_grid
)

from pydairlib.perceptive_locomotion.systems. \
cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)
# Can use DrawAndSaveDiagramGraph for debugging if necessary
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

from dairlib import lcmt_robot_output, lcmt_robot_input, lcmt_osc_output
from pydrake.systems.all import (
    LcmInterfaceSystem,
    LcmSubscriberSystem,
    LcmPublisherSystem
)
from pydrake.lcm import DrakeLcm
from pydrake.systems.all import LeafSystem, TriggerType
from pydrake.systems.framework import BasicVector
from pydrake.common.value import AbstractValue
from pydairlib import lcm
from dairlib import lcmt_radio_out

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

class Extractor(LeafSystem):
    def __init__(self,):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort(
            "msg", AbstractValue.Make(lcmt_radio_out)
        )
        self.DeclareVectorOutputPort(
            "msg_decode", BasicVector(2), self.decode_message
        )

    def decode_message(self, context: Context, output):
        message = self.get_input_port().Eval(context)
        # print(f"Radio Receiver Signal Good: {message.radioReceiverSignalGood}")
        # print(f"Receiver Medulla Signal Good: {message.receiverMedullaSignalGood}")

        out = np.zeros(2)
        out[0] = message.channel[0]/2
        out[1] = message.channel[1]/2.5
        # print(message.channel[0])
        # print(message.channel[1])
        output.SetFromVector(out)

def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepLQR, Diagram]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    sim_env.set_name("CassieFootstepControllerEnvironment")
    controller = sim_env.AddToBuilderWithFootstepController(builder, AlipFootstepLQR) # AlipFootstep
    controller.set_name("AlipFootstepLQR")

    observation = sim_env.AddToBuilderObservations(builder)
    reward = sim_env.AddToBuilderRewards(builder)
    builder.ExportInput(controller.get_input_port_by_name("action_ue"), "actions")
    
    freq = 0.01 #np.random.uniform(low=0.001, high=0.015)
    footstep_zoh = ZeroOrderHold(freq, 3)
    builder.AddSystem(footstep_zoh)
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )
    ### LCM publisher for lcm_logger
    drake_lcm = DrakeLcm("udpm://239.255.76.67:7667?ttl=0")
    lcm1 = builder.AddSystem(LcmInterfaceSystem(drake_lcm))
    publisher1 = LcmPublisherSystem.Make(
        channel="CASSIE_STATE_SIMULATION",
        lcm_type=lcmt_robot_output,
        lcm=lcm1,
        use_cpp_serializer=True
    )
    lcm2 = builder.AddSystem(LcmInterfaceSystem(drake_lcm))
    publisher2 = LcmPublisherSystem.Make(
        channel="OSC_WALKING",
        lcm_type=lcmt_robot_input,
        lcm=lcm2,
        use_cpp_serializer=True
    )
    lcm3 = builder.AddSystem(LcmInterfaceSystem(drake_lcm))
    publisher3 = LcmPublisherSystem.Make(
        channel="OSC_DEBUG_WALKING",
        lcm_type=lcmt_osc_output,
        lcm=lcm3,
        use_cpp_serializer=True
    )

    builder.AddSystem(publisher1)
    builder.AddSystem(publisher2)
    builder.AddSystem(publisher3)

    builder.Connect(sim_env.get_output_port_by_name('lcmt_robot_output'), publisher1.get_input_port())
    builder.Connect(sim_env.get_output_port_by_name('lcmt_robot_input'), publisher2.get_input_port())
    builder.Connect(sim_env.get_output_port_by_name('lcmt_osc_debug'), publisher3.get_input_port())

    # LCM Subscriber for Xbox control
    # drake_lcm = DrakeLcm("udpm://239.255.76.67:7667?ttl=0")
    # lcm = builder.AddSystem(LcmInterfaceSystem(drake_lcm))
    # subscriber = LcmSubscriberSystem.Make(
    #     channel="CASSIE_VIRTUAL_RADIO",
    #     lcm_type=lcmt_radio_out,
    #     lcm=lcm,
    #     wait_for_message_on_initialization_timeout = np.inf
    #     #use_cpp_serializer=True
    # )
    # extractor = Extractor()
    # builder.AddSystem(subscriber)
    # builder.AddSystem(extractor)

    # # # Connect systems
    # builder.Connect(subscriber.get_output_port(), extractor.get_input_port())
    # builder.Connect(extractor.get_output_port(), controller.get_input_port_by_name("desired_velocity"))

    diagram = builder.Build()
    # DrawAndSaveDiagramGraph(diagram, '../CassieEnv1')
    return sim_env, controller, diagram

def reset_handler(simulator, terrain, evaluate, seed, drake_rng):
    # Get controller from context or simulator
    diagram = simulator.get_system()
    context = diagram.CreateDefaultContext()
    controller = diagram.GetSubsystemByName("AlipFootstepLQR")
    sim_env = diagram.GetSubsystemByName("CassieFootstepControllerEnvironment")
    controller_context = controller.GetMyMutableContextFromRoot(context)
    sim_context = sim_env.GetMyMutableContextFromRoot(context)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic.npz'
        )
    )
    
    datapoint = ic_generator.random()
    # datapoint = ic_generator.choose(10) # 0,1,2,3,4,5,6,7,50,60,90,
    
    datapoint['q'] = np.array([1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
    -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
    -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785])
    datapoint['v'] = np.zeros((22,))

    v_x = 0.4
    v_y = 0.1
    
    if terrain == 'no_obs':
        v_y = 0.4
        vx = np.random.uniform(-v_x, v_x)
        vy = np.random.uniform(-v_y, v_y)
    elif terrain == 'flat_obs':
        # print('flat_obs')
        v_y = 0.4
        vx = np.random.uniform(0.0, v_x)
        vy = np.random.uniform(-v_y, v_y)
    elif terrain == 'reg_stair':
        v_x = 0.5
        vx = np.random.uniform(0.0, v_x)
        vy = np.random.uniform(-v_y, v_y)
    else:
        vx = np.random.uniform(0.0, v_x)
        vy = np.random.uniform(-v_y, v_y)

    datapoint['desired_velocity'] = np.array([vx, 0.0]).flatten()

    # datapoint['desired_velocity'] = np.array([0.6, 0.]).flatten()
    print(datapoint['desired_velocity'])

    if evaluate:
        datapoint['desired_velocity'] = np.array([0.4, 0.]).flatten()
        # print(datapoint['desired_velocity'])

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds

    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    # #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + datapoint['phase'] + t_ds
    context.SetTime(0.0)

    # Change initial settings

    if terrain == 'stair':
        rand = np.random.randint(1,5)
        rand = 3
        if rand in [1,2]:
            yaw = 0.0 # Upstair
            rand = np.random.randint(-6,7)
            rand = 0
            # pos = np.random.uniform(low=.7, high=1.0)
            pos = 0.
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
        else:
            yaw = math.pi # Downstair
            rand = np.random.randint(-6,7)
            rand = 0
            pos = np.random.uniform(low=-1.0, high=-0.7)
            pos = -0.
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
        
        if evaluate:
            yaw = 0.0
            rand = 0.
            pos = 0.
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])

    elif terrain == 'reg_stair':
        rand = np.random.randint(1,5)
        # rand = 1
        if rand in [1,2]:
            yaw = 0.0 # Upstair
            rand = np.random.randint(-6,7)
            pos = np.random.uniform(low=.7, high=1.)
            # rand = 0
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
        else:
            yaw = math.pi # Downstair
            rand = np.random.randint(-6,7)
            pos = np.random.uniform(low=-1., high=-.7)
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])

    elif terrain == 'easy_stair':
        rand = np.random.randint(1,5)
        rand = 1
        if rand in [1,2]:
            yaw = 0.0 # Upstair
            rand = np.random.randint(-6,7)
            pos = np.random.uniform(low=.7, high=1.)
            pos = 0
            rand = 0
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
        else:
            yaw = math.pi # Downstair
            rand = np.random.randint(-6,7)
            pos = np.random.uniform(low=-1., high=-.7)
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])

    elif terrain == 'slope':
        rand = np.random.randint(1,5)
        # rand = 1
        if rand in [1,2]:
            yaw = 0.0 # Upstair
            rand = np.random.randint(-6,7)
            # rand = 0
            pos = np.random.uniform(low=.7, high=1.)
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])
        else:
            yaw = math.pi # Downstair
            rand = np.random.randint(-6,7)
            pos = np.random.uniform(low=-1., high=-0.7)
            datapoint['q'][4:6] = np.array([rand*15 + pos, 0])

    elif terrain == 'no_obs':
        # yaw = np.random.uniform(low=-math.pi, high=math.pi)
        yaw = 0

    elif terrain == 'block':
        rand = np.random.randint(1, 3)
        if rand == 1:
            rand = np.random.randint(1, 5)
            pos = np.random.uniform(low=-0.2, high=.2)
            datapoint['q'][4:6] = np.array([pos, pos])
            if rand == 1: # 90 degrees
                yaw = math.pi/2
            elif rand == 2: # 180 degrees
                yaw = math.pi
            elif rand == 3: # -90 degrees
                yaw = -math.pi/2
            else:
                yaw = 0
        else:
            rand = np.random.randint(1, 5)
            if rand == 1: # 90 degrees
                yaw = math.pi/2
                pos = np.random.uniform(low=-3.5, high=-3)
                datapoint['q'][4:6] = np.array([0, pos])
            elif rand == 2: # 180 degrees
                yaw = math.pi
                pos = np.random.uniform(low=3., high=3.5)
                datapoint['q'][4:6] = np.array([pos, 0])
            elif rand == 3: # -90 degrees
                yaw = -math.pi/2
                pos = np.random.uniform(low=3., high=3.5)
                datapoint['q'][4:6] = np.array([0, pos])
            else:
                yaw = 0
                pos = np.random.uniform(low=-3.5, high=-3.)
                datapoint['q'][4:6] = np.array([pos, 0])
    else: # Flat w obs
        rand = np.random.randint(1, 3)
        # rand = 1
        if rand == 1:
            yaw = np.random.uniform(low=-math.pi, high=math.pi)
            # pos = np.random.uniform(low=-.3, high=.3)
            # datapoint['q'][4:6] = np.array([pos, 0])
        else:
            rand = np.random.randint(1, 5)
            if rand == 1: # 90 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = math.pi/2
                datapoint['q'][4:6] = np.array([rand, -10])
            elif rand == 2: # 180 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = math.pi
                datapoint['q'][4:6] = np.array([10, rand])
            elif rand == 3: # -90 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = -math.pi/2
                datapoint['q'][4:6] = np.array([rand, 10])
            else:
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = 0
                datapoint['q'][4:6] = np.array([-10, rand])

    quat = datapoint['q'][:4]
    quat = quat / np.linalg.norm(quat)
    R_WP = RotationMatrix(Quaternion(quat))

    wRp = RotationMatrix.MakeZRotation(yaw) @ R_WP
    q_pelvis = wRp.ToQuaternion().wxyz()
    w_pelvis = wRp @ datapoint['v'][:3]
    v_pelvis = wRp @ datapoint['v'][3:6]
    datapoint['q'][:4] = q_pelvis
    datapoint['v'][:3] = w_pelvis
    datapoint['v'][3:6] = v_pelvis

    diagram.SetRandomContext(context, drake_rng)
    
    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(context)

    # for body_index in range(23):
    #     body = plant.get_body(BodyIndex(body_index+1))
    #     mass = body.get_mass(plant.CreateDefaultContext())
    #     rand = np.random.uniform(low=.7, high=1.3)
    #     body.SetMass(plant_context, mass * rand)
    #     # print(f"Body {body_index+1}: {body.name()}")

    # for joint_index in range(plant.num_joints()):
    #     joint = plant.get_joint(JointIndex(joint_index))
    #     if isinstance(joint, (RevoluteJoint, PrismaticJoint)):
    #         damping_value = joint.default_damping()
    #         random_damping_factor = np.random.uniform(0.5, 1.5)
    #         new_damping = random_damping_factor * damping_value
    #         joint.SetDamping(plant_context, new_damping)
            # print(f"Joint {joint.name()}: Damping = {new_damping}")
    
    # from pydrake.multibody.tree import JointActuatorIndex
    # for actuator_index in range(plant.num_actuators()):
    #     actuator = plant.get_joint_actuator(JointActuatorIndex(actuator_index))  # Convert index
    #     torque_limit = actuator.effort_limit()
    #     print(f"Torque limit for {actuator.name()} is {torque_limit}")

    # set the context state with the initial conditions from the datapoint
    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context = controller_context,
        value = datapoint['desired_velocity']
    )

    # sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.Initialize()
    return context

def simulate_init(sim_params, evaluate=False):
    rand = np.random.randint(1, 21)
    rand = 1
    if rand in [1,2,3,4]:
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/stair/dustair_{rand}.yaml'
        terrain = 'stair'
    elif rand in [7,8,9]:
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/easy_stair/dustair_{rand}.yaml'
        terrain = 'easy_stair'
    elif rand in [5,6,10,11,12,19]:
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/tilt_stair/stair_{rand}.yaml'
        terrain = 'stair'
    elif rand in [13,14,15,18]:
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/slope/stair_{rand}.yaml'
        terrain = 'slope'
    #elif rand in [14,15,16]:
    #    terrain_yaml = 'params/flat_with_block.yaml'
    #    terrain = 'block'
    elif rand in [16,17]:
        terrain_yaml = 'params/flat.yaml'
        terrain = 'no_obs'
    else: # 20
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/flat_obs/flat_{rand}.yaml'
        terrain = 'flat_obs'

    if evaluate:
        terrain_yaml = 'params/tilt_stair/stair_222.yaml'
        terrain = 'stair'

    print(terrain_yaml)

    sim_params.terrain = './terrain/EVAL/dustair_0.yaml'
    # sim_params.terrain = './terrain_eval/dustair_0.yaml'
    # sim_params.terrain = './thesis_terrain/RL_sim_test/dustair_6.yaml'
    # sim_params.terrain = os.path.join(perception_learning_base_folder, terrain_yaml)
    sim_env, controller, diagram = build_diagram(sim_params)
    simulator = Simulator(diagram)
    simulator.Initialize()

    def monitor(context):
        time_limit = 15

        plant = sim_env.cassie_sim.get_plant()
        plant_context = plant.GetMyContextFromRoot(context)
        # actuation = plant.get_actuation_input_port().Eval(plant_context)

        # if abs(actuation[0]) > 70:
        #     print(actuation[0])
        #     print("hip_roll_left_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque0")
        # if abs(actuation[1]) > 70:
        #     print(actuation[1])
        #     print("hip_roll_right_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque1")
        # if abs(actuation[2]) > 45:
        #     print(actuation[2])
        #     print("hip_yaw_left_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque2")
        # if abs(actuation[3]) > 45:
        #     print(actuation[3])
        #     print("hip_yaw_right_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque3")
        # if abs(actuation[4]) > 70:
        #     print(actuation[4])
        #     print("hip_pitch_left_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque4")
        # if abs(actuation[5]) > 70:
        #     print(actuation[5])
        #     print("hip_pitch_right_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque5")
        # if abs(actuation[6]) > 170:
        #     print(actuation[6])
        #     print("knee_left_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque6")
        # if abs(actuation[7]) > 170:
        #     print(actuation[7])
        #     print("knee_right_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque7")
        # if abs(actuation[8]) > 30:
        #     print(actuation[8])
        #     print("toe_left_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque8")
        # if abs(actuation[9]) > 30:
        #     print(actuation[9])
        #     print("toe_right_motor")
        #     # return EventStatus.ReachedTermination(diagram, "Torque9")

        sim_context = sim_env.GetMyMutableContextFromRoot(context)
        track_error = sim_env.get_output_port_by_name('swing_ft_tracking_error').Eval(sim_context)
        
        # if center of mas is 20cm 
        left_toe_pos = plant.CalcPointsPositions(
            plant_context, plant.GetBodyByName("toe_left").body_frame(),
            np.array([0.02115, 0.056, 0.]), plant.world_frame()
        )
        right_toe_pos = plant.CalcPointsPositions(
            plant_context, plant.GetBodyByName("toe_right").body_frame(),
            np.array([0.02115, 0.056, 0.]), plant.world_frame()
        )
        com = plant.CalcCenterOfMassPositionInWorld(plant_context)
        z1 = com[2] - left_toe_pos[2]
        z2 = com[2] - right_toe_pos[2]

        toe_left = plant.GetBodyByName("toe_left").EvalPoseInWorld(plant_context).translation()
        toe_right = plant.GetBodyByName("toe_right").EvalPoseInWorld(plant_context).translation()

        # if abs(toe_left[0] - toe_right[0]) < 0.125 and abs(toe_left[1] - toe_right[1]) < 0.05 and (context.get_time() > 1.):
        #     return EventStatus.ReachedTermination(diagram, "Self collision")

        # if abs(com[1]) > 4:
        #     print(com)
        #     print("Stray")
        #     return EventStatus.ReachedTermination(diagram, "Stray")

        if context.get_time() > time_limit:
            # print(com)
            print("Time")
            return EventStatus.ReachedTermination(diagram, "Max Time Limit")
        
        if z1 < 0.2:
            # print(com)
            print("Toe")
            return EventStatus.ReachedTermination(diagram, "Left Toe Exceeded")

        if z2 < 0.2:
            # print(com)
            print("Toe")
            return EventStatus.ReachedTermination(diagram, "Right Toe Exceeded")

        # scene_graph = sim_env.get_output_port_by_name('scene_graph').Eval(sim_context)
        # front_contact_pt = np.array((-0.0457, 0.112, 0))
        # rear_contact_pt = np.array((0.088, 0, 0))
        # toe_axis = front_contact_pt - rear_contact_pt
        # toe_axis /= np.linalg.norm(toe_axis)

        # toe_left = plant.GetBodyByName("toe_left").EvalPoseInWorld(plant_context)
        # R_WL = toe_left.rotation()
        # p_WL = toe_left.translation()
        # contact_pt = np.array([-0.0641797, 0.094719, 0]) # Local frame
        # p_WQ = R_WL @ contact_pt + p_WL # Transform to world frame
        # left_toe_p = plant.GetBodyByName("toe_left").EvalPoseInWorld(plant_context).translation() + (toe_left_rotation @ toe_axis) * 0.12

        # toe_left_tip = plant.GetBodyByName("toe_left_tip").EvalPoseInWorld(plant_context)
        # left_distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=toe_left_tip.translation(), threshold=1.0)
        # for distances in left_distances:
        #     if distances.distance <= 0.0:
        #         print("Lcollision")
        #         print(distances.distance)
        #         contact_results_port = plant.get_output_port(plant.get_contact_results_output_port().get_index())
        #         contact_results = contact_results_port.Eval(plant_context)
        #         for i in range(contact_results.num_point_pair_contacts()):
        #             contact_info = contact_results.point_pair_contact_info(i)
        #             # print(contact_info)
        #             # body_A = plant.get_body(contact_info.bodyA_index())
        #             body_B = plant.get_body(contact_info.bodyB_index())
        #             contact_force = contact_info.contact_force()
        #             if body_B.name() == "toe_left_tip" and np.linalg.norm(contact_force) > 500:
        #                 print(body_B.name())
        #                 print(contact_force)
        #                 print(np.linalg.norm(contact_force))
                # return EventStatus.ReachedTermination(diagram, "Left Collision")

        # toe_pose = plant.GetBodyByName("toe_right").EvalPoseInWorld(plant_context)
        # R_WL = toe_pose.rotation()
        # p_WL = toe_pose.translation()
        # contact_pt = np.array([-0.0641797, 0.094719, 0])  # Local frame
        # p_WQ = R_WL @ contact_pt + p_WL # Transform to world frame
        # distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=p_WQ, threshold=1.0)
        # right_toe_p = plant.GetBodyByName("toe_right").EvalPoseInWorld(plant_context).translation() + (toe_right_rotation @ toe_axis) * 0.12

        # toe_right_tip = plant.GetBodyByName("toe_right_tip").EvalPoseInWorld(plant_context)
        # distances = scene_graph.ComputeSignedDistanceToPoint(p_WQ=toe_right_tip.translation(), threshold=1.0)
        # for signed_distance in distances:
        #     if signed_distance.distance <= 0.0:
        #         print("Rcollision")
        #         print(signed_distance.distance)
        #         # print(signed_distance.id_G)
        #         # inspector = scene_graph.inspector()
        #         # geometry_name = inspector.GetName(signed_distance.id_G)
        #         # print(geometry_name)
        #         contact_results_port = plant.get_output_port(plant.get_contact_results_output_port().get_index())
        #         contact_results = contact_results_port.Eval(plant_context)
        #         for i in range(contact_results.num_point_pair_contacts()):
        #             contact_info = contact_results.point_pair_contact_info(i)
        #             # print(contact_info)
        #             # body_A = plant.get_body(contact_info.bodyA_index())
        #             body_B = plant.get_body(contact_info.bodyB_index())
        #             contact_force = contact_info.contact_force()
        #             if body_B.name() == "toe_right_tip" and np.linalg.norm(contact_force) > 500:
        #                 print(body_B.name())
        #                 print(contact_force)
        #                 print(np.linalg.norm(contact_force))
                # return EventStatus.ReachedTermination(diagram, "Right Collision")

        contact_results_port = plant.get_output_port(plant.get_contact_results_output_port().get_index())
        contact_results = contact_results_port.Eval(plant_context)
        for i in range(contact_results.num_point_pair_contacts()):
            contact_info = contact_results.point_pair_contact_info(i)
            # print(contact_info)
            # body_A = plant.get_body(contact_info.bodyA_index())
            body_B = plant.get_body(contact_info.bodyB_index())
            contact_force = contact_info.contact_force()
            if body_B.name() == "toe_left_tip" or body_B.name() == "toe_right_tip":
                # print(body_B.name())
                # print(contact_force)
                if np.linalg.norm(contact_force) > 1000:
                    print(context.get_time())
                    print(np.linalg.norm(contact_force))

        # if track_error > 0.5 and (context.get_time() > 1.):
        #     return EventStatus.ReachedTermination(diagram, "Track Error Exceeded")

        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)

    return simulator, terrain

def DrakeCassieEnv(sim_params: CassieFootstepControllerEnvironmentOptions, evaluate=False):
    
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()

    # random_terrain = True
    simulator, terrain = simulate_init(sim_params, evaluate=evaluate)
    simulator.set_publish_every_time_step(True)
    # Define Action and Observation space.
    la = np.array([-1., -1., -1.])
    ha = np.array([1., 1., 1.])
    action_space = spaces.Box(low=np.asarray(la, dtype="float32"),
                                  high=np.asarray(ha, dtype="float32"),
                                  dtype=np.float32)

    # Joint
    observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                    shape=(3*64*64 +6+16+ +23 +3*64*64,),
                                    dtype=np.float32)
    # ALIP
    # observation_space = spaces.Box(low=-np.inf, high=np.inf,
    #                                 shape=(3*64*64 +6 +23 +3*64*64,),
    #                                 dtype=np.float32)
    # full state
    # observation_space = spaces.Box(low=-np.inf, high=np.inf,
    #                                 shape=(3*64*64+6+23 +23 +3*64*64,),
    #                                 dtype=np.float32)

    # Time_step to match walking
    time_step = 0.025
    
    env = DrakeGymEnv(
        simulator=simulator,
        time_step=time_step,
        action_space=action_space,
        observation_space=observation_space,
        reward="reward",
        action_port_id="actions",
        observation_port_id="observations",
        reset_handler = reset_handler,
        sim_params = sim_params,
        terrain = terrain,
        evaluate = evaluate
        )

    return env