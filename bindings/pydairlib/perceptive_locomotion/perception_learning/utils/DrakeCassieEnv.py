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

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"
#sim_params = CassieFootstepControllerEnvironmentOptions()

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

    # cost = False
    # if cost:
    #     cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)
    #     cost_zoh = ZeroOrderHold(0.05, 1) # only need to log the cost at sparse intervals, since it updates once per stride
    #     cost_logger = VectorLogSink(1)
    #     builder.AddSystem(cost_zoh)
    #     builder.AddSystem(cost_logger)
    #     builder.Connect(
    #         cost_system.get_output_port(),
    #         cost_zoh.get_input_port()
    #     )
    #     builder.Connect(
    #         cost_zoh.get_output_port(),
    #         cost_logger.get_input_port()
    #     )
    # else:
    #     cost_logger = 0
    
    freq = np.random.uniform(low=0.001, high=0.025)
    #print(f'Zero Order Hold : {freq}')
    footstep_zoh = ZeroOrderHold(freq, 3)#ZeroOrderHold(1.0 / 30.0, 3)
    #footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)

    builder.AddSystem(footstep_zoh)
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )

    diagram = builder.Build()

    #DrawAndSaveDiagramGraph(diagram, '../CassieEnv_dist')
    return sim_env, controller, diagram#, cost_logger


def reset_handler(simulator, terrain, seed, drake_rng):
    #np.random.seed(seed)
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
            'tmp/ic_new.npz'
        )
    )
    
    #datapoint = ic_generator.random()
    datapoint = ic_generator.choose(0) # 0,1,2,3,4,5,6,7,50,60,90,
    
    v_x = 0.8
    v_y = 0.2
    
    if terrain == 'no_obs':
        v_y = 0.4
        vx = np.random.uniform(-v_x, v_x)
        vy = np.random.uniform(-v_y, v_y)
    else:
        vx = np.random.uniform(0.0, v_x)
        vy = np.random.uniform(-v_y, v_y)
    datapoint['desired_velocity'] = np.array([vx, vy]).flatten()

    #datapoint['desired_velocity'] = np.array([0.6, 0.]).flatten()
    print(datapoint['desired_velocity'])

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds

    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + datapoint['phase'] + t_ds
    context.SetTime(t_init)

    # Change initial settings
    if terrain == 'stair':
        rand = np.random.randint(1,4)
        if rand in [1,2]:
            yaw = 0.0 # Upstair
        else:
            yaw = math.pi # Downstair
        
        rand = np.random.randint(1, 4)
        if rand == 1:
            #rand = np.random.uniform(low=-8.0, high=8.0)
            rand = 0
            datapoint['q'][4:6] = np.array([-15., rand])
        elif rand == 2:
            #rand = np.random.uniform(low=-8.0, high=8.0)
            rand = 0
            datapoint['q'][4:6] = np.array([15., rand])
        else:
            #rand = np.random.uniform(low=-8.0, high=8.0)
            rand = 0
            datapoint['q'][4:6] = np.array([0., rand])
    elif terrain == 'no_obs':
        yaw = np.random.uniform(low=-math.pi, high=math.pi)
    else: # Flat
        rand = np.random.randint(1, 3)
        if rand == 1:
            yaw = np.random.uniform(low=-math.pi, high=math.pi)
        else:
            rand = np.random.randint(1, 5)
            if rand == 1: # 90 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = math.pi/2
                datapoint['q'][4:6] = np.array([rand, -10.0])
            elif rand == 2: # 180 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = math.pi
                datapoint['q'][4:6] = np.array([10.0, rand])
            elif rand == 3: # -90 degrees
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = -math.pi/2
                datapoint['q'][4:6] = np.array([rand, 10.0])
            else:
                rand = np.random.uniform(low=-8.0, high=8.0)
                yaw = 0
                datapoint['q'][4:6] = np.array([-10.0, rand])

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
    simulator.reset_context(context)
    simulator.Initialize()
    return context

def simulate_init(sim_params):
    rand = np.random.randint(1, 21)
    if rand in [1,2,3,4,5,6,7,8,9,10,11,12]:
        rand = np.random.randint(1, 11)
        if rand in [1,2]:
            rand = np.random.randint(0, 1000)
            terrain_yaml = f'params/easy_stair/dustair_{rand}.yaml'
            terrain = 'stair'
        elif rand in [3,4]:
            rand = np.random.randint(0, 1000)
            terrain_yaml = f'params/normal_stair/dustair_{rand}.yaml'
            terrain = 'stair'
        elif rand in [5,6,7,8]:
            rand = np.random.randint(0, 1000)
            terrain_yaml = f'params/reg_stair/dustair_{rand}.yaml'
            terrain = 'stair'
        else:
            rand = np.random.randint(0, 1000)
            terrain_yaml = f'params/new_stair15_25/dustair_{rand}.yaml'
            terrain = 'stair'
    elif rand in [13,14]:
        rand = np.random.randint(0, 1000)
        terrain_yaml = f'params/slope/stair_{rand}.yaml'
        terrain = 'stair'
    elif rand in [15,16]:
        terrain_yaml = 'params/flat.yaml'
        terrain = 'no_obs'
    else:
        #rand = np.random.randint(1, 4)
        #if rand == 1:
        rand = np.random.randint(0, 1500)
        terrain_yaml = f'params/flat/flat_{rand}.yaml'
        terrain = 'flat'
        # elif rand == 2:
        #     rand = np.random.randint(0, 1000)
        #     terrain_yaml = f'params/normal_flat/flat_{rand}.yaml'
        #     terrain = 'flat'
        # else:
        #     rand = np.random.randint(0, 1000)
        #     terrain_yaml = f'params/hard_flat/flat_{rand}.yaml'
        #     terrain = 'flat'
    print(terrain_yaml)
    # terrain_yaml = 'params/flat.yaml'
    # terrain = 'no_obs'
    # terrain_yaml = 'params/easy_stair/dustair_1.yaml'
    # terrain = 'stair'
    # terrain_yaml = 'params/normal_stair/dustair_944.yaml'
    # terrain = 'stair'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain_yaml)
    sim_env, controller, diagram = build_diagram(sim_params)
    simulator = Simulator(diagram)
    simulator.Initialize()

    def monitor(context):
        time_limit = 8

        plant = sim_env.cassie_sim.get_plant()
        plant_context = plant.GetMyContextFromRoot(context)
        
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
        
        front_contact_pt = np.array((-0.0457, 0.112, 0))
        rear_contact_pt = np.array((0.088, 0, 0))

        toe_left_rotation = plant.GetBodyByName("toe_left").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
        left_toe_direction = toe_left_rotation @ (front_contact_pt - rear_contact_pt)
        left_angle = abs(np.arctan2(left_toe_direction[2], np.linalg.norm(left_toe_direction[:2])))
        
        toe_right_rotation = plant.GetBodyByName("toe_right").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
        right_toe_direction = toe_right_rotation @ (front_contact_pt - rear_contact_pt)
        right_angle = abs(np.arctan2(right_toe_direction[2], np.linalg.norm(right_toe_direction[:2])))

        if context.get_time() > time_limit:
            return EventStatus.ReachedTermination(diagram, "Max Time Limit")
        
        if z1 < 0.2:
            return EventStatus.ReachedTermination(diagram, "Left Toe Exceeded")

        if z2 < 0.2:
            return EventStatus.ReachedTermination(diagram, "Right Toe Exceeded")

        if right_angle > .6:
            return EventStatus.ReachedTermination(diagram, "Right Angle Exceeded")
        
        if left_angle > .6:
            return EventStatus.ReachedTermination(diagram, "Left Angle Exceeded")
        
        if track_error > 0.4 and (context.get_time() > 1.5):
            return EventStatus.ReachedTermination(diagram, "Track Error Exceeded")

        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)

    return simulator, terrain

def DrakeCassieEnv(sim_params: CassieFootstepControllerEnvironmentOptions):
    
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()

    # random_terrain = True
    simulator, terrain = simulate_init(sim_params)
    
    # Define Action and Observation space.
    la = np.array([-1., -1., -1.])
    ha = np.array([1., 1., 1.])
    action_space = spaces.Box(low=np.asarray(la, dtype="float32"),
                                  high=np.asarray(ha, dtype="float32"),
                                  dtype=np.float32)

    # Joint                              
    observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                    shape=(3*64*64 +6+16+23 +3*64*64,),
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
        terrain = terrain
        )

    return env