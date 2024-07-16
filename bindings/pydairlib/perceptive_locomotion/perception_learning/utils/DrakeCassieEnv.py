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
    
    freq = np.random.uniform(low=0.001, high=0.05)
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

    #DrawAndSaveDiagramGraph(diagram, '../CassieEnv')
    return sim_env, controller, diagram#, cost_logger

def reset_handler(simulator, terrain, seed):
    np.random.seed(seed)

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
    
    datapoint = ic_generator.random()
    #datapoint = ic_generator.choose(0) # 0,1,2,3,4,5,6,7,50,60,90,
    v_des_theta = 0.1
    v_des_norm = 0.8
    v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    v_norm = np.random.uniform(0.2, v_des_norm)
    datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()
    #datapoint['desired_velocity'] = np.array([0.5, 0.01]).flatten()
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
        rand = np.random.randint(1, 3)
        if rand == 1:
            yaw = math.pi # Downstair
        else:
            yaw = 0.0 # Upstair
        
        rand = np.random.randint(1, 4)
        # if rand == 1:
        #     rand = np.random.uniform(low=-9.0, high=9.0)
        #     datapoint['q'][4:6] = np.array([-30., rand])
        if rand == 1:
            rand = np.random.uniform(low=-8.0, high=8.0)
            datapoint['q'][4:6] = np.array([-15., rand])
        elif rand == 2:
            rand = np.random.uniform(low=-8.0, high=8.0)
            datapoint['q'][4:6] = np.array([15., rand])
        # elif rand == 4:
        #     rand = np.random.uniform(low=-9.0, high=9.0)
        #     datapoint['q'][4:6] = np.array([30., rand])
        else:
            rand = np.random.uniform(low=-8.0, high=8.0)
            datapoint['q'][4:6] = np.array([0., rand])

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
    rand = np.random.randint(1, 3)
    if rand == 1:
        rand = np.random.randint(0, 500)
        terrain_yaml = f'params/easy/du_stair/dustair_{rand}.yaml'
        terrain = 'stair'
    else:
        rand = np.random.randint(0, 500)
        terrain_yaml = f'params/easy/flat/flat_{rand}.yaml'
        terrain = 'flat'
    print(terrain_yaml)
    #sim_params.terrain = 'terrain/flat_0.yaml'
    #sim_params.terrain = 'terrain/dustair_0.yaml'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain_yaml)
    sim_env, controller, diagram = build_diagram(sim_params)
    simulator = Simulator(diagram)
    simulator.Initialize()
    
    def monitor(context):
        time_limit = 10

        plant = sim_env.cassie_sim.get_plant()
        plant_context = plant.GetMyContextFromRoot(context)

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

        if context.get_time() > time_limit:
            return EventStatus.ReachedTermination(diagram, "Max Time Limit")

        if z1 < 0.2:
            return EventStatus.ReachedTermination(diagram, "Left Toe Exceeded")

        if z2 < 0.2:
            return EventStatus.ReachedTermination(diagram, "Right Toe Exceeded")

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
                                  
    observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                    shape=(3*64*64 +6+16+23 +3*64*64,),
                                    dtype=np.float32)

    # Time_step to match walking
    time_step = 0.05
    
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