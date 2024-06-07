import pdb
import os
import numpy as np
import torch
from tqdm import tqdm
from typing import Dict, Tuple
import argparse
import multiprocessing
from pydrake.geometry import Rgba

# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.geometry import Meshcat
from pydrake.systems.all import (
    Diagram,
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

from pydairlib.perceptive_locomotion.perception_learning.alip_nn_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepNNLQR,
    calc_collision_cost_grid
)
from pydairlib.perceptive_locomotion.perception_learning.utils.terrain_utils import (
    make_stairs, random_stairs
)

from pydairlib.perceptive_locomotion.perception_learning.inference.torch_utils \
    import get_device, tile_and_concatenate_inputs

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)
from pydairlib.perceptive_locomotion.perception_learning.utils.true_cost_system import (
    CumulativeCost)

# Can use DrawAndSaveDiagramGraph for debugging if necessary
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"


def build_diagram(sim_params: CassieFootstepControllerEnvironmentOptions,
                  checkpoint_path, simulate_perception=False) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepNNLQR, Diagram, LogVectorOutput]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller = sim_env.AddToBuilderWithFootstepController(
        builder, AlipFootstepNNLQR, model_path=checkpoint_path, elevation_map=simulate_perception
    )
    cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)

    footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)

    builder.AddSystem(footstep_zoh)
    if simulate_perception:
        builder.Connect(
            sim_env.get_output_port_by_name("height_map"),
            controller.get_input_port_by_name("height_map")
        )
        builder.Connect(
            sim_env.get_output_port_by_name("height_map_query"),
            controller.get_input_port_by_name("height_map_query")
        )
    else:    
        builder.Connect(
            sim_env.get_output_port_by_name('height_map'),
            controller.get_input_port_by_name('height_map')
        )

    builder.Connect(
        sim_env.get_output_port_by_name('state'),
        controller.get_input_port_by_name('xut')
    )

    builder.Connect(
        sim_env.get_output_port_by_name('gt_x_u_t'),
        controller.get_input_port_by_name('gt_x_u_t')
    )    
    
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )
    diagram = builder.Build()
    #DrawAndSaveDiagramGraph(diagram, '../AlipNNLQR_DataCollection')
    return sim_env, controller, diagram

def check_termination(sim_env, diagram_context) -> bool:
    plant = sim_env.cassie_sim.get_plant()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
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
    return z1 < 0.2 or z2 < 0.2

# Use Temporal Difference (TD(0)) as the Value target
def calculate_returns(rewards, gamma=0.99):
    n = len(rewards)
    V_targets = [0] * n
    next_value = 0  # Initialize next value as 0 (for terminal states)

    for t in reversed(range(n)):
        V_targets[t] = rewards[t] + gamma * next_value
        next_value = V_targets[t]

    return V_targets

def run(sim_env, controller, diagram, simulate_perception=False, plot=False):
    
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic.npz'
        )
    )
    
    datapoint = ic_generator.random()
    v_des_theta = 0.15
    v_des_norm = 0.8
    v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    v_norm = np.random.uniform(0.2, v_des_norm)
    datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    
    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 0.01  # small number that prevent impact

    # grab the sim and controller contexts for convenience
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    datapoint['stance'] = 0 if datapoint['stance'] == 'left' else 1

    #  First, align the timing with what's given by the initial condition
    t_init = datapoint['stance'] * t_s2s + t_ds + t_eps + datapoint['phase']
    context.SetTime(t_init)

    sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    sim_env.controller.SetSwingFootPositionAtLiftoff(
        context,
        datapoint['initial_swing_foot_pos']
    )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )

    Q = controller.params.Q
    R = controller.params.R

    simulator.reset_context(context)
    ALIPtmp = []
    FOOTSTEPtmp = []
    HMAPtmp = []
    DMAPtmp = []
    VDEStmp = []
    JOINTtmp = []
    GTJOINTtmp = []
    REWARDtmp = []
    terminate = False

    for i in range(1, 401): # 20 seconds

        if check_termination(sim_env, context):
            terminate = True
            break
        simulator.AdvanceTo(t_init + 0.05*i)

        if simulate_perception:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            gt_states = controller.get_input_port_by_name('gt_x_u_t').Eval(controller_context)
            joint_angle = states[:23]
            gt_joint_angle = gt_states[:23]
            #actuator = states[-10:]

            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)
            u = footstep[:2]

            '''
            Reward for Critic
            '''
            LQRreward = (x - xd).T @ Q @ (x - xd) + (u - ud).T @ R @ (u - ud)
            LQRreward = np.exp(-5*LQRreward)
            
            plant = sim_env.cassie_sim.get_plant()
            plant_context = plant.CreateDefaultContext()
            plant.SetPositionsAndVelocities(plant_context, gt_states[:45])

            fb_frame = plant.GetBodyByName("pelvis").body_frame()
            bf_velocity = fb_frame.CalcSpatialVelocity(
                plant_context, plant.world_frame(), fb_frame)
            bf_vel = bf_velocity.translational()
            bf_ang = bf_velocity.rotational()
            vdes = datapoint['desired_velocity']

            velocity_reward = np.exp(-5*np.linalg.norm(vdes[0] - bf_vel[0])) + np.exp(-np.linalg.norm(vdes[1] - bf_vel[1]))
            angular_reward = np.exp(-np.linalg.norm(bf_ang))
            
            reward = 0.5*LQRreward + 0.25*velocity_reward + 0.25*angular_reward

            # Depth map
            dmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            dmap = dmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Height map
            hmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map_query']
            ).get_value()

            hmap = hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Plot depth image
            if plot:
                grid_world = dmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                dmap_query.plot_surface(
                    "hmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        else:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            states = controller.get_input_port_by_name('xut').Eval(controller_context)
            joint_angle = states[:23]
            
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)

            # get heightmap query object
            hmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            # query for cropped height map at nominal footstep location
            hmap = hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )
            if plot:
                grid_world = hmap_query.calc_height_map_world_frame(
                    np.array([ud[0], ud[1], 0])
                )
                hmap_query.plot_surface(
                    "hmap", grid_world[0], grid_world[1],
                    grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        HMAPtmp.append(hmap)
        if simulate_perception:
            DMAPtmp.append(dmap)
        ALIPtmp.append(alip)
        VDEStmp.append(datapoint['desired_velocity'])
        JOINTtmp.append(joint_angle)
        GTJOINTtmp.append(gt_joint_angle)
        REWARDtmp.append(reward)
        FOOTSTEPtmp.append(footstep)

        time = context.get_time()-t_init

    RETURNtmp = REWARDtmp#calculate_returns(REWARDtmp)
    #print(RETURNtmp)
    return HMAPtmp, DMAPtmp, ALIPtmp, VDEStmp, JOINTtmp, GTJOINTtmp, RETURNtmp, FOOTSTEPtmp, terminate, time


def simulation_worker(sim_id, sim_params, checkpoint_path, perception_learning_base_folder):
    random_terrain = True
    HMAP = []
    DMAP = []
    ALIP = []
    VDES = []
    JOINT = []
    GTJOINT = []
    RETURN = []
    FOOTSTEP = []

    print(f"Starting simulation {sim_id}...")
    np.random.seed(sim_id + 32 * 10) # + 32 to change seed value
    for i in range(7):
        if random_terrain:
            rand = 2

            if rand == 1:
                rand = np.random.randint(1, 8)
                if rand in [1, 2, 3]:
                    terrain = 'params/stair_curriculum.yaml'
                elif rand in [4, 5, 6]:
                    terrain = 'params/wavy_terrain.yaml'
                else:
                    terrain = 'params/flat.yaml'
            
            else:
                rand = np.random.randint(1, 10)
                if rand == 1:
                    rand = np.random.randint(0, 500)
                    terrain = f'params/flat/flat_{rand}.yaml'
                else: # 2,3,4,5
                    rand = np.random.randint(0, 500)
                    terrain = f'params/stair/flat_stair_{rand}.yaml'
        else:
            terrain = 'params/stair_curriculum.yaml'
        
        sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
        sim_env, controller, diagram = build_diagram(sim_params, checkpoint_path, sim_params.simulate_perception)
        hmap, dmap, alip, vdes, joint, gtjoint, Return, footstep, terminate, time = run(sim_env, controller, diagram, sim_params.simulate_perception, plot=False)
        print(f"Simulation {sim_id}, Iteration {i}: Terminated in {time} seconds in {terrain}.")

        if not terminate:
            HMAP.extend(hmap)
            DMAP.extend(dmap)
            ALIP.extend(alip)
            VDES.extend(vdes)
            JOINT.extend(joint)
            GTJOINT.extend(gtjoint)
            RETURN.extend(Return)
            FOOTSTEP.extend(footstep)

        del sim_env, controller, diagram

    return HMAP, DMAP, ALIP, VDES, JOINT, GTJOINT, RETURN, FOOTSTEP


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')  # path of trained U-Net
    
    sim_params.simulate_perception = True

    print("Starting multiprocessing simulations...")

    num_processes = int(os.cpu_count() / 2)
    pool = multiprocessing.Pool(processes=num_processes)

    tasks = [(i, sim_params, checkpoint_path, perception_learning_base_folder) for i in range(num_processes)]
    results = pool.starmap(simulation_worker, tasks)

    HMAP = []
    DMAP = []
    ALIP = []
    VDES = []
    JOINT = []
    GTJOINT = []
    RETURN = []
    FOOTSTEP = []

    for result in results:
        HMAP.extend(result[0])
        DMAP.extend(result[1])
        ALIP.extend(result[2])
        VDES.extend(result[3])
        JOINT.extend(result[4])
        GTJOINT.extend(result[5])
        RETURN.extend(result[6])
        FOOTSTEP.extend(result[7])

    print(f"Number of collected datapoints is: {np.array(ALIP).shape[0]}")
    print(f"Number of collected datapoints is: {np.array(VDES).shape[0]}")
    print(f"Number of collected datapoints is: {np.array(JOINT).shape[0]}")
    print(f"Number of collected datapoints is: {np.array(GTJOINT).shape[0]}")
    
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/HMAP.npy', HMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/DMAP.npy', DMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/ALIP.npy', ALIP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/VDES.npy', VDES
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/JOINT.npy', JOINT
    )
    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/GTJOINT.npy', GTJOINT
    )

    print("Saving returns ...")

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/RETURN.npy', RETURN
    )

    del HMAP # Only for Denoising

    print("Saving actions and observations ...")

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/actions.npy', FOOTSTEP
    )

    DMAP = np.asarray(DMAP)
    ALIP = np.asarray(ALIP)
    VDES = np.asarray(VDES)
    JOINT = np.asarray(JOINT)
    GTJOINT = np.asarray(GTJOINT)
    DMAP = DMAP.reshape((DMAP.shape[0], -1))

    np.save(
        f'{perception_learning_base_folder}/tmp/data_collection'
        f'/observations.npy', np.concatenate((DMAP, ALIP, VDES, JOINT, GTJOINT), axis=1)
    )

if __name__ == '__main__':
    main()
