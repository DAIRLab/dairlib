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
from pydairlib.perceptive_locomotion.perception_learning.terrain_utils import (
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
from pydairlib.perceptive_locomotion.perception_learning.true_cost_system import (
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
        builder, AlipFootstepNNLQR, model_path=checkpoint_path
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
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )
    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../AlipNNLQR_DataCollection')
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

def run(sim_env, controller, diagram, simulate_perception=False):
    
    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            #'tmp/index/initial_conditions_2.npz'
            'tmp/initial_conditions_2.npz'
        )
    )

    #datapoint = ic_generator.random()
    #v_des_theta = np.pi / 6
    #v_des_norm = 1.0
    #v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    #v_norm = np.random.uniform(0.2, v_des_norm)
    #datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()
    
    datapoint = ic_generator.random()
    #datapoint = ic_generator.choose(35)
    v_des_norm = 0.8
    v_norm = np.random.uniform(0.2, v_des_norm)
    #coeff = np.random.uniform(0., 0.1)
    #datapoint['desired_velocity'] = np.array([v_norm, 0])
    datapoint['desired_velocity'] = np.array([0.4, 0])

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

    simulator.reset_context(context)
    ALIPtmp = []
    FOOTSTEPtmp = []
    HMAPtmp = []
    DMAPtmp = []
    VDEStmp = []
    terminate = False
    for i in range(1, 500):
        if check_termination(sim_env, context):
            print('terminate')
            terminate = True
            break
        simulator.AdvanceTo(t_init + 0.05*i)
        if simulate_perception:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)
            
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
            #residual_grid_world = hmap_query.calc_height_map_world_frame(
            #    np.array([ud[0], ud[1], 0])
            #)
            #hmap_query.plot_surface(
            #    "residual", residual_grid_world[0], residual_grid_world[1],
            #    residual_grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))

        else:
            footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
            alip = controller.get_output_port_by_name('x_xd').Eval(controller_context)
            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
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
        #print(hmap.shape)
        ALIPtmp.append(alip)
        FOOTSTEPtmp.append(footstep)
        HMAPtmp.append(hmap)
        if simulate_perception:
            DMAPtmp.append(dmap)
        VDEStmp.append(datapoint['desired_velocity'])
        time = context.get_time()-t_init
    return ALIPtmp, FOOTSTEPtmp, HMAPtmp, DMAPtmp, VDEStmp, terminate, time


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    #checkpoint_path = os.path.join(
    #    perception_learning_base_folder, 'tmp/copper-cherry-3.pth')
    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')
    sim_params.visualize = True
    sim_params.simulate_perception = False
    ALIP = []
    FOOTSTEP = []
    HMAP = []
    DMAP = []
    VDES = []
    terrain = 'params/stair_curriculum.yaml'
    #terrain = 'params/wavy_terrain.yaml'
    #terrain = 'params/flat_stair.yaml'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
    sim_env, controller, diagram = build_diagram(sim_params, checkpoint_path, sim_params.simulate_perception)

    for i in range(100):
        print(i)
        alip, footstep, hmap, dmap, vdes, terminate, time = run(sim_env, controller, diagram, sim_params.simulate_perception)
        print(time)
        if time > 20:
            ALIP.extend(alip)
            FOOTSTEP.extend(footstep)
            HMAP.extend(hmap)
            DMAP.extend(dmap)
            VDES.extend(vdes)

    print(np.array(ALIP).shape)
    print(np.array(FOOTSTEP).shape)
    print(np.array(HMAP).shape)
    print(np.array(DMAP).shape)
    print(np.array(VDES).shape)

    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/ALIP1.npy', ALIP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/FOOTSTEP1.npy', FOOTSTEP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/HMAP1.npy', HMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/DMAP1.npy', DMAP
    )
    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/VDES1.npy', VDES
    )

if __name__ == '__main__':
    main()
