import pdb
import os
import numpy as np
from tqdm import tqdm
from typing import Dict, Tuple
import argparse
import multiprocessing

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
    AlipFootstepNNLQR
)
from pydairlib.perceptive_locomotion.perception_learning.terrain_utils import (
    make_stairs, random_stairs
)

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
                  checkpoint_path) \
        -> Tuple[CassieFootstepControllerEnvironment, AlipFootstepNNLQR, Diagram, LogVectorOutput]:
    builder = DiagramBuilder()
    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller = sim_env.AddToBuilderWithFootstepController(
        builder, AlipFootstepNNLQR, model_path=checkpoint_path
    )
    cost_system = CumulativeCost.AddToBuilder(builder, sim_env, controller)

    footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)
    cost_zoh = ZeroOrderHold(0.05, 1) # only need to log the cost at sparse intervals, since it updates once per stride
    cost_logger = VectorLogSink(1)

    builder.AddSystem(footstep_zoh)
    builder.AddSystem(cost_zoh)
    builder.AddSystem(cost_logger)

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
    builder.Connect(
        cost_system.get_output_port(),
        cost_zoh.get_input_port()
    )
    builder.Connect(
        cost_zoh.get_output_port(),
        cost_logger.get_input_port()
    )

    diagram = builder.Build()
    #DrawAndSaveDiagramGraph(diagram, '../AlipNNLQRTest')
    return sim_env, controller, cost_logger, diagram

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

def run(sim_params: CassieFootstepControllerEnvironmentOptions, i):

    #checkpoint_path = os.path.join(
    #    perception_learning_base_folder, 'tmp/copper-cherry-3.pth')
    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')

    sim_env, controller, cost_logger, diagram = build_diagram(sim_params, checkpoint_path)
    simulator = Simulator(diagram)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/index/initial_conditions_2.npz'
        )
    )
    #datapoint = ic_generator.random()
    #v_des_theta = np.pi / 6
    #v_des_norm = 1.0
    #v_theta = np.random.uniform(-v_des_theta, v_des_theta)
    #v_norm = np.random.uniform(0.2, v_des_norm)
    #datapoint['desired_velocity'] = np.array([v_norm * np.cos(v_theta), v_norm * np.sin(v_theta)]).flatten()
    
    datapoint = ic_generator.choose(i)
    #v_des_norm = 1.0
    #v_norm = np.random.uniform(0.2, v_des_norm)
    #datapoint['desired_velocity'] = np.array([v_norm, 0])
    datapoint['desired_velocity'] = np.array([0.4, 0])
    context = diagram.CreateDefaultContext()

    # timing aliases
    t_ss = controller.params.single_stance_duration
    t_ds = controller.params.double_stance_duration
    t_s2s = t_ss + t_ds
    t_eps = 0.01  # small number that prevent impact

    # grab the sim and controller contexts for convenience
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)
    cost_system_context = cost_logger.GetMyMutableContextFromRoot(context)
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

    Terminate = False
    time = 0
    simulator.reset_context(context)
    for i in range(1, 100):
        if check_termination(sim_env, context):
            print(time)
            print('terminate')
            Terminate = True
            break
        simulator.AdvanceTo(t_init + 0.05*i)
        time = context.get_time()-t_init
    cost_log = cost_logger.FindLog(context).data()
    return cost_log, t_init, Terminate, time


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = os.path.join(
        perception_learning_base_folder, 'params/stair_curriculum.yaml'#'params/stair_curriculum.yaml'#'params/wavy_test.yaml' #'params/wavy_terrain.yaml' # 'params/flat.yaml'
    )
    sim_params.visualize = False
    cost_list = []
    t_list = []
    init = []
    for _ in range(400):
        i = np.random.randint(0, 157870)
        print(i)
        cost, t_init, terminate, time = run(sim_params, i)
        if time > 4.5:
            #print("append")
            init.append(i)
        #print(cost)
        #cost_list.append(cost)
        #t_list.append(t_init)
    #cost = np.array(cost_list)
    init = np.array(init)
    np.save(
        f'{perception_learning_base_folder}/tmp/index'
        f'/index999.npy', init
    )
    #np.save(
    #    f'{perception_learning_base_folder}/tmp'
    #    f'/accumulated_cost_with_time.npy', cost
    #)


if __name__ == '__main__':
    main()
