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

from pydairlib.perceptive_locomotion.perception_learning. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    perception_learning_base_folder,
    InitialConditionsServer
)
from pydairlib.perceptive_locomotion.perception_learning.true_cost_system import (
    CumulativeCost)

# Can use DrawAndSaveDiagramGraph for debugging if necessary
from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph


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
    DrawAndSaveDiagramGraph(diagram, '../AlipNNLQRTest')
    return sim_env, controller, cost_logger, diagram


def run(sim_params: CassieFootstepControllerEnvironmentOptions, i):

    checkpoint_path = os.path.join(
        perception_learning_base_folder, 'tmp/cosmic-plasma-149.pth')

    sim_env, controller, cost_logger, diagram = build_diagram(sim_params, checkpoint_path)
    simulator = Simulator(diagram)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/initial_conditions_2.npz'
        )
    )
    # datapoint = ic_generator.random()
    datapoint = ic_generator.choose(i)
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

    simulator.reset_context(context)
    simulator.AdvanceTo(t_init + 5)

    cost_log = cost_logger.FindLog(context).data()
    return cost_log, t_init


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = os.path.join(
        perception_learning_base_folder, 'params/stair_curriculum.yaml'
    )
    sim_params.visualize = True
    cost_list = []
    t_list = []
    for i in range(10):
        cost, t_init = run(sim_params, i)
        cost_list.append(cost)
        t_list.append(t_init)
    cost = np.array(cost_list)
    np.save(
        f'{perception_learning_base_folder}/tmp'
        f'/accumulated_cost_with_time.npy', cost
    )


if __name__ == '__main__':
    main()
