"""
    This file is for a proof of concept example of residual LQR learning
    where we just look at one initial state and one heightmap and see what the
    landscape of the residual function looks like.
"""
import os
import sys

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
)

from pydairlib.perceptive_locomotion.systems.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    perception_learning_base_folder
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np


def run_experiment():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = os.path.join(
        perception_learning_base_folder,
        'params/alip_lqr_cost_experiment_terrain.yaml'
    )
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    controller = AlipFootstepLQR(controller_params)

    builder = DiagramBuilder()
    builder.AddSystem(sim_env)

    desired_velocity = ConstantVectorSource(np.array([0.0, 0.0]))
    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)

    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port_by_name("desired_velocity")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("fsm"),
        controller.get_input_port_by_name("fsm")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("time_until_switch"),
        controller.get_input_port_by_name("time_until_switch")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("alip_state"),
        controller.get_input_port_by_name("state")
    )

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    context.SetTime(0.2)
    q, v = sim_env.cassie_sim.SetPlantInitialConditionFromIK(
        diagram,
        context,
        np.zeros((3,)),
        0.15,
        1.0
    )

    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    controller_context = controller.GetMyMutableContextFromRoot(context)

    ud = controller.get_output_port_by_name('lqr_reference').Eval(
        controller_context
    )[-2:]
    heightmap_center = np.zeros((3,))
    heightmap_center[:2] = ud
    hmap = sim_env.get_heightmap(sim_context, center=heightmap_center)
    print(heightmap_center)
    Ts2s = controller_params.single_stance_duration + \
           controller_params.double_stance_duration

    def residual_datapoint(footstep_command: np.ndarray) -> float:
        context = diagram.CreateDefaultContext()
        context.SetTime(0.11)
        sim_env.cassie_sim.SetPlantInitialCondition(diagram, context, q, v)
        simulator.reset_context(context)
        simulator.Initialize()

        controller_context = controller.GetMyMutableContextFromRoot(context)
        sim_context = sim_env.GetMyMutableContextFromRoot(context)

        V_kp1 = controller.get_next_value_estimate_for_footstep(
            footstep_command, controller_context
        )

        sim_env.get_input_port_by_name("footstep_command").FixValue(
            context=sim_context,
            value=footstep_command
        )

        simulator.AdvanceTo(Ts2s + controller_params.double_stance_duration)
        while context.get_time() < 2 * Ts2s - 0.03:
            command = controller.get_output_port_by_name(
                'footstep_command'
            ).Eval(controller_context).ravel()
            command[2] = sim_env.query_heightmap(sim_context, command)
            sim_env.get_input_port_by_name("footstep_command").FixValue(
                context=sim_context,
                value=command
            )
            simulator.AdvanceTo(context.get_time() + 1e-2)
        V_k = controller.get_value_estimate(controller_context)
        return V_k - V_kp1

    residual = np.zeros(hmap.shape[1:])
    for i in range(hmap.shape[1]):
        for j in range(hmap.shape[2]):
            residual[i, j] = residual_datapoint(footstep_command=hmap[:, i, j])

    return hmap, residual


def compare_contours(hmap, obstacle_data, flat_data):
    import matplotlib.pyplot as plt

    obstacle_data[2] /= np.max(np.abs(obstacle_data[2]))
    obstacle_data[2] = np.abs(obstacle_data[2])
    flat_data[2] /= np.max(np.abs(obstacle_data[2]))
    flat_data[2] = np.abs(flat_data[2])

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
    fig.suptitle('LQR Cost to Go Experiment')
    ax1.contourf(hmap[0], hmap[1], hmap[2], levels=20)
    ax1.set_title('Height Map')
    ax2.contourf(
        obstacle_data[0], obstacle_data[1], obstacle_data[2], levels=20
    )
    ax2.set_title('Obstacle Residual')
    ax3.contourf(flat_data[0], flat_data[1], flat_data[2], levels=20)
    ax3.set_title('Control Residual')
    for ax in [ax1, ax2, ax3]:
        ax.set_xlabel('x')
        ax.set_ylabel('y')

    plt.show()


def plot_results(data):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(data[0], data[1], data[2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('LQR Cost to go residual')
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'contours':
        hmap = np.load(
            os.path.join(
                perception_learning_base_folder,
                'tmp/height_map_obstacle.npy'
            )
        )
        obs_data = np.load(
            os.path.join(
                perception_learning_base_folder,
                'tmp/residual_test_obstacle.npy'
            )
        )
        flat_data = np.load(
            os.path.join(
                perception_learning_base_folder,
                'tmp/residual_test_flat.npy'
            )
        )
        compare_contours(hmap, obs_data, flat_data)

    elif len(sys.argv) > 1:
        fname = sys.argv[1]
        data = np.load(fname)
        plot_results(data)
    else:
        xyz, residual = run_experiment()
        np.save(
            f'{perception_learning_base_folder}/tmp/height_map_obstacle'
            f'.npy', xyz
        )
        xyz[-1] = residual
        np.save(
            f'{perception_learning_base_folder}/tmp'
            f'/residual_test_obstacle.npy', xyz
        )
        plot_results(xyz)
