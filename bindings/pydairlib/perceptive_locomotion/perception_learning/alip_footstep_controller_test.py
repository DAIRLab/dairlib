# Even if all of these aren't explicitly used, they may be needed for python to
# recognize certain derived classes
from pydrake.systems.all import (
    Value,
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    ConstantValueSource,
    ZeroOrderHold,
)

from pydairlib.perceptive_locomotion.systems.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
    InitialConditionsServer
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np
import os
from grid_map import GridMap

from pydrake.geometry import Rgba

elevation_mapping_yaml = 'bindings/pydairlib/perceptive_locomotion/params/elevation_mapping_params_simulation.yaml'
perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/stair_curriculum.yaml'
    #sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/perception_learning/params/stair_curriculum.yaml'
    sim_params.visualize = True
    sim_params.simulate_perception = True
    #sim_params.elevation_mapping_params_yaml = elevation_mapping_yaml
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    builder = DiagramBuilder()

    controller = AlipFootstepLQR(controller_params, elevation = sim_params.simulate_perception)

    footstep_zoh = ZeroOrderHold(1.0 / 50.0, 3)
    builder.AddSystem(footstep_zoh)
    builder.AddSystem(sim_env)

    desired_velocity = ConstantVectorSource(np.array([0.4, 0]))

    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)

    # controller give footstep command to sim_environment (i.e. cassie)
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )

    # external user assign desired velocity to controller
    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port_by_name("desired_velocity")
    )

    # sim_env (cassie) returns state_feedback to controller
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
    if sim_params.simulate_perception:    
        builder.Connect(
            sim_env.get_output_port_by_name("height_map"),
            controller.get_input_port_by_name("height_map")
        )

    diagram = builder.Build()
    if sim_params.simulate_perception: 
        DrawAndSaveDiagramGraph(diagram, '../alip_footstep_controller_test_elevation')
    else:
        DrawAndSaveDiagramGraph(diagram, '../alip_footstep_controller_test')
    
    simulator = Simulator(diagram)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/initial_conditions_2.npz'
        )
    )

    datapoint = ic_generator.choose(0)
    #datapoint = ic_generator.random()
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

    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    t_next = 0.05
    grid = []
    input("Start..")
    while t_next < 20:
        simulator.AdvanceTo(t_init + t_next)
        if sim_params.simulate_perception:
            hmap_query = controller.EvalAbstractInput(
                controller_context, controller.input_port_indices['height_map']
            ).get_value()

            xd_ud = controller.get_output_port_by_name('lqr_reference').Eval(controller_context)
            xd = xd_ud[:4]
            ud = xd_ud[4:]
            x = controller.get_output_port_by_name('x').Eval(controller_context)
            hmap = hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            residual_grid_world = hmap_query.calc_height_map_world_frame(
                np.array([ud[0], ud[1], 0])
            )
            hmap_query.plot_surface(
                "residual", residual_grid_world[0], residual_grid_world[1],
                residual_grid_world[2], rgba = Rgba(0.5424, 0.6776, 0.7216, 1.0))
        
        grid.append(hmap)
        t_next += 0.05
    np.save('grid.npy', grid)

if __name__ == "__main__":
    main()
