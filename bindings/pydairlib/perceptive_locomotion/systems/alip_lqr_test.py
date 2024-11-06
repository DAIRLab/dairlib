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

import os
import numpy as np
from grid_map import GridMap
from pydairlib.perceptive_locomotion import vision_utils
from pydrake.geometry import Meshcat

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/flat.yaml'
    sim_params.simulate_perception = True
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    builder = DiagramBuilder()

    controller = AlipFootstepLQR(controller_params)
    footstep_zoh = ZeroOrderHold(1.0 / 30.0, 3)
    builder.AddSystem(footstep_zoh)
    builder.AddSystem(sim_env)

    #desired_velocity = ConstantVectorSource(np.array([0.8, 0]))
    builder.AddSystem(controller)
    # builder.AddSystem(desired_velocity)

    # controller give footstep command to sim_environment (i.e. cassie)
    builder.Connect(
        controller.get_output_port_by_name('footstep_command'),
        footstep_zoh.get_input_port()
    )
    builder.Connect(
        footstep_zoh.get_output_port(),
        sim_env.get_input_port_by_name('footstep_command')
    )

    # external user assign desire velocity to controller
    # builder.Connect(
    #     desired_velocity.get_output_port(),
    #     controller.get_input_port_by_name("desired_velocity")
    # )

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

    diagram = builder.Build()
    # DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_context = sim_env.GetMyMutableContextFromRoot(context)

    ic_generator = InitialConditionsServer(
        fname=os.path.join(
            perception_learning_base_folder,
            'tmp/ic_new.npz'
        )
    )
    
    datapoint = ic_generator.random()
    datapoint['desired_velocity'] = np.array([0.8, 0.]).flatten()
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

    # sim_env.initialize_state(context, diagram, datapoint['q'], datapoint['v'])
    # sim_env.controller.SetSwingFootPositionAtLiftoff(
    #     context,
    #     datapoint['initial_swing_foot_pos']
    # )
    controller.get_input_port_by_name("desired_velocity").FixValue(
        context=controller_context,
        value=datapoint['desired_velocity']
    )

    sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    t_next = 0.05
    input("start")
    while t_next < np.inf:
        simulator.AdvanceTo(t_init + t_next)
        footstep = controller.get_output_port_by_name('footstep_command').Eval(controller_context)
        print(footstep)
        # elevation_map = sim_env.get_output_port_by_name('height_map').Eval(sim_context)
        # elevation_map.convertToDefaultStartIndex()
        # vision_utils.get_safe_terrain(elevation_map['elevation'], elevation_map.getResolution())
        t_next += 0.05
        print(context.get_time())


if __name__ == "__main__":
    main()