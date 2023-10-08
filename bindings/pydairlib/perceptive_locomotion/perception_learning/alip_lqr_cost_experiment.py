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

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQROptions,
    AlipFootstepLQR
)

from pydairlib.perceptive_locomotion.perception_learning. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
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
    q, v = sim_env.cassie_sim.SetPlantInitialConditionFromIK(
        diagram,
        context,
        np.zeros((3,)),
        0.15,
        1.0
    )

    simulator.reset_context(context)
    simulator.Initialize()

    controller_context = controller.GetMyMutableContextFromRoot(context)
    sim_context = sim_env.GetMyMutableContextFromRoot(context)

    hmap = sim_env.get_heightmap(sim_context)
    sim_env.get_input_port_by_name("footstep_command").FixValue(
        context=sim_context,
        value=hmap[:, 0, 0]
    )

    Ts2s = controller_params.single_stance_duration + \
           controller_params.double_stance_duration

    simulator.AdvanceTo(Ts2s + controller_params.double_stance_duration)

    while context.get_time() < 2 * Ts2s - 2e-2:
        command = controller.get_output_port_by_name.Eval(controller_context)
        sim_env.get_input_port_by_name("footstep_command").FixValue(
            context=sim_context,
            value=command
        )
        simulator.AdvanceTo(context.get_time() + 1e-2)


if __name__ == '__main__':
    main()
