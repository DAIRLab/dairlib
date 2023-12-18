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
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np
from grid_map import GridMap
from pydairlib.perceptive_locomotion import vision_utils


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/stair_curriculum.yaml'
    sim_params.simulate_perception = True
    sim_params.visualize = True
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

    desired_velocity = ConstantVectorSource(np.array([0.1, 0]))
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

    # external user assign desire velocity to controller
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

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_context = sim_env.GetMyMutableContextFromRoot(context)

    sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    t_next = 0.05
    while t_next < np.inf:
        simulator.AdvanceTo(t_next)
        elevation_map = sim_env.get_output_port_by_name('height_map').Eval(sim_context)
        elevation_map.convertToDefaultStartIndex()
        vision_utils.get_safe_terrain(elevation_map['elevation'], elevation_map.getResolution())
        t_next += 0.05


if __name__ == "__main__":
    main()
