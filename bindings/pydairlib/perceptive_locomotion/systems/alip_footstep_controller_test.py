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

from pydairlib.perceptive_locomotion.systems.alip_mpfc import (
    AlipMPFC
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph
from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet

import numpy as np

# controller_type = 'mpfc'


controller_type = 'lqr'


def big_flat_polygon():
    poly = ConvexPolygon()
    poly.SetPlane(np.array([0., 0., 1.]), np.array([0., 0., 0.]))
    for n in [np.array([1., 0., 0.]), np.array([1., 0., 0.]),
              np.array([1., 0., 0.]), np.array([1., 0., 0.])]:
        poly.AddFace(n, 1000 * n)
        return ConvexPolygonSet([poly])


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.visualize = True
    sim_params.simulate_perception = True
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    builder = DiagramBuilder()

    controller = AlipFootstepLQR(controller_params) if (
        controller_type == 'lqr') else AlipMPFC(
        controller_params, sim_env.controller_plant
    )
    footstep_zoh = ZeroOrderHold(1.0 / 50.0, 3)
    builder.AddSystem(footstep_zoh)
    builder.AddSystem(sim_env)

    desired_velocity = ConstantVectorSource(np.array([0.1, 0]))
    foothold_source = ConstantValueSource(Value(big_flat_polygon()))

    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)
    builder.AddSystem(foothold_source)

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
        controller.get_input_port_by_name("alip_state")
    )
    if controller_type == 'mpfc':
        builder.Connect(
            sim_env.get_output_port_by_name('state'),
            controller.get_input_port_by_name('robot_state')
        )
        builder.Connect(
            foothold_source.get_output_port(),
            controller.get_input_port_by_name('convex_footholds')
        )

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_lqr')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    simulator.AdvanceTo(np.inf)


if __name__ == "__main__":
    main()
