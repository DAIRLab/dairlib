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
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph
from pydairlib.geometry.convex_polygon import ConvexPolygon, ConvexPolygonSet

import numpy as np
from grid_map import GridMap

from pydrake.geometry import Rgba
# controller_type = 'mpfc'

controller_type = 'lqr'
elevation_mapping_yaml = 'bindings/pydairlib/perceptive_locomotion/params/elevation_mapping_params_simulation.yaml'
#mpfc_gains = 'bindings/pydairlib/perceptive_locomotion/params/mpfc_gains.yaml'

def big_flat_polygon():
    poly = ConvexPolygon()
    poly.SetPlane(np.array([0., 0., 1.]), np.array([0., 0., 0.]))
    for n in [np.array([1., 0., 0.]), np.array([1., 0., 0.]),
              np.array([1., 0., 0.]), np.array([1., 0., 0.])]:
        poly.AddFace(n, 1000 * n)
        return ConvexPolygonSet([poly])


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    #sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/stair_curriculum.yaml'
    sim_params.visualize = True
    sim_params.simulate_perception = True
    #sim_params.elevation_mapping_params_yaml = elevation_mapping_yaml
    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        #mpfc_gains,
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    builder = DiagramBuilder()

    controller = AlipFootstepLQR(controller_params, True) if (
        controller_type == 'lqr') else AlipMPFC(
        controller_params, sim_env.controller_plant
    )
    footstep_zoh = ZeroOrderHold(1.0 / 50.0, 3)
    builder.AddSystem(footstep_zoh)
    builder.AddSystem(sim_env)

    desired_velocity = ConstantVectorSource(np.array([0.3, 0]))
    #foothold_source = ConstantValueSource(Value(big_flat_polygon()))

    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)
    #builder.AddSystem(foothold_source)

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
    #if controller_type == 'mpfc':
    #    builder.Connect(
    #        sim_env.get_output_port_by_name('state'),
    #        controller.get_input_port_by_name('robot_state')
    #    )
    #    builder.Connect(
    #        foothold_source.get_output_port(),
    #        controller.get_input_port_by_name('convex_footholds')
    #    )

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../alip_footstep_controller_test')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    controller_context = controller.GetMyMutableContextFromRoot(context)
    sim_context = sim_env.GetMyMutableContextFromRoot(context)
    sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    t_next = 0.05
    while t_next < 20:
        simulator.AdvanceTo(t_next)

        hmap_query = controller.EvalAbstractInput(
            controller_context, controller.input_port_indices['height_map']
        ).get_value()
        #print(hmap_query)
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
            residual_grid_world[2], rgba = Rgba(0.678, 0.847, 0.902, 1.0))
        #print(hmap)
        #print(elevation_map)
        t_next += 0.05

if __name__ == "__main__":
    main()
