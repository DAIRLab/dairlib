from dairlib import lcmt_foothold_set, lcmt_robot_output

from pydrake.lcm import DrakeLcm

from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    TriggerType,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    LcmPublisherSystem,
    ZeroOrderHold,
)

from pydairlib.perceptive_locomotion.diagrams import (
    AlipMPFCDiagram,
    MpfcOscDiagramInputType,
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.geometry.convex_polygon import ConvexPolygonSender

from pydairlib.perceptive_locomotion.systems.alip_mpfc import AlipMPFC
from pydairlib.perceptive_locomotion.systems.alip_lqr import \
    AlipFootstepLQROptions

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem


from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np

import pydairlib.lcm  # needed for cpp serialization of lcm messages



def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/stair_curriculum.yaml'
    sim_params.visualize = True
    sim_params.simulate_perception = True

    builder = DiagramBuilder()

    sim_env = CassieFootstepControllerEnvironment(sim_params)

    controller_params = AlipFootstepLQROptions.calculate_default_options(
        sim_params.mpfc_gains_yaml,
        sim_env.controller_plant,
        sim_env.controller_plant.CreateDefaultContext(),
    )
    controller = AlipMPFC(
        controller_params, sim_env.controller_plant
    )
    desired_velocity = ConstantVectorSource(np.array([0.2, 0]))
    terrain_segmentation = TerrainSegmentationSystem()
    convex_decomposition = ConvexTerrainDecompositionSystem()
    foothold_sender = ConvexPolygonSender()
    lcm = DrakeLcm()
    foothold_publisher = LcmPublisherSystem.Make(
        channel="FOOTHOLDS_PROCESSED",
        lcm_type=lcmt_foothold_set,
        lcm=lcm,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )
    state_publisher = LcmPublisherSystem.Make(
        channel='CASSIE_STATE_SIMULATION',
        lcm_type=lcmt_robot_output,
        lcm=lcm,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )

    builder.AddSystem(sim_env)
    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)
    builder.AddSystem(foothold_sender)
    builder.AddSystem(foothold_publisher)
    builder.AddSystem(state_publisher)

    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
    )
    builder.Connect(
        convex_decomposition.get_output_port(),
        controller.get_input_port_by_name('convex_footholds')
    )
    builder.Connect(
        controller.get_output_port_by_name("footstep_command"),
        sim_env.get_input_port_by_name("footstep_command")
    )
    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port_by_name("desired_velocity")
    )
    builder.Connect(
        sim_env.get_output_port_by_name("height_map"),
        terrain_segmentation.get_input_port()
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
        controller.get_input_port_by_name("alip_state")
    )
    builder.Connect(
        convex_decomposition.get_output_port(),
        foothold_sender.get_input_port(),
    )
    builder.Connect(
        foothold_sender.get_output_port(),
        foothold_publisher.get_input_port()
    )
    builder.Connect(
        sim_env.get_output_port_by_name('lcmt_robot_output'),
        state_publisher.get_input_port()
    )
    builder.Connect(
        sim_env.get_output_port_by_name("state"),
        controller.get_input_port_by_name("robot_state")
    )
    diagram = builder.Build()
    DrawAndSaveDiagramGraph(diagram, '../full_stack_miqp_simulation')

    simulator = Simulator(diagram)
    context = diagram.CreateDefaultContext()
    sim_env.initialize_state(context, diagram)

    simulator.reset_context(context)
    simulator.set_target_realtime_rate(1.0)

    tnext = 0.01
    while tnext < np.inf:
        diagram.CalcForcedUnrestrictedUpdate(
            context, context.get_mutable_state()
        )
        diagram.ForcedPublish(context)
        simulator.AdvanceTo(tnext)
        tnext += 0.01


if __name__ == '__main__':
    main()