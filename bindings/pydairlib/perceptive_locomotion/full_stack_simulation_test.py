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

from pydairlib.perceptive_locomotion.diagrams import (
    AlipMPFCDiagram,
    MpfcOscDiagramInputType,
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem


from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.terrain = 'bindings/pydairlib/perceptive_locomotion/params/stair_curriculum.yaml'
    sim_params.visualize = True
    sim_params.simulate_perception = True
    sim_params.controller_input_type = MpfcOscDiagramInputType.kLcmtAlipMpcOutput

    builder = DiagramBuilder()

    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller = AlipMPFCDiagram(
        sim_env.controller_plant, sim_params.mpfc_gains_yaml, 1/30
    )
    desired_velocity = ConstantVectorSource(np.array([0.5, 0]))
    terrain_segmentation = TerrainSegmentationSystem()
    convex_decomposition = ConvexTerrainDecompositionSystem()

    builder.AddSystem(sim_env)
    builder.AddSystem(controller)
    builder.AddSystem(desired_velocity)
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)

    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
    )
    builder.Connect(
        convex_decomposition.get_output_port(),
        controller.get_input_port_footholds()
    )
    builder.Connect(
        controller.get_output_port(), sim_env.get_input_port()
    )
    builder.Connect(
        desired_velocity.get_output_port(),
        controller.get_input_port_vdes()
    )
    builder.Connect(
        sim_env.get_output_port_by_name("height_map"),
        terrain_segmentation.get_input_port()
    )
    builder.Connect(
        sim_env.get_output_port_by_name("lcmt_robot_output"),
        controller.get_input_port_state()
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
        diagram.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())
        simulator.AdvanceTo(tnext)
        tnext += 0.01


if __name__ == '__main__':
    main()