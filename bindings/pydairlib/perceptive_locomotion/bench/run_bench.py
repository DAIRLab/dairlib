import os

import numpy as np

from pydairlib.perceptive_locomotion.bench.bench_harness import (
    BenchEnvOptions,
    BenchHarness
)

from pydairlib.perceptive_locomotion.diagrams import (
    AlipMPFCDiagram
)

from pydairlib.systems import DrawAndSaveDiagramGraph

import pydairlib.perceptive_locomotion.terrain_segmentation as terrain_seg

from pydrake.systems.all import (
    State,
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    LeafSystem,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
)

from pydrake.geometry import Meshcat


def add_mpc_perception_deps(builder, env, mpc):
    segmentation = builder.AddSystem(
        terrain_seg.TerrainSegmentationSystem({
                'curvature_criterion': terrain_seg.curvature_criterion,
                'variance_criterion': terrain_seg.variance_criterion,
            },
        update_period=1.0/30.0)
    )
    decomposition = builder.AddSystem(
        terrain_seg.ConvexTerrainDecompositionSystem()
    )
    builder.Connect(
        env.get_output_port_by_name('elevation_map'),
        segmentation.get_input_port(),
    )
    builder.Connect(
        segmentation.get_output_port(),
        decomposition.get_input_port()
    )
    builder.Connect(
        decomposition.get_output_port(),
        mpc.get_input_port_footholds()
    )
    builder.Connect(
        env.get_output_port_by_name('lcmt_robot_output'),
        mpc.get_input_port_state()
    )


def main():
    opts = BenchEnvOptions()
    opts.visualize = True
    opts.meshcat = Meshcat()

    env = BenchHarness(opts)
    controller = AlipMPFCDiagram(env.controller_plant, opts.mpfc_gains_yaml, 30)

    builder = DiagramBuilder()
    builder.AddSystem(env)
    builder.AddSystem(controller)

    add_mpc_perception_deps(builder, env, controller)

    builder.Connect(
        controller.get_output_port_mpc_output(),
        env.get_input_port_by_name("command")
    )

    vdes_source = builder.AddSystem(ConstantVectorSource(np.array([0.0, 0.0])))

    builder.Connect(
        vdes_source.get_output_port(),
        controller.get_input_port_vdes()
    )

    diagram = builder.Build()

    DrawAndSaveDiagramGraph(diagram, 'bench')
    simulator = Simulator(diagram)

    # repeat the simulation a few times
    q = np.array([1, 0, 0, 0, 0, 0, 0.95, -0.0320918, 0, 0.539399, -1.31373,
    -0.0410844, 1.61932, -0.0301574, -1.67739, 0.0320918, 0, 0.539399,
    -1.31373, -0.0404818, 1.61925, -0.0310551, -1.6785])
    v = np.zeros((22,))

    context = diagram.CreateDefaultContext()
    env.initialize_state(context, diagram, q, v)
    simulator.reset_context(context)
    simulator.Initialize()
    input('waiting')
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(10.0)


if __name__ == '__main__':
    main()
