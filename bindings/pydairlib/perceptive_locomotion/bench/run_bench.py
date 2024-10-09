import os

import numpy as np

from pydairlib.perceptive_locomotion.bench.bench_harness import (
    BenchEnvOptions,
    BenchHarness
)

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


def main():
    opts = BenchEnvOptions()
    env = BenchHarness(opts)

    builder = DiagramBuilder()
    builder.AddSystem(env)
    footstep_source = ConstantVectorSource(np.array([0.1, -0.3, 0.0]))
    builder.AddSystem(footstep_source)

    builder.Connect(
        footstep_source.get_output_port(),
        env.get_input_port_by_name("command")
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # repeat the simulation a few times
    for _ in range(5):
        context = diagram.CreateDefaultContext()
        env.cassie_sim.SetPlantInitialConditionFromIK(
            diagram,
            context,
            np.zeros((3,)),
            0.15,
            1.0
        )
        simulator.reset_context(context)
        simulator.Initialize()
        simulator.set_target_realtime_rate(1.0)
        simulator.AdvanceTo(1.0)


if __name__ == '__main__':
    main()
