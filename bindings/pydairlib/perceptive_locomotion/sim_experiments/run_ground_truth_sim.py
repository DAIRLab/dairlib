import numpy as np

from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    DiagramBuilder,
    LcmPublisherSystem,
    LcmSubscriberSystem,
    TriggerType,
)

from pydrake.common.value import AbstractValue

from pydairlib.perceptive_locomotion import FullSimDiagram

from pydairlib.systems import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import OutputVector

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria

params = "bindings/pydairlib/perceptive_locomotion/sim_experiments/sim_opts_beam.yaml"
terrain = "bindings/pydairlib/perceptive_locomotion/sim_experiments/terrains/beam.yaml"
gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"


def main():
    sim_diagram = FullSimDiagram(gains, terrain, params)

    builder = DiagramBuilder()
    builder.AddSystem(sim_diagram)
    
    diagram = builder.Build()
    DrawAndSaveDiagramGraph(
        diagram,
        '../full_sim_diagram'
    )

    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)
    # simulator.set_target_realtime_rate(1.0)

    input("\n\n-- Press Enter to start the simulation --")

    simulator.AdvanceTo(20.0)


if __name__ == '__main__':
    main()
