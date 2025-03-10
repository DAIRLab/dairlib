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

from pydairlib.perceptive_locomotion import PerceptiveFullSimDiagram

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion import ConvexTerrainDecompositionSystem

from pydairlib.systems import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import OutputVector

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria

base_folder = "bindings/pydairlib/perceptive_locomotion/sim_experiments/"

params = "bindings/pydairlib/perceptive_locomotion/sim_experiments/sim_opts_beam.yaml"
terrain = "bindings/pydairlib/perceptive_locomotion/sim_experiments/terrains/perceptive_sine.yaml"
gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"


def main():
    terrain_segmentation = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    terrain_segmentation.MakeDrivenByStandaloneSimulator(1.0/30.0)

    convex_decomposition = ConvexTerrainDecompositionSystem()
    sim_diagram = PerceptiveFullSimDiagram(gains, terrain, params)

    builder = DiagramBuilder()
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)
    builder.AddSystem(sim_diagram)

    builder.Connect(
        sim_diagram.get_output_port_grid_map(),
        terrain_segmentation.get_input_port()
    )
    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
    )
    builder.Connect(
        terrain_segmentation.get_output_port(),
        sim_diagram.get_input_port_grid_map()
    )
    builder.Connect(
        convex_decomposition.get_output_port(),
        sim_diagram.get_input_port_footholds()
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)

    input("\n\n-- Press Enter to start the simulation --")

    simulator.AdvanceTo(30.0)


if __name__ == '__main__':
    main()
