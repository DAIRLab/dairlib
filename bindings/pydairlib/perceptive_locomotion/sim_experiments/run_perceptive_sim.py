import os
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

terrains = {
    'sine': 'terrains/perceptive_sine.yaml',
    'beam': 'terrains/perceptive_beam.yaml',
    'stairs': 'terrains/perceptive_stairs.yaml'
}

log_folder = "../sim_experiment_logs"

gains = os.path.join(base_folder, "gains/mpfc_gains_default.yaml")


def select_terrain_and_log_file():
    choice = input('pick terrain type:\n(1) beam\n(2) stairs\n(3) sine\n\nSelection: ')
    choice = choice.strip()
    if choice == '1':
        return os.path.join(base_folder, terrains['beam']), \
            os.path.join(base_folder, 'sim_opts_beam.yaml'), \
            os.path.join(log_folder, 'perceptive_beam'),
        
    if choice == '2':
        return os.path.join(base_folder, terrains['stairs']), \
            os.path.join(base_folder, 'sim_opts_stairs.yaml'), \
            os.path.join(log_folder, 'perceptive_stairs')
    if choice == '3':
        return os.path.join(base_folder, terrains['sine']), \
            os.path.join(base_folder, 'sim_opts_beam.yaml'), \
            os.path.join(log_folder, 'perceptive_sine')
    
    raise RuntimeError("invalid or no terrain specified")


def main():
    terrain, params, logfile = select_terrain_and_log_file()
    terrain_segmentation = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    terrain_segmentation.opencv_inpaint = False
    terrain_segmentation.erosion_kernel_length = 0.1
    terrain_segmentation.safety_hysteresis = 0.4

    terrain_segmentation.MakeDrivenByStandaloneSimulator(1.0/30.0)
    
    convex_decomposition = ConvexTerrainDecompositionSystem()
    sim_diagram = PerceptiveFullSimDiagram(gains, terrain, params, "")

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

    simulator.AdvanceTo(20.0)
    
    sim_diagram.SaveLcmLog(logfile)


if __name__ == '__main__':
    main()
