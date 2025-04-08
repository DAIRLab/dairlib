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

from pydairlib.perceptive_locomotion import FullSimDiagram

from pydairlib.systems import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import OutputVector

base_folder = "bindings/pydairlib/perceptive_locomotion/sim_experiments/"

terrains = {
    'beam': 'terrains/beam.yaml',
    'stairs': 'terrains/stairs.yaml',
    'stones': 'terrains/stones_10.yaml',
}

log_folder = "../sim_experiment_logs"

gains = os.path.join(base_folder, "gains/mpfc_gains_default.yaml")


def select_terrain_and_log_file():
    choice = input('pick terrain type:\n(1) beam\n(2) stairs\n(3) stones\n\nSelection: ')
    choice = choice.strip()
    if choice == '1':
        return os.path.join(base_folder, terrains['beam']), \
            os.path.join(base_folder, 'sim_opts_beam.yaml'), \
            os.path.join(log_folder, 'beam')
    if choice == '2':
        return os.path.join(base_folder, terrains['stairs']), \
            os.path.join(base_folder, 'sim_opts_stairs.yaml'), \
            os.path.join(log_folder, 'stairs')
    if choice == '3':
        return os.path.join(base_folder, terrains['stones']), \
            os.path.join(base_folder, 'sim_opts_stones.yaml'), \
            os.path.join(log_folder, 'stones')
    
    raise RuntimeError("invalid or no terrain specified")


def main():
    terrain, params, logfile = select_terrain_and_log_file()
    
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

    simulator.AdvanceTo(25.0)
    
    sim_diagram.SaveLcmLog(logfile)


if __name__ == '__main__':
    main()
