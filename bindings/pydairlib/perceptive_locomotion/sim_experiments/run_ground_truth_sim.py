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
from pydairlib.systems.framework import OutputVector

base_folder = "bindings/pydairlib/perceptive_locomotion/sim_experiments/"

terrains = {
    'beam': 'terrains/beam.yaml',
    'stairs': 'terrains/stairs.yaml',
    'heights': 'terrains/heights.yaml'
}

log_folder = "../sim_experiment_logs"

gains = os.path.join(base_folder, "gains/mpfc_gains_default.yaml")
solver_options = os.path.join(base_folder, 'solver_options/gurobi_options_planner.yaml')
gains_admm = os.path.join(base_folder, 'gains/mpfc_gains_comparison_to_id_mpc.yaml')
solver_options_admm = os.path.join(base_folder, 'solver_options/ncqp_params.yaml')


def select_terrain_and_log_file():
    choice = input('pick terrain type:\n(1) beam\n(2) stairs\n\nSelection: ')
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
        return os.path.join(base_folder, terrains['heights']), \
            os.path.join(base_folder, 'sim_opts_heights.yaml'), \
            os.path.join(log_folder, 'heights')
    
    raise RuntimeError("invalid or no terrain specified")


def main():
    terrain, params, logfile = select_terrain_and_log_file()
    
    sim_diagram = FullSimDiagram(
        gains, solver_options, terrain, params, True)

    builder = DiagramBuilder()
    builder.AddSystem(sim_diagram)
    
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)

    input("\n\n-- Press Enter to start the simulation --")

    try:
        simulator.AdvanceTo(40.0)
    finally:
        sim_diagram.SaveLcmLog(logfile)


if __name__ == '__main__':
    main()
