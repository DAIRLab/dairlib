import os
import tempfile
import numpy as np
from copy import deepcopy
from pydairlib.id_mpc.bench.utils import random_stepping_stones

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
from pydairlib.perceptive_locomotion.diagrams import FullSimDiagram


def trial(trial_idx: int, gains: str, solver_options: str, logfile: str = None):
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    sim_params = os.path.join(base_folder, 'alip_bench_params.yaml')
    terrain_file = tempfile.NamedTemporaryFile()

    random_stepping_stones(
        seed=trial_idx,
        min_sidelength=0.35,
        savefile=terrain_file.name
    )
    
    sim_diagram = FullSimDiagram(
        gains,
        solver_options,
        terrain_file.name,
        sim_params
    )

    builder = DiagramBuilder()
    builder.AddSystem(sim_diagram)
    
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)
    try:
        simulator.AdvanceTo(15.0)
    except Exception as e:
        print(e)
        terrain_file.close()
        return False

    if logfile:
        sim_diagram.SaveLcmLog(logfile)

    terrain_file.close()

    return True


def run_experiment():
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    success_counts = {
        'admm': {},
        'miqp': {}
    }
    for method in ['admm', 'miqp']:
        for horizon in [3, 4, 5]:
            solver_options = 'gurobi_options_planner' if method == 'miqp' else\
                             'ncqp_params'
            solver_options_file = os.path.join(
                base_folder, f'gains/{solver_options}.yaml'
            )
            gains_file = os.path.join(
                base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
            )
            success_count = 0
            for idx in range(50):
                success = trial(
                    idx,
                    gains_file,
                    solver_options_file
                )
                if success:
                    success_count += 1
            success_counts[method][horizon] = success_count

    np.savez(
        '../alip_bench_results',
        success_counts=success_counts
    )
    print(success_counts)

if __name__ == '__main__':
    run_experiment()
