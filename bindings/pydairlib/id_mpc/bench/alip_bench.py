import os
import tempfile
import numpy as np
from copy import deepcopy
from functools import partial
from concurrent.futures import ProcessPoolExecutor
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

    print(trial_idx)

    random_stepping_stones(
        seed=trial_idx,
        min_sidelength=0.35,
        savefile=terrain_file.name
    )
    
    sim_diagram = FullSimDiagram(
        gains,
        solver_options,
        terrain_file.name,
        sim_params,
        True
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
        simulator.AdvanceTo(20.0)
    except Exception as e:
        print(e)
        terrain_file.close()
        if logfile:
            sim_diagram.SaveLcmLog(logfile + "_" + str(trial_idx))
        return False

    if logfile:
        sim_diagram.SaveLcmLog(logfile + "_" + str(trial_idx))

    terrain_file.close()

    return True


def run_miqp_vs_admm_experiment():
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    success_counts = {
        'admm': {},
        'miqp': {}
    }

    horizons = {
        'admm': [3, 4, 5, 6],
        'miqp': [3, 4, 5, 6]
    }
    for method in ['admm', 'miqp']:
        for horizon in horizons[method]:
            solver_options = 'gurobi_options_planner' if method == 'miqp' else\
                             'ncqp_params'
            solver_options_file = os.path.join(
                base_folder, f'gains/{solver_options}.yaml'
            )
            gains_file = os.path.join(
                base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
            )
            success_count = 0

            worker_fn = partial(
                trial,
                gains=gains_file,
                solver_options=solver_options_file,
            )
            with ProcessPoolExecutor(max_workers=8) as exec:
                executor = exec
                for success in executor.map(worker_fn, range(100)):
                    if success:
                        success_count += 1
            success_counts[method][horizon] = success_count
            print(success_counts)

    np.savez(
        '../alip_bench_results_increased_footstep_cost',
        success_counts=success_counts
    )
    print(success_counts)


def run_admm_params_grid_search():
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    successes = {}

    iterations = [0, 1, 2, 3, 4, 5]
    rho_exps = [-2, -1, 0, 1, 2]

    for iteration in iterations:
        successes[iteration] = {}
        for rho_exp in rho_exps:
            method = 'admm'
            horizon = 4
            solver_options = f'ncqp_params_{rho_exp}_{iteration}'
            solver_options_file = os.path.join(
                base_folder, f'admm_grid_search_gains/{solver_options}.yaml'
            )
            gains_file = os.path.join(
                base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
            )
            success_count = 0

            worker_fn = partial(
                trial,
                gains=gains_file,
                solver_options=solver_options_file,
            )
            with ProcessPoolExecutor(max_workers=8) as exec:
                for success in exec.map(worker_fn, range(100)):
                    if success:
                        success_count += 1
            successes[iteration][rho_exp] = success_count
            print(successes)

    np.savez(
        '../alip_bench_grid_search_results_4_step',
        success_counts=successes
    )
    print(successes)


def make_log(logfile: str, idx: int, method: str, horizon: int):
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    solver_options = 'gurobi_options_planner' if method == 'miqp' else \
        'ncqp_params'
    solver_options_file = os.path.join(
        base_folder, f'gains/{solver_options}.yaml'
    )
    gains_file = os.path.join(
        base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
    )
    success = trial(idx, gains_file, solver_options_file, logfile)


def make_log_driver():
    idx = int(input('Enter index: '))
    method = input('Method (admm or miqp): ')
    horizon = int(input("Horizon: "))
    file = f"../{input('Save file name: ../')}"
    make_log(file, idx, method, horizon)


def choose_your_fighter():
    choice = input("What do you want to do?\n 1) run ADMM vs MIQP experiment\n"
                   " 2) Run a grid search over the ADMM parameters\n"
                   " 3) run a single trial and make a log\n\nchoice: ")
    if int(choice) == 1:
        run_miqp_vs_admm_experiment()
    elif int(choice) == 2:
        run_admm_params_grid_search()
    elif int(choice) == 3:
        make_log_driver()
    else:
        print("invalid option")


if __name__ == '__main__':
    choose_your_fighter()
