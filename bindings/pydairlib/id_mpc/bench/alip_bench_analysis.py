import os
import lcm
import glob
import argparse
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor

import dairlib

from pydairlib.systems import(
    make_alip_s2s_mpfc_params_from_yaml,
    AlipS2SMpfcSolution,
    AlipS2SMpfcParams,
    AlipS2SMpfcInput,
    AlipS2SMPFC,
    Stance
)

from pydairlib.geometry import (
    ConvexPolygonSet,
    ConvexPolygon
)

from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context
from pydairlib.analysis.mpfc_plotting_utils import alip_mpfc_debug_complete_callback
from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.perceptive_locomotion.results.analysis_utils import setup_plots, plotting_palette


def make_mpfc_params(method: str, horizon: int):
    base_folder = "bindings/pydairlib/id_mpc/bench/"
    solver_options = 'gurobi_options_multithreaded' if method == 'miqp' else \
        'ncqp_params'
    solver_options_file = os.path.join(
        base_folder, f'gains/{solver_options}.yaml'
    )
    gains_file = os.path.join(
        base_folder, f'gains/mpfc_gains_{method}_{horizon}.yaml'
    )
    plant, context = make_plant_and_context()
    return make_alip_s2s_mpfc_params_from_yaml(
        gains_file,
        solver_options_file,
        plant,
        context
    )


def foothold_set_from_lcm(msg):
    footholds = []
    for f in msg.footholds:
        foothold = ConvexPolygon()
        Aeq = np.array(f.Aeq)
        A = np.array(f.A)
        b = np.array(f.b)
        foothold.SetPlane(Aeq, f.beq)
        foothold.AddFaces(A, b)
        footholds.append(foothold)
    return ConvexPolygonSet(footholds)


def extract_mpfc_inputs(data, mpfc_input_channel):
    inputs = []
    for msg in data[mpfc_input_channel]:
        mpfc_input = AlipS2SMpfcInput()
        mpfc_input.x = np.array(msg.x)
        mpfc_input.p = np.array(msg.p)
        mpfc_input.t = msg.t
        mpfc_input.vdes = np.array(msg.vdes)
        mpfc_input.tmin = msg.tmin
        mpfc_input.tmax = msg.tmax
        mpfc_input.stance = Stance.kLeft if msg.stance < 0 else Stance.kRight
        mpfc_input.footholds = foothold_set_from_lcm(msg.footholds)
        mpfc_input.p_prev_stance = np.array(msg.p_prev_stance)
        inputs.append(mpfc_input)
    return inputs


def get_mpfc_inputs(logfile: str):
    log = lcm.EventLog(logfile, "r")

    channel = "ALIP_S2S_MPFC_INPUTS"
    data_channels = {channel: dairlib.lcmt_alip_s2s_mpfc_input}
    input_data = get_log_data(
        log,
        data_channels,
        0, -1,
        extract_mpfc_inputs,
        channel
    )
    return input_data


def calc_cost_sensitivity_to_x(problem: AlipS2SMpfcInput, solver: AlipS2SMPFC):
    n = 5
    noise_bounds = np.array([0.01, 0.01, 0.1, 0.1])
    costs = []
    for i in range(n):
        tmp = problem
        tmp.x = tmp.x + np.random.uniform(-noise_bounds, noise_bounds)
        sol = solver.Solve(problem)
        costs.append(sol.total_cost)

    return np.max(costs) - np.min(costs)


def sensitivity_analysis(mpfc_inputs, horizon: int):
    params_admm = make_mpfc_params('admm', horizon)
    params_miqp = make_mpfc_params('miqp', horizon)
    admm_solver = AlipS2SMPFC(params_admm)
    miqp_solver = AlipS2SMPFC(params_miqp)

    ranges_miqp = [
        calc_cost_sensitivity_to_x(inp, miqp_solver) for inp in mpfc_inputs
    ]
    ranges_admm = [
        calc_cost_sensitivity_to_x(inp, admm_solver) for inp in mpfc_inputs
    ]
    plt.plot(ranges_admm)
    plt.plot(ranges_miqp)
    plt.legend(['ADMM', 'MIQP'])
    plt.show()


def cost_comparison(mpfc_inputs, horizon: int):
    params_admm = make_mpfc_params('admm', horizon)
    params_miqp = make_mpfc_params('miqp', horizon)
    admm_solver = AlipS2SMPFC(params_admm)
    miqp_solver = AlipS2SMPFC(params_miqp)

    solutions_miqp = [
        miqp_solver.Solve(inp) for inp in mpfc_inputs
    ]
    solutions_admm = [
        admm_solver.Solve(inp) for inp in mpfc_inputs
    ]

    admm_costs = [sol.total_cost for sol in solutions_admm]
    miqp_costs = [sol.total_cost for sol in solutions_miqp]
    admm_success = [1 if sol.success else 0 for sol in solutions_admm]
    miqp_success = [1 if sol.success else 0 for sol in solutions_miqp]
    admm_step_sizes = [np.linalg.norm(sol.pp[1] - sol.pp[0]) for sol in solutions_admm]
    miqp_step_sizes = [np.linalg.norm(sol.pp[1] - sol.pp[0]) for sol  in solutions_miqp]

    plt.plot(admm_costs)
    plt.plot(miqp_costs)
    plt.legend(['ADMM', 'MIQP'])

    plt.figure()
    plt.plot(admm_success)
    plt.plot(miqp_success)
    plt.legend(['ADMM', 'MIQP'])

    plt.figure()
    plt.plot(admm_step_sizes)
    plt.plot(miqp_step_sizes)
    plt.legend(['ADMM', 'MIQP'])
    plt.show()


def log_analysis_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logfile")
    args = parser.parse_args()
    mpfc_inputs = get_mpfc_inputs(args.logfile)
    cost_comparison(mpfc_inputs, 5)


def plot_heatmap(data_dict):
    setup_plots()
    # Extract all row and column indices
    rows = sorted(data_dict.keys())
    cols = sorted(set(col for row_dict in data_dict.values() for col in row_dict.keys()))

    # Create a numpy array to hold the data
    matrix = np.zeros((len(rows), len(cols)))

    # Fill the matrix with values from the dictionary
    for i, row in enumerate(rows):
        for j, col in enumerate(cols):
            matrix[i, j] = data_dict.get(row, {}).get(col, 0)

    # Create the figure and axis
    plt.figure(figsize=(7.5, 7.5))

    # Create heatmap
    sns.heatmap(matrix, annot=True, fmt='g',
                xticklabels=cols, yticklabels=rows, cbar=False)

    # Add labels and title
    plt.xlabel('$\\log_{10}(\\rho)$')
    plt.ylabel('ADMM Iterations')
    plt.title('Stepping Stone Success Rate')

    # Adjust layout and display
    plt.tight_layout()
    plt.savefig('../admm_grid_search_results.svg')

    return matrix


def plot_success_rates(results_file_name, savefile_name):
    data = np.load(results_file_name, allow_pickle=True)[
        'success_counts'
    ].item()

    setup_plots()
    plt.figure(figsize=(8, 4))
    plt.plot([k - 1 for k in data['admm'].keys()][:3], list(data['admm'].values())[:3], label='ADMM')
    plt.plot([k - 1 for k in data['miqp'].keys()], data['miqp'].values(), label='MIQP')
    plt.xlabel('Planning Horizon (Footsteps)')
    plt.xticks([2, 3, 4])
    plt.ylabel('Success Rate (pct.)')
    plt.title('Success Rate vs. Planning Horizon')
    plt.legend()
    plt.tight_layout()
    plt.savefig(savefile_name)


def get_mpfc_debug_data(logfile: str):
    log = lcm.EventLog(logfile, "r")
    channel = "ALIP_S2S_MPFC_DEBUG"
    data_channels = {channel: dairlib.lcmt_alip_mpfc_debug_complete}

    data = get_log_data(
        log,
        data_channels,
        0, -1,
        alip_mpfc_debug_complete_callback,
        channel
    )
    return data


def load_log_and_extract_solve_times(logfile: str):
    data = get_mpfc_debug_data(logfile)
    solve_times = data['solve_time']
    n = int(0.95 * len(solve_times))
    return solve_times[:n]


def load_solve_times(logfolder: str, method: str, horizon: int):
    pattern = os.path.join(
        logfolder,
        f'stepping_stone_bench_{method}_{horizon}_*'
    )
    logs = glob.glob(pattern)
    assert(len(logs) == 100)
    solve_times = []
    with ProcessPoolExecutor(max_workers=8) as exec:
        for data in exec.map(load_log_and_extract_solve_times, logs):
            solve_times.extend(data)

    return solve_times


def solve_time_plots(data, savefile):
    # Extract methods and n values
    methods = list(data.keys())
    horizons = list(data[methods[0]].keys())

    # Set up figure
    fig, ax = plt.subplots(figsize=(10, 6))

    # Calculate positions
    positions = np.arange(len(horizons))
    width = 0.3

    # Plot each method
    for i, method in enumerate(methods):
        # Extract data for this method
        method_data = [data[method][n] for n in horizons]

        # Create box plot
        offset = width * (i - 0.5 * (len(methods) - 1))
        bp = ax.boxplot(
            method_data,
            positions=positions + offset,
            patch_artist=True,
            whis=(0.0, 99.9)
        )

        # Style the boxes
        color = plotting_palette[3] if method == 'admm' else plotting_palette[4]
        for box in bp['boxes']:
            box.set(facecolor=color)

    # Add labels and styling
    ax.set_xticks(positions)
    ax.set_xticklabels([f'N={n-1}' for n in horizons])
    ax.set_xlabel('Planning Horizon (Footsteps)')
    ax.set_ylabel('Solve Time')
    ax.set_yscale('log')
    ax.set_title('Solve Time Distributions')
    ax.grid(axis='y', linestyle='--', alpha=0.7)

    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=plotting_palette[3], label='ADMM'),
                       Patch(facecolor=plotting_palette[4], label='MIQP')]
    ax.legend(handles=legend_elements)

    plt.tight_layout()
    plt.savefig(savefile)


def solve_time_main(logfolder: str, savefile: str):
    data = {
        'admm': {n: load_solve_times(logfolder, 'admm', n) for n in [3, 4, 5]},
        'miqp': {n: load_solve_times(logfolder, 'miqp', n) for n in [3, 4, 5]}
    }
    solve_time_plots(data, savefile)

def grid_search_analysis_main():
    data = np.load(
        '../alip_bench_grid_search_results_4_step.npz',
        allow_pickle=True
    )['success_counts'].item()
    plot_heatmap(data)


def results_analysis_main():
    bench_results = '../alip_bench_results.npz'
    savefolder = '/Volumes/Extreme SSD/alip_mpfc_bench_logs/figures'
    logfolder = '/Volumes/Extreme SSD/alip_mpfc_bench_logs'
    setup_plots()
    solve_time_main(
        logfolder,
        os.path.join(savefolder, 'solve_times.svg')
    )
    plot_success_rates(
        bench_results,
        os.path.join(savefolder, 'success_rates.svg')
    )


if __name__ == '__main__':
    results_analysis_main()

