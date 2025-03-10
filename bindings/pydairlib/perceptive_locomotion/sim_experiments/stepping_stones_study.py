import os
import sys
import signal
import tempfile
import numpy as np
from copy import deepcopy
import multiprocessing as mp
from functools import partial
from dataclasses import dataclass
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from concurrent.futures import ProcessPoolExecutor

from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    DiagramBuilder,
    LcmPublisherSystem,
    LcmSubscriberSystem,
    TriggerType,
)

from pydrake.math import RigidTransform

from pydrake.common.value import AbstractValue

from pydairlib.perceptive_locomotion import (
    ConvexTerrainDecompositionSystem,
    TerrainSegmentationSystem,
    PerceptiveFullSimDiagram,
    FullSimDiagram
)

from pydairlib.perceptive_locomotion.results.analysis_utils import setup_plots

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria


@dataclass
class TrialParams:
    gains: str
    terrain: str
    sim_params: str
    perceptive: bool
    terrain_size: float
    log_name: str = None
    safety_margin: float = None


# Utils for generating random stepping stones
def get_block_string(x, y, z, n, lx, ly, lz, yaw):
    return f'  - [[{x}, {y}, {z}], [{n[0]}, {n[1]}, {n[2]}], ' \
           f'[{lx}, {ly}, {lz}], [{yaw}]]'


def get_blocks_string(x, y, z, normal, lx, ly, lz, yaw):
    out = 'stones:\n'
    for i in range(len(x)):
        n = normal[i]
        out = out + get_block_string(
            x[i], y[i], z[i], n, lx[i], ly[i], lz[i], yaw[i]
        ) + '\n'
    return out


def random_stepping_stones(seed, min_sidelength, savefile=None):
    rng = np.random.default_rng(seed)
    rows = 5
    cols = 3
    base_len = 1.0 + 0.25 - min_sidelength
    y_variation = 0.075
    x_variation = 0.075
    z_variation = 0.075
    yaw_variation = 10.0 * np.pi / 180.0
    x_dist = min_sidelength + 0.25
    y_dist = min_sidelength + 0.25
    
    # initialize stepping stone geometry with the start, end, and floor blocks
    xs = [0.0, base_len * 2 + (rows + 1) * x_dist, 5.0]
    ys = [0.0, 0.0, 0.0]
    zs = [0.0, 0.0, -0.5]
    normals = [np.array([0, 0, 1]), np.array([0, 0, 1]), np.array([0, 0, 1])]
    lxs = [2.0, 2.0, 20.0]
    lys = [2.5, 2.5, 20.0]
    lzs = [0.2, 0.2, 0.2]
    yaws = [0.0, 0.0, 0.0]

    for r in range(rows):
        x = base_len + x_dist * (r + 1) + rng.uniform(-x_variation, x_variation)
        for c in range(cols):
            y = rng.uniform(-y_variation, y_variation) + y_dist * (c - 0.5 * (cols - 1))
            z = rng.uniform(-z_variation, z_variation)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            normals.append(rng.uniform([-0.09, -0.06, 1.0], [0.09, 0.09, 1.0]))
            lxs.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lys.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lzs.append(0.3)
            yaws.append(rng.uniform(-yaw_variation, yaw_variation))
    
    block_str = get_blocks_string(xs, ys, zs, normals, lxs, lys, lzs, yaws)
    if savefile is None:
        print(block_str)
    else:
        with open(savefile, 'w') as fp:
            fp.write(block_str)


def goal_x(terrain_size: float):
    base_len = 1.0 + 0.25 - terrain_size
    x_dist = terrain_size + 0.25
    return base_len * 2.0 + x_dist * 6.0 - 1.0


def build_and_run_sim(trial_params: TrialParams):
    sim_diagram = FullSimDiagram(trial_params.gains, trial_params.terrain, trial_params.sim_params)

    builder = DiagramBuilder()
    builder.AddSystem(sim_diagram)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)

    sim_params = load(open(trial_params.sim_params, 'r'), Loader=Loader)
    run_time = sim_params['time'][0]

    try:
        simulator.AdvanceTo(run_time)
    except:
        if trial_params.log_name:
            sim_diagram.SaveLcmLog(trial_params.log_name + '_fail')
        return False

    pelvis_pose = sim_diagram.GetCassiePelvisPoseInWorld(context)

    success = pelvis_pose.translation().ravel()[-1] > 0.8 and \
              pelvis_pose.translation().ravel()[0] > goal_x(trial_params.terrain_size)

    if success and trial_params.log_name:
        sim_diagram.SaveLcmLog(trial_params.log_name + '_success')
    elif trial_params.log_name:
        sim_diagram.SaveLcmLog(trial_params.log_name + '_fail')

    # we consider the simulation to be a success if the robot stays upright
    # and makes it to the final stepping stone within the allotted time
    return success


def build_and_run_perceptive_sim(trial_params: TrialParams):
    
    terrain_segmentation = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    terrain_segmentation.MakeDrivenByStandaloneSimulator(1.0/30.0)
    terrain_segmentation.erosion_kernel_length = trial_params.safety_margin

    convex_decomposition = ConvexTerrainDecompositionSystem()
    sim_diagram = PerceptiveFullSimDiagram(trial_params.gains, trial_params.terrain, trial_params.sim_params)
    
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
    
    sim_params = load(open(trial_params.sim_params, 'r'), Loader=Loader)
    run_time = sim_params['time'][0]

    try:
        simulator.AdvanceTo(run_time)
    except:
        if trial_params.log_name:
            sim_diagram.SaveLcmLog(trial_params.log_name + f'_margin_{100 * trial_params.safety_margin:.0f}_cm_fail')
        return False
    
    pelvis_pose = sim_diagram.GetCassiePelvisPoseInWorld(context)

    success = pelvis_pose.translation().ravel()[-1] > 0.8 and \
              pelvis_pose.translation().ravel()[0] > goal_x(trial_params.terrain_size)
    
    if success and trial_params.log_name:
        sim_diagram.SaveLcmLog(trial_params.log_name + f'margin_{100 * trial_params.safety_margin:.0f}_cm_success')
    elif trial_params.log_name:
        sim_diagram.SaveLcmLog(trial_params.log_name + f'margin_{100 * trial_params.safety_margin:.0f}_cm_fail')
    
    # we consider the simulation to be a success if the robot stays upright
    # and makes it to the final stepping stone within the allotted time
    return success


def make_log_name(trial_idx: int, trial_params: TrialParams):
    gains_prefix = trial_params.gains.split("/")[-1].replace(".yaml", "")
    size = str(trial_params.terrain_size).replace("0.", "") + "_cm"
    perceptive = 'perceptive' if trial_params.perceptive else 'gt_terrain'
    return f'{gains_prefix}_{size}_trial_{perceptive}_{trial_idx}'


def run_single_trial(trial_idx: int, trial_params: TrialParams):
    """Run a single simulation trial"""
    logfolder = '../stepping_stone_study_results/logs/'

    logname = make_log_name(trial_idx, trial_params)
    os.makedirs(logfolder, exist_ok=True)
    save_log = os.path.join(logfolder, logname)

    terrain_file = tempfile.NamedTemporaryFile(delete=False, prefix=f'{trial_idx}_terrain')
    random_stepping_stones(trial_idx, trial_params.terrain_size, savefile=terrain_file.name)

    sim_params_text = f'goal_location: [{goal_x(trial_params.terrain_size) + 0.5}, 0]\ntime: [30]\nrealtime_rate: [-1]'
    sim_params_file = tempfile.NamedTemporaryFile(delete=False, prefix=f'{trial_idx}_sim_params')
    with open(sim_params_file.name, 'w') as fp:
        fp.write(sim_params_text)

    params = trial_params
    params.log_name = save_log
    params.sim_params = sim_params_file.name
    params.terrain = terrain_file.name

    try:
        if params.perceptive:
            return 1 if build_and_run_perceptive_sim(params) else 0
        else:
            return 1 if build_and_run_sim(params) else 0

    finally:
        terrain_file.close()
        sim_params_file.close()
        try:
            os.unlink(terrain_file.name)
        except FileNotFoundError:
            pass
        try:
            os.unlink(sim_params_file.name)
        except FileNotFoundError:
            pass

# Global executor for cleanup on signal
executor = None


def signal_handler(sig, frame):
    """Handle Ctrl+C by shutting down all worker processes"""
    print("\nKeyboard interrupt received. Shutting down worker processes...")
    if executor:
        executor.shutdown(wait=False)
    sys.exit(1)


def run_study_parallel(trial_params_template: TrialParams,  num_trials: int, num_workers: int = 8):
    """Parallelized version of run_study with signal handling"""
    global executor
    worker_fn = partial(run_single_trial, trial_params=trial_params_template)
    success_count = 0
    fail_count = 0

    # Set up signal handler before creating processes
    signal.signal(signal.SIGINT, signal_handler)

    try:
        with ProcessPoolExecutor(max_workers=num_workers) as exec:
            executor = exec
            for success in executor.map(worker_fn, range(num_trials)):
                success_count += success
                fail_count += (1 - success)
    except KeyboardInterrupt:
        # This will be caught by the signal handler
        pass
    finally:
        executor = None

    return {'success': success_count, 'fail': fail_count}


def timing_study_main(fname):
    n_trials = 100
    gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"
    gains_no_timing = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_no_timing_adaptation.yaml"

    results_gt = {}
    results_gt_no_timing = {}
    results_perceptive = {}
    results_perceptive_no_timing = {}

    for terrain_size in [0.25, 0.3, 0.35, 0.4, 0.45, 0.5]:
        study_params = TrialParams(
            gains="",
            terrain="",
            sim_params="",
            perceptive=False,
            terrain_size=terrain_size,
            safety_margin=0.12
        )
        try:
            study_params.gains = gains
            results_gt[terrain_size] = run_study_parallel(study_params, n_trials)

            study_params.gains = gains_no_timing
            results_gt_no_timing[terrain_size] = run_study_parallel(study_params, n_trials)

            study_params.perceptive = True
            results_perceptive_no_timing[terrain_size] = run_study_parallel(study_params, n_trials)

            study_params.gains = gains
            results_perceptive[terrain_size] = run_study_parallel(study_params, n_trials)
        except KeyboardInterrupt:
            print("\nStudy terminated by user.")

    np.savez(
        fname,
        results_gt=results_gt,
        results_gt_no_timing=results_gt_no_timing
    )


def perception_study_main(fname):
    n_trials = 100
    gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"

    results = {}

    study_params = TrialParams(
        gains=gains,
        terrain="",
        sim_params="",
        perceptive=True,
        terrain_size=0,
    )
    for margin in [0.15, 0.12, 0.09, 0.06]:
        results[margin] = {}
        study_params.safety_margin = margin
        for terrain_size in [0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3]:
            try:
                study_params.terrain_size = terrain_size
                results[margin][terrain_size] = run_study_parallel(study_params, n_trials)
            except KeyboardInterrupt:
                print("\nStudy terminated by user.")

    np.savez(fname, results=results)


def cost_study_main(fname):
    n_trials = 50
    params = "bindings/pydairlib/perceptive_locomotion/sim_experiments/sim_opts_stones.yaml"
    gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"
    gains_gait_cost = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_gait_cost.yaml"

    results_vel = {}
    results_gait = {}
    for terrain_size in [0.25, 0.3, 0.35, 0.4, 0.45]:
        try:
            results_vel[terrain_size] = run_study_parallel(
                gains, params, n_trials, terrain_size, perceptive=False
            )
            results_gait[terrain_size] = run_study_parallel(
                gains_gait_cost, params, n_trials, terrain_size, perceptive=False
            )
        except KeyboardInterrupt:
            print("\nStudy terminated by user.")

    np.savez(
        fname,
        results_vel=results_vel,
        results_gait=results_gait
    )


def plot_timing_results(folder):
    fname = os.path.join(folder, 'timing_adaptation_results.npz')
    data = np.load(fname, allow_pickle=True)
    conditions = {
        "results_gt": "Timing Adaptation",
        "results_gt_no_timing": "No Timing Adaptation",
    }

    markers = {
        "results_gt": "*",
        "results_gt_no_timing": "x",
        "results_perceptive": "^",
        "results_perceptive_no_timing": "o"
    }

    setup_plots()

    terrain_sizes = list(data["results_gt"].item().keys())
    for condition, title in conditions.items():
        success_rates = []
        terrain_size_data = data[condition].item()
        for terrain_size in terrain_size_data.keys():
            trial_data = terrain_size_data[terrain_size]
            success_rate = 100 * float(trial_data['success']) / float(trial_data['success'] + trial_data['fail'])
            success_rates.append(success_rate)
        plt.plot(terrain_sizes, success_rates, label=title, marker=markers[condition])

    plt.title('Success Rates for Randomly Generated Stepping Stones')
    plt.xticks(terrain_sizes, terrain_sizes)
    plt.xlabel('Minimum Stepping Stone Side Length (m)')
    plt.ylabel('Success Rate (\\%)')
    plt.legend()


def plot_margin_results(folder):
    fname = os.path.join(folder, 'margin_adaptation_results.npz')
    data = np.load(fname, allow_pickle=True)
    results = data['results'].item()
    margins = [*results]
    margins.sort()

    markers = {
        0.06: "*",
        0.09: "x",
        0.12: "^",
        0.15: "o"
    }

    for margin in margins:
        label = f'{margin:.2f} m'
        sizes = [*results[margin]]
        sizes.sort()

        success_rates = []
        for size in sizes:
            trial_data = results[margin][size]
            success_rate = 100 * float(trial_data['success']) / float(trial_data['success'] + trial_data['fail'])
            success_rates.append(success_rate)
        plt.plot(sizes, success_rates, label=label, marker=markers[margin])
        plt.title('Stepping Stone Success vs. S3 Safety Margin')
        plt.xlabel('$d_{min}$')
        plt.ylabel('Success Rate')
        plt.legend()


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "--saved_results_folder",
        type=str,
        help='Filename of study results that have already been saved for plotting. Leave empy to run the study',
        default=None
    )
    args = parser.parse_args()
    if args.saved_results_folder:
        plot_timing_results(args.saved_results_folder)
        plt.figure()
        plot_margin_results(args.saved_results_folder)
        plt.show()
    else:
        perception_study_main('../stepping_stone_study_results/margin_adaptation_results.npz')
        timing_study_main('../stepping_stone_study_results/timing_adaptation_results.npz')
