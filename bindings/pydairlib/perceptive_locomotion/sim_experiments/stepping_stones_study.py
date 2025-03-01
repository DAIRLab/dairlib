import os
import sys
import signal
import tempfile
import numpy as np
import multiprocessing as mp
from functools import partial
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
    PerceptiveFullSimDiagram,
    FullSimDiagram
)

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion import ConvexTerrainDecompositionSystem

from pydairlib.systems import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import OutputVector

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria


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
    base_len = 1.35
    y_variation = 0.05
    x_variation = 0.05
    z_variation = 0.05
    x_dist = 0.6
    y_dist = 0.6
    
    # initialize stepping stone geometry with the start, end, and floor blocks
    xs = [0.0, 1.5 + (rows + 1) * x_dist, 5.0]
    ys = [0.0, 0.0, 0.0]
    zs = [0.0, 0.0, -0.5]
    normals = [np.array([0, 0, 1]), np.array([0, 0, 1]), np.array([0, 0, 1])]
    lxs = [2.0, 2.0, 20.0]
    lys = [2.5, 2.5, 20.0]
    lzs = [0.2, 0.2, 0.2]
    yaws = [0.0, 0.0, 0.0]
    
    z = 0
    for r in range(rows):
        x = base_len + x_dist * r + rng.uniform(-x_variation, x_variation)
        for c in range(cols):
            y = rng.uniform(-y_variation, y_variation) + y_dist * (c - 0.5 * (cols - 1))
            z = rng.uniform(-z_variation, z_variation)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            normals.append(rng.uniform([-0.04, -0.04, 1.0], [0.04, 0.04, 1.0]))
            lxs.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lys.append(rng.uniform(min_sidelength, min_sidelength + 0.05))
            lzs.append(0.3)
            yaws.append(rng.uniform(-0.1, 0.1))
    
    block_str = get_blocks_string(xs, ys, zs, normals, lxs, lys, lzs, yaws)
    if savefile is None:
        print(block_str)
    else:
        with open(savefile, 'w') as fp:
            fp.write(block_str)


def build_and_run_sim(gains: str, terrain: str, params: str, log_name: str=None):
    sim_diagram = FullSimDiagram(gains, terrain, params)

    builder = DiagramBuilder()
    builder.AddSystem(sim_diagram)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    sim_diagram.SetPlantInitialConditions(diagram, context)

    simulator = Simulator(diagram, context)
    simulator.set_publish_every_time_step(False)
    simulator.set_publish_at_initialization(False)

    sim_params = load(open(params, 'r'), Loader=Loader)
    run_time = sim_params['time'][0]

    try:
        simulator.AdvanceTo(run_time)
    except:
        return False

    pelvis_pose = sim_diagram.GetCassiePelvisPoseInWorld(context)

    success = pelvis_pose.translation().ravel()[-1] > 0.8 and \
              pelvis_pose.translation().ravel()[0] > 4.0

    if success and log_name:
        sim_diagram.SaveLcmLog(log_name + '_success')
    elif log_name:
        sim_diagram.SaveLcmLog(log_name + '_fail')

    # we consider the simulation to be a success if the robot stays upright
    # and makes it to the final stepping stone within the allotted time
    return success


def build_and_run_perceptive_sim(gains: str, terrain: str, params: str, log_name: str=None):
    
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
    
    sim_params = load(open(params, 'r'), Loader=Loader)
    run_time = sim_params['time'][0]
    
    try:
        simulator.AdvanceTo(run_time)
    except:
        return False
    
    pelvis_pose = sim_diagram.GetCassiePelvisPoseInWorld(context)
    
    success = pelvis_pose.translation().ravel()[-1] > 0.8 and \
              pelvis_pose.translation().ravel()[0] > 4.0
    
    if success and log_name:
        sim_diagram.SaveLcmLog(log_name + '_success')
    elif log_name:
        sim_diagram.SaveLcmLog(log_name + '_fail')
    
    # we consider the simulation to be a success if the robot stays upright
    # and makes it to the final stepping stone within the allotted time
    return success


def run_single_trial(trial_idx: int, gains: str, params: str, perceptive: bool, terrain_size: float):
    """Run a single simulation trial"""
    logname = gains.split('/')[-1].strip('.yaml')
    save_log = None

    terrain_file = tempfile.NamedTemporaryFile(delete=False)
    random_stepping_stones(trial_idx, terrain_size, savefile=terrain_file.name)

    try:
        if perceptive:
            return 1 if build_and_run_perceptive_sim(
                gains, terrain_file.name, params, save_log) else 0
        else:
            return 1 if build_and_run_sim(
                gains, terrain_file.name, params, save_log) else 0

    finally:
        terrain_file.close()
        try:
            os.unlink(terrain_file.name)
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


def run_study_parallel(gains: str, params: str, num_trials: int, terrain_size: float, perceptive: bool, num_workers: int=3):
    """Parallelized version of run_study with signal handling"""
    global executor
    worker_fn = partial(run_single_trial, gains=gains, params=params, perceptive=perceptive, terrain_size=terrain_size)
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
                print(f'successes: {success_count}\nfailures: {fail_count}')
    except KeyboardInterrupt:
        # This will be caught by the signal handler
        pass
    finally:
        executor = None

    return {'success': success_count, 'fail': fail_count}


def main(fname):
    n_trials = 100
    params = "bindings/pydairlib/perceptive_locomotion/sim_experiments/sim_opts_stones.yaml"
    gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"
    gains_no_timing = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_no_timing_adaptation.yaml"

    results_gt = {}
    results_gt_no_timing = {}
    results_perceptive = {}
    results_perceptive_no_timing = {}

    for terrain_size in [0.3, 0.35, 0.4, 0.45, 0.5]:
        try:
            results_gt[terrain_size] = run_study_parallel(
                gains, params, n_trials, terrain_size, perceptive=False
            )
            results_gt_no_timing[terrain_size] = run_study_parallel(
                gains_no_timing, params, n_trials, terrain_size, perceptive=False
            )
            results_perceptive[terrain_size] = run_study_parallel(
                gains, params, n_trials, terrain_size, perceptive=True
            )
            results_perceptive_no_timing[terrain_size] = run_study_parallel(
                gains_no_timing, params, n_trials, terrain_size, perceptive=True
            )
        except KeyboardInterrupt:
            print("\nStudy terminated by user.")

    np.savez(
        fname,
        results_gt=results_gt,
        results_gt_no_timing=results_gt_no_timing,
        results_perceptive=results_perceptive,
        results_perceptive_no_timing=results_perceptive_no_timing
    )


def plot_results(fname):
    data = np.load(fname, allow_pickle=True)
    conditions = {
        "results_gt": "GT",
        "results_gt_no_timing": "GT (No Timing Adaptation)",
        "results_perceptive": "Perceptive",
        "results_perceptive_no_timing": "Perceptive (No Timing Adaptation)"
    }

    for condition, title in conditions.items():
        terrain_sizes = []
        success_rates = []
        terrain_size_data = data[condition].item()
        for terrain_size in terrain_size_data.keys():
            trial_data = terrain_size_data[terrain_size]
            success_rate = float(trial_data['success']) / float(trial_data['success'] + trial_data['fail'])
            terrain_sizes.append(terrain_size)
            success_rates.append(success_rate)
        plt.plot(terrain_sizes, success_rates, label=title)
    plt.legend()

    plt.show()


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "--saved_results_file",
        type=str,
        help='Filename of study results that have already been saved for plotting. Leave empy to run the study',
        default=None
    )
    parser.add_argument(
        "--results_file",
        type=str,
        help='Filename to save results at',
        default='../stepping_stone_study_results/results.npz',
    )
    args = parser.parse_args()

    if args.saved_results_file:
        plot_results(args.saved_results_file)
    else:
        main(args.results_file)
