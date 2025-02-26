import os
import tempfile
import numpy as np
import multiprocessing as mp
from functools import partial
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

from pydairlib.perceptive_locomotion import FullSimDiagram

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


def random_stepping_stones(seed, savefile=None):
    rng = np.random.default_rng(seed)
    rows = 5
    cols = 3
    base_len = 1.35
    y_variation = 0.05
    x_variation = 0.03
    z_variation = 0.05
    x_dist = 0.6
    y_dist = 0.65
    
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
            lxs.append(rng.uniform(0.4, 0.45))
            lys.append(rng.uniform(0.45, 0.5))
            lzs.append(0.3)
            yaws.append(rng.uniform(-0.1, 0.1))
    
    block_str = get_blocks_string(xs, ys, zs, normals, lxs, lys, lzs, yaws)
    if savefile is None:
        print(block_str)
    else:
        with open(savefile, 'w') as fp:
            fp.write(block_str)


def build_and_run_sim(gains: str, terrain: str, params: str):
    
    terrain_segmentation = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    terrain_segmentation.MakeDrivenByStandaloneSimulator(1.0/30.0)
    convex_decomposition = ConvexTerrainDecompositionSystem()
    sim_diagram = FullSimDiagram(gains, terrain, params)
    
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
    
    # we consider the simulation to be a success if the robot stays upright
    # and makes it to the final stepping stone within the allotted time
    return success


def run_single_trial(trial_idx, gains, params):
    """Run a single simulation trial"""
    terrain_file = tempfile.NamedTemporaryFile(delete=False)
    try:
        random_stepping_stones(trial_idx, terrain_file.name)
        return 1 if build_and_run_sim(gains, terrain_file.name, params) else 0
    finally:
        terrain_file.close()
        try:
            os.unlink(terrain_file.name)
        except:
            pass


def run_study_parallel(gains: str, params: str, num_trials: int, num_workers: int = 3):
    """Parallelized version of run_study"""
    worker_fn = partial(run_single_trial, gains=gains, params=params)
    success_count = 0
    fail_count = 0
    
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        for success in executor.map(worker_fn, range(num_trials)):
            success_count += success
            fail_count += (1 - success)
            print(f'successes: {success_count}\nfailures: {fail_count}')
    
    return {'success': success_count, 'fail': fail_count}


def main():
    n_trials = 20
    params = "bindings/pydairlib/perceptive_locomotion/sim_experiments/sim_opts_stones.yaml"
    gains = "bindings/pydairlib/perceptive_locomotion/sim_experiments/gains/mpfc_gains_default.yaml"

    results = run_study_parallel(gains, params, n_trials)
    print(results)
    
    
if __name__ == '__main__':
    main()
