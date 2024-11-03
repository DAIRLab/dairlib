import time
import timeit
from time import sleep
from typing import List
from copy import deepcopy

import lcm
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dairlib import lcmt_grid_map, lcmt_foothold_set, lcmt_robot_output

from pydrake.systems.all import (
    Diagram,
    Context,
    DiagramBuilder,
    LcmPublisherSystem,
    TriggerType,
)

from pydrake.geometry import Meshcat

from pydrake.all import DrakeLcm

import pydairlib.lcm  # needed for cpp serialization of lcm messages

from grid_map import GridMap

from pydairlib.systems.perception import GridMapSender, PlaneSegmentationSystem

from pydairlib.analysis.process_lcm_log import get_log_data

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem, plot_polygon, plot_polygons_with_holes

from pydairlib.perceptive_locomotion.terrain_segmentation import perception_analysis_utils as utils

from pydairlib.geometry.convex_polygon import ConvexPolygonSender

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria


from argparse import ArgumentParser


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'


def get_grid_maps_from_log(logfile: str):
    log = lcm.EventLog(logfile, "r")
    grid_maps, _ = get_log_data(
        log,
        {
            elevation_map_channel: lcmt_grid_map,
            state_channel: lcmt_robot_output
        }, 0, 5,
        utils.process_grid_maps,
        elevation_map_channel,
        state_channel,
    )
    return grid_maps


def make_pipeline_figures_from_map(grid_map: GridMap):

    # Turn on debug to save intermediate computations
    segmentation = TerrainSegmentationSystem({
        'curvature_criterion': seg_criteria.curvature_criterion,
        'variance_criterion': seg_criteria.variance_criterion,
    })
    decomposition = ConvexTerrainDecompositionSystem()
    segmentation.debug = True
    decomposition.debug = True
    segmentation_context = segmentation.CreateDefaultContext()
    decomposition_context = decomposition.CreateDefaultContext()

    # Do the segmentation
    segmentation.get_input_port().FixValue(
        segmentation_context, grid_map
    )
    segmentation.UpdateTerrainSegmentation(
        segmentation_context, segmentation_context.get_mutable_state()
    )

    # Do the convex decomposition
    decomposition.get_input_port().FixValue(
        decomposition_context,
        segmentation.get_output_port().Eval(segmentation_context)
    )
    convex_polygons = decomposition.get_output_port().Eval(decomposition_context)
    print('done')


def profile_segmentation(system, grid_maps):
    runtime = []
    iou = []

    context = system.CreateDefaultContext()
    process_grid_maps = []
    segmentations = []
    for map in grid_maps:
        system.get_input_port().FixValue(context, map)
        start = time.time()
        system.CalcForcedUnrestrictedUpdate(
            context,
            context.get_mutable_state()
        )
        try:
            process_grid_maps.append(deepcopy(system.get_output_port().Eval(context)))
        except RuntimeError:
            import pdb; pdb.set_trace()

        segmentations.append(process_grid_maps[-1]['segmentation'])
        end = time.time()
        runtime.append(end - start)

    for i in range(len(process_grid_maps) - 1):
        iou.append(
            utils.safe_terrain_iou(process_grid_maps[i], process_grid_maps[i+1])
        )

    return {
        'name': system.get_name(),
        'iou': iou,
        'runtime': runtime,
        'segmentations': segmentations
    }


def animate_segmentations(results):
    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.set_title(results[0]['name'])
    ax2.set_title(results[1]['name'])
    imdata = np.zeros_like(results[0]['segmentations'][0])

    # Display the initial frame using imshow
    img1 = ax1.imshow(imdata, interpolation='none', aspect=1, vmin=0, vmax=1)
    img2 = ax2.imshow(imdata, interpolation='none', aspect=1, vmin=0, vmax=1)

    def anim_callback(i):
        img1.set_array(results[0]['segmentations'][i])
        img2.set_array(results[1]['segmentations'][i])
        return [img1, img2]

    animation = FuncAnimation(
        fig, anim_callback, frames=range(len(results[0]['segmentations'])), blit=False, interval=50
    )

    plt.show()


def plot_segmentation_run_time_results(results, title, savefile):
    fig = plt.figure()
    plt.title(title)
    for r in results:
        plt.plot(r['runtime'])
    plt.xlabel('Frame Number')
    plt.ylabel('Segmentation Run Time (s)')
    plt.legend([r['name'] for r in results])
    fig.tight_layout()
    plt.savefig(savefile)


def plot_iou_results(results, title, savefile):
    fig = plt.figure()
    plt.title(title)
    for r in results:
        plt.plot(r['iou'])
    plt.legend([r['name'] for r in results])
    plt.xlabel('Frame Number')
    plt.ylabel('IOU  with Next Frame')
    fig.tight_layout()
    plt.savefig(savefile)


def run_segmentation_profiling(logfile):
    grid_maps = get_grid_maps_from_log(logfile)
    plane_segmentation = PlaneSegmentationSystem(
        'systems/perception/ethz_plane_segmentation/params.yaml'
    )
    s3 = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'variance_criterion': seg_criteria.variance_criterion,
        }
    )

    results = [
        profile_segmentation(plane_segmentation, deepcopy(grid_maps)),
        profile_segmentation(s3, deepcopy(grid_maps))
    ]

    utils.setup_plots()

    plot_segmentation_run_time_results(results, 'Run Time', '../test1.svg')
    plot_iou_results(results, 'Frame-to-Frame IoU', '../test2.svg')

    plt.show()


def run_pipeline_figure_script(logfile):
    example_idx = 60
    grid_maps = get_grid_maps_from_log(logfile)
    make_pipeline_figures_from_map(grid_maps[example_idx])


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfile', type=str)
    args = parser.parse_args()
    run_pipeline_figure_script(args.logfile)
    # run_segmentation_profiling(args.logfile)
    # profile_full_pipeline(args.logfile)


if __name__ == '__main__':
    main()
