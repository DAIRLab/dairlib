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
from pydrake.all import DrakeLcm

import pydairlib.lcm  # needed for cpp serialization of lcm messages

from grid_map import GridMap

from pydairlib.systems.perception import GridMapSender, PlaneSegmentationSystem

from pydairlib.analysis.process_lcm_log import get_log_data

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem

from pydairlib.peceptive_locomotion.terrain_segmentation import perception_analysis_utils as utils

from pydairlib.geometry.convex_polygon import ConvexPolygonSender

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria


from argparse import ArgumentParser


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'


def build_diagram(mode: str, lcm: DrakeLcm, profiling=None) -> Diagram:

    builder = DiagramBuilder()

    terrain_segmentation = PlaneSegmentationSystem(
        'systems/perception/ethz_plane_segmentation/params.yaml'
    ) if mode == 'planar' else TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'variance_criterion': seg_criteria.variance_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion
        },
        profiling
    )

    convex_decomposition = ConvexTerrainDecompositionSystem(profiling)
    foothold_sender = ConvexPolygonSender()

    # Add a foothold publisher to drive the simulation with ForcedPublish
    foothold_publisher = LcmPublisherSystem.Make(
        channel="FOOTHOLDS_PROCESSED",
        lcm_type=lcmt_foothold_set,
        lcm=lcm,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)
    builder.AddSystem(foothold_publisher)
    builder.AddSystem(foothold_sender)

    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
    )
    builder.Connect(
        convex_decomposition.get_output_port(),
        foothold_sender.get_input_port()
    )
    builder.Connect(
        foothold_sender.get_output_port(),
        foothold_publisher.get_input_port()
    )

    builder.ExportInput(
        terrain_segmentation.get_input_port(),
        "grid_map"
    )
    diagram = builder.Build()
    return diagram


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


def run_segmentation_profiling(logfile):
    log = lcm.EventLog(logfile, "r")
    grid_maps, _ = get_log_data(
        lcm_log=log,
        lcm_channels={
            'CASSIE_ELEVATION_MAP': lcmt_grid_map,
            state_channel: lcmt_robot_output
        },
        start_time=0,
        duration=5,
        data_processing_callback=utils.process_grid_maps,
        state_channel=state_channel
    )
    plane_segmentation = PlaneSegmentationSystem(
        'systems/perception/ethz_plane_segmentation/params.yaml'
    )
    s3 = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'variance_criterion': seg_criteria.variance_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion
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
    plt.title(f'Frame-to-Frame IoU')
    for r in results:
        plt.plot(r['iou'])
    plt.legend([r['name'] for r in results])
    plt.xlabel('Frame Number')
    plt.ylabel('IOU  with Next Frame')
    fig.tight_layout()
    plt.savefig(savefile)


def profile_full_pipeline(logfile):
    profiling_results = {
        'segmentation': [],
        'seg_callbacks': [[], [], []],
        'decomposition': [],
        'plane_fitting': [],
        'num_polygons': []
    }

    log = lcm.EventLog(logfile, "r")
    lcm_interface = DrakeLcm()

    diagram = build_diagram('convex', lcm_interface, profiling=profiling_results)

    grid_maps, robot_output = get_log_data(
        lcm_log=log,
        lcm_channels={
            elevation_map_channel: lcmt_grid_map,
            state_channel: lcmt_robot_output
        },
        start_time=0,
        duration=-1,
        data_processing_callback=utils.process_grid_maps,
        elevation_map_channel=elevation_map_channel,
        state_channel=state_channel
    )

    context = diagram.CreateDefaultContext()

    states_per_gm = round(len(robot_output) / len(grid_maps))
    s = 0
    for map in grid_maps:
        diagram.get_input_port().FixValue(context, map)
        start = time.time()
        diagram.CalcForcedUnrestrictedUpdate(
            context,
            context.get_mutable_state()
        )
        diagram.ForcedPublish(context)
        end = time.time()
        print(end - start)
        for i in range(states_per_gm):
            if s >= len(robot_output):
                break
            lcm_interface.Publish(
                state_channel,
                robot_output[s].encode()
            )
            s += 1
        sleep(0.05)

    for i in range(3):
        plt.plot(profiling_results['seg_callbacks'][i])

    plt.legend(['curvature', 'variance', 'inclination'])
    plt.show()


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfile', type=str)
    args = parser.parse_args()
    run_segmentation_profiling(args.logfile)
    # profile_full_pipeline(args.logfile)


if __name__ == '__main__':
    main()
