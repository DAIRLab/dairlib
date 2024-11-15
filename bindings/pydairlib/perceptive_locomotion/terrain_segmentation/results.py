import os
import time
import timeit
from time import sleep
from copy import deepcopy
from multiprocessing import Pool

import lcm
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import to_rgba
from dairlib import lcmt_grid_map, lcmt_foothold_set, lcmt_robot_output

from pydrake.systems.all import (
    Diagram,
    Context,
    DiagramBuilder,
    LcmPublisherSystem,
    TriggerType,
)

from pydrake.geometry import Meshcat

from grid_map import GridMap

from pydairlib.multibody import MultiposeVisualizer

from pydairlib.systems import (
    PlaneSegmentationSystem,
    GridMapVisualizer,
    GridMapReceiver,
    PlantVisualizer,
    RobotOutputReceiver
)
from pydairlib.geometry import ConvexPolygonVisualizer, ConvexPolygonReceiver

from pydairlib.analysis.process_lcm_log import get_log_data

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem, plot_polygon, plot_polygons_with_holes

from pydairlib.perceptive_locomotion.terrain_segmentation import perception_analysis_utils as utils

import pydairlib.perceptive_locomotion.terrain_segmentation. \
    segmentation_criteria as seg_criteria

from pydairlib.common import MeshcatChromeCapture, write_meshcat_video_from_log

from argparse import ArgumentParser


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'


def get_grid_maps_from_log(logfile: str, start_time=0, duration=-1):
    log = lcm.EventLog(logfile, "r")
    grid_maps, robot_output = get_log_data(
        log,
        {
            elevation_map_channel: lcmt_grid_map,
            state_channel: lcmt_robot_output
        }, start_time, duration,
        utils.process_grid_maps,
        elevation_map_channel,
        state_channel,
    )
    return grid_maps, robot_output


def write_perception_video(logfile: str):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"
    update_period = 1.0 / 30.01
    plant_visualizer = PlantVisualizer(urdf, "pelvis")
    meshcat = plant_visualizer.get_meshcat()

    visualizers = {
        "state": plant_visualizer,
        "grid_map": GridMapVisualizer(meshcat, update_period, ["segmented_elevation"]),
        "polygons": ConvexPolygonVisualizer(meshcat, update_period)
    }
    receivers = {
        "state": RobotOutputReceiver(visualizers["state"].get_plant()),
        "grid_map": GridMapReceiver(),
        "polygons": ConvexPolygonReceiver()
    }

    builder = DiagramBuilder()
    for k in visualizers.keys():
        builder.AddSystem(visualizers[k])
        builder.AddSystem(receivers[k])
        builder.Connect(
            receivers[k].get_output_port(), visualizers[k].get_input_port())

    diagram = builder.Build()

    types = {
        'NETWORK_CASSIE_STATE_DISPATCHER': lcmt_robot_output,
        'CASSIE_ELEVATION_MAP': lcmt_grid_map,
        'FOOTHOLDS_PROCESSED': lcmt_foothold_set
    }

    ports = {
        'NETWORK_CASSIE_STATE_DISPATCHER': receivers['state'].get_input_port(),
        'CASSIE_ELEVATION_MAP': receivers['grid_map'].get_input_port(),
        'FOOTHOLDS_PROCESSED': receivers['polygons'].get_input_port()
    }

    lcm_log = lcm.EventLog(logfile)
    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, '../perception_video_test.mp4', duration=30.0)


def make_pipeline_figures_from_map(grid_map: GridMap, q: np.ndarray, save_folder: str=''):

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
        grid_map
    )
    convex_polygons = decomposition.get_output_port().Eval(decomposition_context)

    plant_vis = MultiposeVisualizer(
        "examples/Cassie/urdf/cassie_v2_shells.urdf", 1, "")
    meshcat = plant_vis.GetMeshcat()

    map_vis = GridMapVisualizer(meshcat, 1, [])
    poly_vis = ConvexPolygonVisualizer(meshcat, 1)
    capture = MeshcatChromeCapture(meshcat=meshcat, window_size=(1080, 1080))
    plant_vis.DrawPoses(q)

    # wait for plant to load
    sleep(5)

    center = grid_map.getPosition()
    height = grid_map.atPosition("interpolated", center)
    poi = np.zeros((3,))
    poi[:2] = center.ravel()
    poi[2] = height - 0.25

    capture.look_at(poi, np.array([0, -2.0, 1.5]))

    for layer in grid_map.getLayers():
        map_vis.DrawGridMap(grid_map, [layer])
        meshcat.Flush()
        capture.grab(os.path.join(save_folder, f'{layer}_meshcat.png'))
        sleep(0.5)
        meshcat.Delete(f'grid_map_{layer}')

    poly_vis.DrawPolygons(convex_polygons)
    meshcat.Flush()
    capture.grab(os.path.join(save_folder, 'convex_polygons_meshcat.png'))

    utils.setup_plots()
    utils.save_matrix_plot('Elevation Map', grid_map['elevation'], save_folder)
    utils.save_matrix_plot('Segmentation', grid_map['segmentation'], save_folder)
    for name, data in segmentation.safety_scores.items():
        title = (name.replace('_', ' ') + ' score').title()
        utils.save_matrix_plot(title, data, save_folder)

    save_decomposition_debug_plots(decomposition.debug_info, save_folder)
    print('done')


def save_decomposition_debug_plots(debug_info, save_folder):

    def despine(ax):
        for _, spine in ax.spines.items():
            spine.set_visible(False)

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_polygons_with_holes(debug_info['unprocessed_polygons'])

    despine(ax)
    limits = utils.do_perception_fig_layout_and_save(
        ax, fig, 'Non-Convex Polygons', save_folder)

    fig, ax = plt.subplots(figsize=(8, 8))
    for c in debug_info['acd_components']:
        plot_polygon(c)
    despine(ax)
    utils.do_perception_fig_layout_and_save(
        ax, fig, 'Approximate Convex\nDecomposition', save_folder, limits)

    fig, ax = plt.subplots(figsize=(8, 8))
    despine(ax)
    for c in debug_info['convex_polygons']:
        plot_polygon(c.GetVertices()[:2, :])
    utils.do_perception_fig_layout_and_save(
        ax, fig, 'Convex Polygons', save_folder, limits)


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
    plt.yscale('log')
    plt.ylim([1e-3, 5e-1])
    plt.legend([r['name'] for r in results])
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def plot_iou_results(results, title, savefile):
    fig = plt.figure()
    #results = [results[0], results[1], results[0]]
    linestyles = ['-', '-.', '--']
    colors = ['blue', 'red', 'green']
    facecolrs = ['blue', 'orange', 'green']
    bins = np.histogram(np.hstack([r['iou'] for r in results]), bins=20)[1]
    plot_ordered_histograms(
        [r['iou'] for r in results],
        [r['name'] for r in results],
        'IoU With Previous Frame',
        'Number of Frames',
        title
    )
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def make_plane_segmentation_system_info(params_folder):
    yamls = [
        os.path.join(params_folder, f'{y}.yaml') for y in
        ['ransac_and_preprocessing', 'no_ransac_with_preprocessing']
    ]
    configs = ['_PR', '']
    system_names = ['EM_cupy' + config for config in configs]
    return zip(yamls, system_names)


def profile_worker_wrapper(args):
    sys_info, grid_maps = args
    grid_maps_copy = deepcopy(grid_maps)
    params, name = sys_info
    system = PlaneSegmentationSystem(params)
    system.set_name(name)
    return profile_segmentation(system, grid_maps_copy)


def run_segmentation_profiling(logfile):
    grid_maps, robot_outputs = get_grid_maps_from_log(logfile, duration=40)
    params_folder = 'bindings/pydairlib/perceptive_locomotion/terrain_segmentation/plane_segmentation_results_params/'
    s3 = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )

    args = [(sys_info, grid_maps) for sys_info in make_plane_segmentation_system_info(params_folder)]
    num_processes = len(args)
    # Create a process pool and map the function across all systems
    with Pool(processes=num_processes) as pool:
        results = pool.map(profile_worker_wrapper, args)

    results.append(profile_segmentation(s3, deepcopy(grid_maps)))

    utils.setup_plots()

    np.savez('../cached_results_for_plot_testing_2.npz', results=results)
    plot_segmentation_run_time_results(results, 'Run Time', None)
    plot_iou_results(results, 'Frame-to-Frame IoU', None)

    plt.show()


def run_pipeline_figure_script(logfile):
    example_idx = 0
    grid_maps, robot_output = get_grid_maps_from_log(logfile, start_time=0, duration=1)
    q = robot_output['q'][0]
    make_pipeline_figures_from_map(
        grid_maps[example_idx], q, '../terrain_seg_figures')


def plot_ordered_histograms(data_list, labels, xlabel, ylabel, title, bins=20, alpha=1.0, colors=None):
    """
    Plot multiple histograms with bars ordered by height within each bin.

    Parameters:
    data_list: List of arrays containing the data for each histogram
    labels: List of labels for each dataset
    bins: Number of bins or array of bin edges
    alpha: Transparency of bars
    colors: List of colors for each histogram (optional)
    """
    # Set default colors if none provided
    if colors is None:
        colors = plt.cm.viridis(np.linspace(0, 1, len(data_list)))

    # Calculate bin edges using the combined range of all datasets
    all_data = np.concatenate(data_list)
    bin_edges = np.histogram_bin_edges(all_data, bins=bins)

    # Calculate histograms for all datasets
    hists = []
    for data in data_list:
        hist, _ = np.histogram(data, bins=bin_edges)
        hists.append(hist)

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 6))

    # Width of bars
    width = (bin_edges[1] - bin_edges[0])

    # Plot histograms bin by bin
    for bin_idx in range(len(bin_edges) - 1):
        # Get heights for this bin from all histograms
        heights = [hist[bin_idx] for hist in hists]
        # Sort indices by height
        sorted_indices = np.argsort(heights)
        sorted_indices = reversed(sorted_indices)

        # Plot bars in order of height (Highest to lowest)
        x = bin_edges[bin_idx]
        for idx in sorted_indices:
            if heights[idx] > 0:  # Only plot if there's data
                ax.bar(x, heights[idx], width,
                       alpha=alpha,
                       color=colors[idx],
                       label=labels[idx] if bin_idx == 0 else "")

    # Customize plot
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()

    return fig, ax

def test_iou_plot():
    utils.setup_plots()
    data = np.load('../cached_results_for_plot_testing.npz', allow_pickle=True)
    results = data['results']
    plot_iou_results(results, 'Frame-to-Frame IoU', None)
    plt.show()


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfile', type=str)
    args = parser.parse_args()
    # run_pipeline_figure_script(args.logfile)
    # run_segmentation_profiling(args.logfile)
    # profile_full_pipeline(args.logfile)
    # write_perception_video(args.logfile)
    test_iou_plot()


if __name__ == '__main__':
    main()
