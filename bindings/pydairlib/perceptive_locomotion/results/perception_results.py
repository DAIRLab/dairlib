# standard library imports
import os
import time
import glob
import subprocess
from time import sleep
from copy import deepcopy
from functools import partial
from multiprocessing import Pool
from argparse import ArgumentParser

# installed
import cv2
import lcm
import yaml
import numpy as np

# Plotting
import matplotlib
import seaborn as sns
import matplotlib.pyplot as plt

# lcmtypes
from dairlib import(
    lcmt_grid_map,
    lcmt_foothold_set,
    lcmt_robot_output,
    lcmt_alip_s2s_mpfc_debug
)

# pydrake
from pydrake.systems.all import (
    Diagram,
    Context,
    DiagramBuilder,
    LcmPublisherSystem,
    TriggerType,
)
from pydrake.geometry import Meshcat


# Grid Map
from grid_map import GridMap

# pydairlib
from pydairlib.multibody import MultiposeVisualizer
from pydairlib.common import MeshcatChromeCapture, write_meshcat_video_from_log

from pydairlib.systems import (
    PlaneSegmentationSystem,
    GridMapVisualizer,
    GridMapReceiver,
    PlantVisualizer,
    RobotOutputReceiver
)
from pydairlib.geometry import ConvexPolygonVisualizer, ConvexPolygonReceiver

from pydairlib.perceptive_locomotion.terrain_segmentation import (
    ConvexTerrainDecompositionSystem,
    TerrainSegmentationSystem,
    plot_polygons_with_holes,
    plot_polygon,
    segmentation_criteria as seg_criteria
)

from pydairlib.perceptive_locomotion.results import perception_analysis_utils as utils
from pydairlib.perceptive_locomotion.systems import AlipMPFCMeshcatVisualizer

state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'


def write_mpfc_debug_video(logfile: str, duration=-1):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"

    theta = -2 * np.pi / 3
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 0.25]))
    meshcat = plant_visualizer.get_meshcat()
    mpc_visualizer = AlipMPFCMeshcatVisualizer(meshcat, plant_visualizer.get_plant())
    state_receiver = RobotOutputReceiver(plant_visualizer.get_plant())

    builder = DiagramBuilder()
    builder.AddSystem(plant_visualizer)
    builder.AddSystem(mpc_visualizer)
    builder.AddSystem(state_receiver)

    builder.Connect(state_receiver.get_output_port(),
                    plant_visualizer.get_input_port())
    builder.Connect(state_receiver.get_output_port(),
                    mpc_visualizer.get_input_port_state())

    diagram = builder.Build()

    types = {
        state_channel: lcmt_robot_output,
        terrain_channel: lcmt_foothold_set,
        mpfc_debug_channel: lcmt_alip_s2s_mpfc_debug
    }
    ports = {
        state_channel: state_receiver.get_input_port(),
        terrain_channel: mpc_visualizer.get_input_port_terrain(),
        mpfc_debug_channel: mpc_visualizer.get_input_port_mpc()
    }
    lcm_log = lcm.EventLog(logfile)
    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, '../mpc_debug_video_test.mp4', duration=duration)


def write_perception_meshcat_video(logfile: str, duration=60):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"
    update_period = 1.0 / 30.01
    theta = -2 * np.pi / 3
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 0.25]))
    meshcat = plant_visualizer.get_meshcat()

    visualizers = {
        "state": plant_visualizer,
        "grid_map": GridMapVisualizer(meshcat, update_period, ["segmented_elevation"]),
    }
    receivers = {
        "state": RobotOutputReceiver(plant_visualizer.get_plant()),
        "grid_map": GridMapReceiver(),
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
    }

    ports = {
        'NETWORK_CASSIE_STATE_DISPATCHER': receivers['state'].get_input_port(),
        'CASSIE_ELEVATION_MAP': receivers['grid_map'].get_input_port(),
    }

    lcm_log = lcm.EventLog(logfile)
    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, '../perception_video_test.mp4', duration=duration)


def make_pipeline_figures_from_map(grid_map: GridMap, q: np.ndarray, save_folder: str=''):

    segmentation = TerrainSegmentationSystem({
        'curvature_criterion': seg_criteria.curvature_criterion,
        'inclination_criterion': seg_criteria.inclination_criterion,
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
    def despine(_ax):
        for _, spine in _ax.spines.items():
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


def make_hist_figure(results, title, key, edges, log_x_axis=False):
    plt.title(title)
    for i, r in enumerate(results):
        alpha = 0.8 * (1.0 - float(i) / len(results))
        sns.histplot(
            data=r[key],
            bins=edges,
            element='step',
            alpha=alpha,
            stat='proportion'
        )

    if log_x_axis:
        plt.gca().set_xscale('log')
    if title == 'Lab':
        plt.legend([r['name'] for r in results])
    if title == 'Grass':
        plt.xlabel('Segmentation Run Time (s)')
    else:
        plt.xticks([], [])
        plt.minorticks_off()


def plot_segmentation_run_time_results(results, title, savefile):
    fig = plt.figure()
    edges = np.logspace(np.log10(1e-3), np.log10(3e-1), 21)
    make_hist_figure(results, title, 'runtime', edges, log_x_axis=True)
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def plot_iou_results(results, title, savefile):
    fig = plt.figure()
    edges = np.linspace(0, 1, 21)
    make_hist_figure(results, title, 'iou', edges)
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def make_segmentation_videos(logfile, start_time, duration, env_name=''):
    results = utils.run_segmentation_comparison_on_log(
        logfile,
        start_time=start_time,
        duration=duration
    )
    for r in results:
        utils.write_arrays_to_video(
            [cv2.resize(255 * s.astype(np.uint8), (300, 300), cv2.INTER_NEAREST_EXACT) for s in r['segmentations']],
            f'../segmentation_videos/{env_name}_{r["name"]}_segmentation.mp4'
        )


def segmentation_comparison_results_runner(env_config, logfolder):
    results = utils.run_segmentation_comparison_on_log(
        logfile=os.path.join(logfolder, env_config['log']),
        start_time=env_config['start'],
        duration=env_config['duration']
    )
    return results


def save_all_results(logfolder):
    runner = partial(segmentation_comparison_results_runner, logfolder=logfolder)
    with open(os.path.join(logfolder, 'results_config.yaml')) as stream:
        config = yaml.safe_load(stream)
    with Pool(4) as pool:
        results = pool.map(runner, config.values())

    data = dict(zip(config.keys(), results))
    np.savez(os.path.join(logfolder, 'processed_results.npz'), data=data)


def make_all_segmentation_videos(logfolder):
    with open(os.path.join(logfolder, 'results_config.yaml')) as stream:
        config = yaml.safe_load(stream)

    for env, env_config in config.items():
        make_segmentation_videos(
            logfile=os.path.join(logfolder, env_config['log']),
            start_time=env_config['start'],
            duration=env_config['duration'],
            env_name=env
        )


def plot_segmentation_profiling(results, env_name='', save_prefix=None):
    print(f'Generating profiling results for {env_name}')
    utils.setup_plots()

    run_time_save = save_prefix + '_run_time.svg' if save_prefix is not None else None
    iou_save = save_prefix + '_iou.svg' if save_prefix is not None else None
    plot_segmentation_run_time_results(results, env_name, run_time_save)
    plot_iou_results(results, env_name, iou_save)

    if save_prefix is None:
        plt.show()


def make_all_results_figures(logfolder, savefolder):
    all_results = np.load(os.path.join(logfolder, 'processed_results.npz'), allow_pickle=True)
    data = all_results['data'].item()
    for env, results in data.items():
        plot_segmentation_profiling(
            results,
            env_name=env,
            save_prefix=savefolder + env.replace(' ', '_')
        )
    crop_perception_results_svgs(savefolder)


def make_segmentation_tiles(logfolder, savefolder):
    all_results = np.load(os.path.join(logfolder, 'processed_results.npz'), allow_pickle=True)
    data = all_results['data'].item()
    N = 16
    offset = 1050
    interval = 30
    envs = ['Lab', 'Brick Steps', 'Grass']
    utils.setup_plots()
    matplotlib.rcParams['axes.titlesize'] = 12
    for env in envs:
        fig = plt.figure(figsize=(4, N))
        gs = fig.add_gridspec(N, 4)
        plt.suptitle(f'{env}')
        for i in range(N):
            grid_map = data[env][0]['grid_maps'][offset + interval * i]
            ax = fig.add_subplot(gs[i, 0])
            if i == 0:
                ax.set_title('Elevation')
            ax.imshow(grid_map['elevation'])
            ax.set_xticks([])
            ax.set_yticks([])
            if env == 'Lab':
                ax.set_ylabel('\\textbf{' + f't = {(offset + interval * i)/30}' + '}', fontsize=12)
            for j in range(3):
                seg = data[env][j]['segmentations'][offset + + interval * i]
                name = data[env][j]['name']
                ax = fig.add_subplot(gs[i, j + 1])
                if i == 0:
                    ax.set_title(name)
                ax.imshow(seg, cmap='gray')
                ax.set_xticks([])
                ax.set_yticks([])
        fig.tight_layout(pad=1.01)
        plt.savefig(os.path.join(savefolder, f'{env.replace(" ", "_")}_segmentation_tiles.svg'))
    plt.show()


def crop_perception_results_svgs(savefolder):
    current = os.getcwd()
    os.chdir(savefolder)
    files = glob.glob('*.svg')
    for f in files:
        subprocess.run(['inkscape', '--export-type=svg', '--export-id=axes_1', f, '-o', f])


def run_pipeline_figure_script(logfile):
    example_idx = 0
    grid_maps, robot_output = utils.get_grid_maps_from_log(
        logfile,
        start_time=0,
        duration=1
    )
    q = robot_output['q'][0]
    make_pipeline_figures_from_map(
        grid_maps[example_idx], q, '../terrain_seg_figures')


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfolder', type=str, default='')
    parser.add_argument('--logfile', type=str, default='')

    args = parser.parse_args()

    # make_all_segmentation_videos(args.logfolder)

    # run_pipeline_figure_script(args.logfile)
    # save_all_results(args.logfolder)
    # make_segmentation_tiles(
    #     args.logfolder,
    #     '../manuscripts/perceptive_walking_tro/figures/perception_results/'
    # )
    # make_all_results_figures(
    #     args.logfolder,
    #     '../manuscripts/perceptive_walking_tro/figures/perception_results/'
    # )
    #
    # profile_full_pipeline(args.logfile)
    # write_perception_meshcat_video(args.logfile, duration=60.0)
    # write_mpfc_debug_video(args.logfile, 60)
    # test_iou_plot()


if __name__ == '__main__':
    main()
