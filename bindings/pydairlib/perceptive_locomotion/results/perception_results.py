# standard library imports
import os
import time
import glob
import shutil
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
    lcmt_alip_s2s_mpfc_debug,
    lcmt_alip_mpfc_debug_complete,
)

# pydrake
from pydrake.systems.all import (
    System,
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
from pydairlib.multibody import MultiposeVisualizer, AddSteppingStonesToMeshcatFromYaml
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

from pydairlib.perceptive_locomotion.results import analysis_utils as utils
from pydairlib.perceptive_locomotion.systems import AlipMPFCMeshcatVisualizer

state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'

# Outputs
output_folder = 'perceptive_locomotion_results_figures/'

precomputed_results_fname = 'precomputed_segmentation_results.npz'


def check_mpfc_debug_version(event):
    try:
        lcmt_alip_mpfc_debug_complete.decode(event.data)
    except:
        return lcmt_alip_s2s_mpfc_debug

    return lcmt_alip_mpfc_debug_complete


def write_sim_log_video(logfile: str, savefile: str, terrain_yaml: str):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"

    theta = -2 * np.pi / 3
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 0.25]))
    meshcat = plant_visualizer.get_meshcat()
    state_receiver = RobotOutputReceiver(plant_visualizer.get_plant())
    AddSteppingStonesToMeshcatFromYaml(meshcat, terrain_yaml)

    builder = DiagramBuilder()
    builder.AddSystem(plant_visualizer)
    builder.AddSystem(state_receiver)

    builder.Connect(state_receiver.get_output_port(),
                    plant_visualizer.get_input_port())

    diagram = builder.Build()

    # we have multiple debug types - check which one is being used
    lcm_log = lcm.EventLog(logfile)

    types = {
        state_channel: lcmt_robot_output,
    }

    ports = {
        state_channel: state_receiver.get_input_port(),
    }

    lcm_log = lcm.EventLog(logfile)
    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, savefile, start=0.0, duration=-1)


def write_mpfc_debug_video(logfile: str, savefile: str, duration=-1, terrain_yaml=None):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"

    theta = -2 * np.pi / 3
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 0.25]))
    meshcat = plant_visualizer.get_meshcat()
    mpc_visualizer = AlipMPFCMeshcatVisualizer(meshcat, plant_visualizer.get_plant())
    state_receiver = RobotOutputReceiver(plant_visualizer.get_plant())

    if terrain_yaml is not None:
        AddSteppingStonesToMeshcatFromYaml(meshcat, terrain_yaml)

    builder = DiagramBuilder()
    builder.AddSystem(plant_visualizer)
    builder.AddSystem(mpc_visualizer)
    builder.AddSystem(state_receiver)

    builder.Connect(state_receiver.get_output_port(),
                    plant_visualizer.get_input_port())
    builder.Connect(state_receiver.get_output_port(),
                    mpc_visualizer.get_input_port_state())

    diagram = builder.Build()

    # we have multiple debug types - check which one is being used
    lcm_log = lcm.EventLog(logfile)
    mpfc_debug_type = None
    for event in lcm_log:
        if event.channel == mpfc_debug_channel:
            mpfc_debug_type = check_mpfc_debug_version(event)
            break

    types = {
        state_channel: lcmt_robot_output,
        mpfc_debug_channel: mpfc_debug_type
    }

    ports = {
        state_channel: state_receiver.get_input_port(),
        mpfc_debug_channel: mpc_visualizer.get_input_port_mpc() if
                            mpfc_debug_type == lcmt_alip_mpfc_debug_complete else
                            mpc_visualizer.get_input_port_mpc_legacy()
    }

    if terrain_yaml is None:
        types[terrain_channel] = lcmt_foothold_set
        ports[terrain_channel] = mpc_visualizer.get_input_port_terrain()

    lcm_log = lcm.EventLog(logfile)
    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, savefile, start=0.0, duration=duration)


def write_grid_map_meshcat_video(logfile: str, savefile: str, layer="segmented_elevation", duration=60):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"
    update_period = 1.0 / 30.01
    theta = -2 * np.pi / 3
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 0.25]))
    meshcat = plant_visualizer.get_meshcat()

    visualizers = {
        "state": plant_visualizer,
        "grid_map": GridMapVisualizer(meshcat, update_period, [layer]),
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
        diagram, lcm_log, meshcat, types, ports, savefile, duration=duration)


def write_segmentation_results_video(segmenter: System, logfile: str, savefile: str, start=0.0, duration=60.0):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"
    update_period = 1.0 / 30.01
    theta = -np.pi
    r = 0.2
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([r * np.cos(theta), r * np.sin(theta), 2.0]))
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
    builder.AddSystem(segmenter)
    for key in ['state', 'grid_map']:
        builder.AddSystem(visualizers[key])
        builder.AddSystem(receivers[key])

    builder.Connect(
        receivers['state'].get_output_port(),
        visualizers['state'].get_input_port()
    )
    builder.Connect(
        receivers['grid_map'].get_output_port(),
        segmenter.get_input_port()
    )
    builder.Connect(
        segmenter.get_output_port(),
        visualizers['grid_map'].get_input_port()
    )
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
        diagram, lcm_log, meshcat, types, ports, savefile, start=start, duration=duration,
        window_size=(1000, 1000)
    )


def moving_obstacle_experiments_videos(logfile: str, savefolder: str, start: float, duration: float):
    hysts = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    logname = logfile.split('/')[-1].strip().lower()

    for khyst in hysts:
        segmentation = TerrainSegmentationSystem({
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        })
        segmentation.safety_hysteresis = khyst
        savename = os.path.join(savefolder, f'{logname}_khyst_{khyst:0.1f}.mp4')
        write_segmentation_results_video(
            segmenter=segmentation,
            logfile=logfile,
            savefile=savename,
            start=start,
            duration=duration
        )


def make_pipeline_figures_from_map(grid_map: GridMap, q: np.ndarray, save_folder: str=''):

    segmentation = TerrainSegmentationSystem({
        'curvature_criterion': seg_criteria.curvature_criterion,
        'inclination_criterion': seg_criteria.inclination_criterion,
    })
    segmentation.safety_hysteresis = 0.0
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

    grid_map = segmentation.get_output_port().Eval(segmentation_context)

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
        map_vis.DrawGridMap(grid_map, [layer], "")
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


def make_hist_figure(ax, results, title, key, edges, log_x_axis=False):
    ax.set_title(title)
    for i, r in enumerate(results):
        alpha = 0.8 * (1.0 - float(i) / len(results))
        sns.histplot(
            data=r[key],
            bins=edges,
            element='step',
            alpha=alpha,
            stat='proportion',
            ax=ax
        )

    if log_x_axis:
        ax.set_xscale('log')
    if title in ['Grass'] and log_x_axis:
        ax.legend([r['name'] for r in results])
    if title in ['Grass', 'Brick Steps']:
        ax.set_yticks([], [])
        ax.set_ylabel('')
        ax.minorticks_off()


def plot_segmentation_run_time_results(results, title, savefile):
    fig = plt.figure()
    edges = np.logspace(np.log10(1e-3), np.log10(3e-1), 21)
    make_hist_figure(results, title, 'runtime', edges, log_x_axis=True)
    if title == 'Grass':
        plt.xlabel('Segmentation Run Time (s)')

    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def plot_iou_results(results, title, savefile, edges=None):
    fig = plt.figure()
    if edges is None:
        edges = np.linspace(0, 1, 21)
    make_hist_figure(results, title, 'iou', edges)
    if title == 'Grass':
        plt.xlabel('Frame-to-Frame IoU')
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def plot_iou_vs_poly_iou(results, title, savefile):
    plt.figure(figsize=(10,6))
    edges = np.linspace(0.75, 1.0, 20)
    for i, k in enumerate(['poly_iou', 'iou']):
        alpha = 0.8 * (1.0 - float(i) / len(results))
        sns.histplot(
            data=results[k],
            bins=edges,
            element='step',
            alpha=alpha,
            stat='proportion'
        )
    plt.legend(['Convex Decomposition IoU', 'Segmentation IoU'])
    plt.title(title)
    plt.xlabel('Frame-to-Frame IoU')
    plt.savefig(savefile)


def make_segmentation_videos(logfile, start_time, duration, env_name=''):
    systems = utils.make_segmentation_systems()
    save_folder = os.path.join(output_folder, 'segmentation_videos')
    utils.make_dir_if_missing(save_folder)

    for s in systems:
        savefile = os.path.join(save_folder, f'{env_name}_{s.get_name()}.mp4')
        write_segmentation_results_video(s, logfile, savefile, start=start_time, duration=duration)


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
    # make single threaded for evaluation to avoid threads
    # influencing each others' performance
    with Pool(1) as pool:
        results = pool.map(runner, config.values())

    data = dict(zip(config.keys(), results))

    # save processed results to the output folder to avoid overwriting source
    np.savez(
        os.path.join(output_folder, precomputed_results_fname),
        data=data
    )


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


def make_combined_iou_results_results_figures(logfolder, savefolder):
    
    all_results = np.load(os.path.join(logfolder, precomputed_results_fname), allow_pickle=True)
    data = all_results['data'].item()
    utils.setup_plots()
    
    # Create a figure with 2 rows and 3 columns
    fig, axs = plt.subplots(2, 3, figsize=(20, 10))
    
    # Runtime edges for log scale
    runtime_edges = np.logspace(np.log10(1e-3), np.log10(3e-1), 21)
    # IoU edges
    iou_edges = np.linspace(0.5, 1, 21)
    
    # Plot each environment in its own column
    col = 0
    for env, results in data.items():
        make_hist_figure(axs[0, col], results, env, 'runtime', runtime_edges, log_x_axis=True)
        make_hist_figure(axs[1, col], results, env, 'iou', iou_edges)
        
        # Add x-axis labels only to the bottom row
        if col == 1:
            axs[1, col].set_xlabel('Frame-to-Frame IoU')
            axs[0, col].set_xlabel('Segmentation Run Time (s)')

        col += 1
    # Adjust layout
    plt.tight_layout()
    
    # Save figure if prefix is provided
    if savefolder is not None:
        plt.savefig(
            os.path.join(savefolder,  's3_results_combined.svg')
        )
    else:
        plt.show()


def make_all_results_figures(logfolder, savefolder):
    all_results = np.load(os.path.join(logfolder, precomputed_results_fname), allow_pickle=True)
    data = all_results['data'].item()
    for env, results in data.items():
        plot_segmentation_profiling(
            results,
            env_name=env,
            save_prefix=os.path.join(savefolder, env.replace(' ', '_'))
        )
    crop_perception_results_svgs(savefolder)


def make_convex_polygon_iou_figure(logfolder, savefolder, env_name):
    all_results = np.load(os.path.join(logfolder, precomputed_results_fname), allow_pickle=True)
    data = all_results['data'].item()
    results = data[env_name][-1]
    assert results['name'] == 'S3 (Ours)'

    utils.calc_polygon_iou(results)
    utils.setup_plots()
    plot_iou_vs_poly_iou(
        results,
        f'Convex Decomposition IoU: {env_name}',
        os.path.join(savefolder, f'Decomposition_IoU_{env_name.replace(" ", "_")}.svg')
    )


def make_segmentation_tiles(logfolder, savefolder):
    all_results = np.load(os.path.join(logfolder, precomputed_results_fname), allow_pickle=True)
    data = all_results['data'].item()
    N = 10
    offset = 600
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


def crop_perception_results_svgs(savefolder):
    current = os.getcwd()
    os.chdir(savefolder)
    files = glob.glob('*.svg')
    for f in files:
        subprocess.run(['inkscape', '--export-type=svg', '--export-id=axes_1', f, '-o', f])
    os.chdir(current)


def run_pipeline_figure_script(logfile):
    example_idx = 0
    grid_maps, robot_output = utils.get_grid_maps_from_log(
        logfile,
        start_time=0,
        duration=1
    )
    q = robot_output['q'][0]

    save_folder = os.path.join(output_folder, 'terrain_seg_figures')
    utils.make_dir_if_missing(save_folder)

    make_pipeline_figures_from_map(
        grid_maps[example_idx], q, save_folder)


def make_full_profiling_plot(logfile, savefile):
    mapping_results = utils.get_elevation_map_profiling(logfile)
    results = utils.profile_full_perception_pipeline(logfile)
    worst_case = utils.get_worst_case_data_by_num_polygons([results])

    data = {
        'elevation_mapping':  np.max(mapping_results) * np.ones_like(worst_case['segmentation'])
    }
    data.update(worst_case)

    utils.setup_plots()
    # Prepare the plot
    fig, ax = plt.subplots(figsize=(10, 6))

    # X-axis values (sorted unique num_polygons)
    x_values = sorted(set(results['num_polygons']))

    # Keep track of the bottom of each stack for cumulative plotting
    bottom = np.zeros(len(x_values))

    merged = ['elevation_mapping', 'segmentation']

    legend_labels = {
        'segmentation': 'S3',
        'elevation_mapping': 'Elevation Mapping',
        'decomposition': 'Convex Decomposition',
        'plane_fitting': 'Plane Fitting'
    }

    alpha = 0.7

    # Plot each label as a segment
    for label in data:
        if label in merged:
            width = x_values[-1] - x_values[0] + 1
            center = 0.5 * (x_values[0] + x_values[-1])
            ax.bar(
                center,
                np.max(data[label]),
                width=width,
                linewidth=0.5,
                edgecolor='black',
                bottom=bottom[0],
                label=legend_labels[label],
                alpha=alpha,
            )
            bottom += np.max(data[label])
        else:
            # Plot the bar segment
            ax.bar(
                x_values,
                data[label],
                width=1.0,
                linewidth=0.5,
                edgecolor='black',
                bottom=bottom,
                label=legend_labels[label],
                alpha=alpha
            )

            # Update the bottom for the next segment
            bottom += data[label]

    ax.set_xlabel('Number of Resulting Polygons')
    ax.set_ylabel('Worst Case Runtime (s)')
    ax.set_title('Perception Stack Detailed Profiling')
    ax.legend(title='Operation', framealpha=1, loc='lower right')

    plt.tight_layout()
    plt.savefig(savefile)


def plot_hysteresis_comparison(logfile, savefile=None):
    utils.setup_plots()
    results = utils.hysteresis_comparison(logfile)
    edges = np.linspace(np.min(results[0]['iou']), 1, 16)
    fig = plt.figure()
    ax = plt.gca()
    make_hist_figure(ax, results, None, 'iou', edges)
    ax.legend([r['name'] for r in results])
    plt.title('Temporal Consistency vs $k_{hyst}$')
    plt.xlabel('Frame-to-Frame IoU')
    fig.tight_layout()
    if savefile is not None:
        plt.savefig(savefile)


def acknowledge_rerun_option(rerun: bool):
    message = ''
    if rerun:
        message = (
            f'You have chosen to re-run the segmentation analysis.\n'
            f'The results comparing IoU and run time against EM_cupy'
            f' and EM_cupy_NR will be recomputed and saved to '
            f'{output_folder}{precomputed_results_fname}.\n'
            f'The timing results, may not exactly match those reported in'
            f' the paper due to system differences.'
        )
    else:
        message = (
            f'You are using the pre-computed results to generate the IoU '
            f'and run time comparison against EM_cupy and EM_cupy_NR,'
            f' and the perception stack detailed profiling. \n'
            f'To recompute the comparisons, rerun this '
            f'script with the option --rerun_plane_segmentation_comparisons'
        )

    message = message + (
        '\n\nThe non-elevation mapping full stack profiling results will be '
        'recomputed for either option. \n'
        'The elevation mapping profiling results are computed '
        'online and saved to the log to avoid the overhead of publishing and '
        'logging point cloud data\n')

    print(message)
    response = input('Continue? (y/[n]): ')

    if response.lower().strip() != 'y':
        print('exiting.')
        exit(0)


def moving_obstacle_videos(args):
    moving_obstacle_experiments_videos(
        logfile=os.path.join(args.data_root, 'lcmlog-laptop-moving-obstacle-standing'),
        savefolder=os.path.join(args.data_root, 'moving_obstacle_videos'),
        start=5.0,
        duration=8.0,
    )
    moving_obstacle_experiments_videos(
        logfile=os.path.join(args.data_root, 'lcmlog-laptop-moving-obstacle-walking'),
        savefolder=os.path.join(args.data_root, 'moving_obstacle_videos'),
        start=4.0,
        duration=10.0,
    )


def simulation_videos(data_root):
    terrain_folder = 'bindings/pydairlib/perceptive_locomotion/sim_experiments/terrains/'
    log_folder = os.path.join(data_root, 'sim_experiment_logs/')
    out_folder = os.path.join(data_root, 'simulation_videos/')

    perceptive_terrains = ['perceptive_stones', 'perceptive_beam', 'perceptive_stairs', 'perceptive_sine']
    for terrain in perceptive_terrains:
        write_mpfc_debug_video(
            logfile=os.path.join(log_folder, terrain),
            savefile=os.path.join(out_folder, f'{terrain}_mpfc_debug.mp4')
        )
        write_grid_map_meshcat_video(
            logfile=os.path.join(log_folder, terrain),
            savefile=os.path.join(out_folder, f'{terrain}_segmentation.mp4')
        )
        write_grid_map_meshcat_video(
            logfile=os.path.join(log_folder, terrain),
            savefile=os.path.join(out_folder, f'{terrain}_elevation.mp4'),
            layer="elevation"
        )

    global state_channel
    state_channel = "CASSIE_STATE_SIMULATION"

    for terrain in perceptive_terrains:
        write_sim_log_video(
            logfile=os.path.join(log_folder, terrain),
            savefile=os.path.join(out_folder, f'{terrain}_boxy.mp4'),
            terrain_yaml=os.path.join(terrain_folder, f'{terrain}.yaml')
        )

    for terrain in ['stones', 'beam', 'stairs']:
        write_mpfc_debug_video(
            logfile=os.path.join(log_folder, terrain),
            savefile=os.path.join(out_folder, f'{terrain}.mp4'),
            terrain_yaml=os.path.join(terrain_folder, f'{terrain}.yaml')
        )


def cost_comparison_videos(data_root):
    terrain_folder = 'bindings/pydairlib/perceptive_locomotion/sim_experiments/terrains/'
    log_folder = os.path.join(data_root, 'sim_experiment_logs/')
    out_folder = os.path.join(data_root, 'simulation_videos/')

    write_mpfc_debug_video(
        logfile=os.path.join(log_folder, 'stairs_gait_cost'),
        savefile=os.path.join(out_folder, 'stairs_gait_cost.mp4'),
        terrain_yaml=os.path.join(terrain_folder, 'perceptive_stairs.yaml')
    )
    write_mpfc_debug_video(
        logfile=os.path.join(log_folder, 'stairs_vel_cost'),
        savefile=os.path.join(out_folder, 'stairs_vel_cost.mp4'),
        terrain_yaml=os.path.join(terrain_folder, 'perceptive_stairs.yaml')
    )


def get_user_results_gen_choice():
    option = input('\nWhich results should be generated?\n1: Simulation Videos'
                   '\n2: Moving obstacle videos\n3: Hardware perception results (Figures Only)'
                   '\n4: Hardware perception results (Figures and Videos)\n\n Selection: ')
    return int(option)


def paper_main(args):
    if not args.data_root:
        raise RuntimeError(
            "Please provide the location of the data via the"
            " --data_root argument"
        )

    results_selection = get_user_results_gen_choice()
    if results_selection == 1:
        simulation_videos(args.data_root)
        exit(0)
    if results_selection == 2:
        moving_obstacle_videos(args)
        exit(0)

    acknowledge_rerun_option(args.rerun_plane_segmentation_comparisons)

    # define relevant data files
    hysteresis_comparison_log = os.path.join(
        args.data_root, 'brick_steps/lcmlog-laptop-21'
    )
    detailed_profiling_log = os.path.join(
        args.data_root, 'elevation_mapping_profiling_log/lcmlog-laptop-16'
    )
    demo_reel_log = os.path.join(
        args.data_root, 'other/stairs_and_grass/lcmlog-laptop-01'
    )
    pipeline_figure_log = os.path.join(args.data_root, 'lcmlog-vision-demo-sim')

    # Run the figure scripts
    plot_hysteresis_comparison(
        hysteresis_comparison_log,
        os.path.join(output_folder, 'hysteresis_comparison.svg')
    )

    results_folder = args.data_root

    # If recomputing, we put the recomputed resutls into the output folder
    if args.rerun_plane_segmentation_comparisons:
        save_all_results(args.data_root)
        results_folder = output_folder

    make_convex_polygon_iou_figure(args.data_root, output_folder, 'Brick Steps')
    make_combined_iou_results_results_figures(results_folder, output_folder)
    make_segmentation_tiles(results_folder, output_folder)
    run_pipeline_figure_script(pipeline_figure_log)

    make_full_profiling_plot(
        detailed_profiling_log,
        os.path.join(output_folder, 'perception_stack_full_profiling.svg')
    )

    if results_selection == 4:
        make_all_segmentation_videos(args.data_root)
        write_grid_map_meshcat_video(
            demo_reel_log,
            os.path.join(output_folder, 'segmented_elevation_demo_animation.mp4'),
            duration=60
        )
        write_mpfc_debug_video(
            demo_reel_log,
            os.path.join(output_folder, 'mpfc_output_demo_animation.mp4'),
            duration=60
        )


def scratch_main(args):
    make_all_segmentation_videos(args.data_root)


def main():
    parser = ArgumentParser()
    parser.add_argument(
        '--data_root',
        type=str,
        default=''
    )
    parser.add_argument(
        '--rerun_plane_segmentation_comparisons',
        type=bool,
        default=False
    )
    args = parser.parse_args()
    paper_main(args)


if __name__ == '__main__':
    main()
