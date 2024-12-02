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
from matplotlib.animation import FuncAnimation

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

from pydairlib.analysis.process_lcm_log import get_log_data

from pydairlib.perceptive_locomotion.terrain_segmentation import (
    ConvexTerrainDecompositionSystem,
    TerrainSegmentationSystem,
    plot_polygons_with_holes,
    plot_polygon,
    perception_analysis_utils as utils,
    segmentation_criteria as seg_criteria
)

from pydairlib.perceptive_locomotion.systems import AlipMPFCMeshcatVisualizer

state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'


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


def write_mpfc_debug_video(logfile: str, duration=-1):
    urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf"

    theta = -np.pi
    r = 1.7
    plant_visualizer = PlantVisualizer(urdf, "pelvis", np.array([np.cos(theta), np.sin(theta), 0.5]))
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
    plant_visualizer = PlantVisualizer(urdf, "pelvis")
    meshcat = plant_visualizer.get_meshcat()

    visualizers = {
        "state": plant_visualizer,
        "grid_map": GridMapVisualizer(meshcat, update_period, ["segmented_elevation"]),
        "polygons": ConvexPolygonVisualizer(meshcat, update_period)
    }
    receivers = {
        "state": RobotOutputReceiver(plant_visualizer.get_plant()),
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


def despine(ax):
    for _, spine in ax.spines.items():
        spine.set_visible(False)


def save_decomposition_debug_plots(debug_info, save_folder):
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

        seg = process_grid_maps[-1]['segmentation']

        # make binary (0 to 1)
        seg = seg / np.maximum(1.0, seg.max())
        segmentations.append(seg)
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
        'segmentations': segmentations,
        'grid_maps': grid_maps
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

    edges = np.logspace(np.log10(1e-3), np.log10(3e-1), 21)

    for i, r in enumerate(results):
        alpha = 0.8 * (1.0 - float(i) / len(results))
        sns.histplot(data=r['runtime'], bins=edges, element='step', alpha=alpha, stat='proportion')

    plt.gca().set_xscale('log')
    if title == 'Grass':
        plt.xlabel('Segmentation Run Time (s)')
    else:
        plt.xticks([], [])
        plt.minorticks_off()

    if title == 'Lab':
        plt.legend([r['name'] for r in results])
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def plot_iou_results(results, title, savefile):
    fig = plt.figure()
    plt.title(title)
    # results = reversed(results)

    edges = np.linspace(0, 1, 21)
    for i, r in enumerate(results):
        alpha = 0.8 * (1.0 - float(i) / len(results))
        sns.histplot(data=r['iou'], bins=edges, element='step', alpha=alpha, stat='proportion')

    if title == 'Grass':
        plt.xlabel('IoU With Previous Segmentation')
    else:
        plt.xticks([], [])

    if title == 'Lab':
        plt.legend([r['name'] for r in results])
    fig.tight_layout()
    if savefile:
        plt.savefig(savefile)


def make_plane_segmentation_system_info(params_folder):
    yamls = [
        os.path.join(params_folder, f'{y}.yaml') for y in
        ['ransac_and_preprocessing', 'no_ransac_with_preprocessing']
    ]
    configs = ['', '_NR']
    system_names = ['EM_cupy' + config for config in configs]
    return zip(yamls, system_names)


def profile_worker_wrapper(args):
    sys_info, grid_maps = args
    grid_maps_copy = deepcopy(grid_maps)
    params, name = sys_info
    system = PlaneSegmentationSystem(params)
    system.set_name(name)
    return profile_segmentation(system, grid_maps_copy)


def get_segmentation_results(logfile, duration, start_time=0):
    grid_maps, _ = get_grid_maps_from_log(logfile, start_time=start_time, duration=duration)
    params_folder = \
        'bindings/pydairlib/perceptive_locomotion/terrain_segmentation/plane_segmentation_results_params/'
    s3 = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    s3.set_name('S3 (Ours)')

    args = [(sys_info, grid_maps) for sys_info in make_plane_segmentation_system_info(params_folder)]
    results = [profile_worker_wrapper(arg) for arg in args]
    results.append(profile_segmentation(s3, deepcopy(grid_maps)))

    return results


def make_segmentation_videos(logfile, start_time, duration, env_name=''):
    results = get_segmentation_results(logfile, start_time=start_time, duration=duration)
    for r in results:
        utils.write_arrays_to_video(
            [cv2.resize(255 * s.astype(np.uint8), (300, 300), cv2.INTER_NEAREST_EXACT) for s in r['segmentations']],
            f'../segmentation_videos/{env_name}_{r["name"]}_segmentation.mp4'
        )


def results_runner(env_config, logfolder):
    results = get_segmentation_results(
        logfile=os.path.join(logfolder, env_config['log']),
        start_time=env_config['start'],
        duration=env_config['duration']
    )
    return results


def save_all_results(logfolder):
    runner = partial(results_runner, logfolder=logfolder)
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
    grid_maps, robot_output = get_grid_maps_from_log(logfile, start_time=0, duration=1)
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
    write_mpfc_debug_video(args.logfile, -1)
    # test_iou_plot()


if __name__ == '__main__':
    main()
