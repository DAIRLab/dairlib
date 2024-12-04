import os
import lcm
import time
import tempfile
import subprocess
import matplotlib
import numpy as np
from PIL import Image
from copy import deepcopy
from grid_map import GridMap
from matplotlib import pyplot as plt
from pydairlib.analysis.mbp_plotting_utils import process_state_channel
from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context

# lcmtypes
from dairlib import(
    lcmt_grid_map,
    lcmt_foothold_set,
    lcmt_robot_output,
    lcmt_alip_s2s_mpfc_debug
)

from pydairlib.systems import (
    PlaneSegmentationSystem
)

from pydairlib.perceptive_locomotion.terrain_segmentation import (
    ConvexTerrainDecompositionSystem,
    TerrainSegmentationSystem,
    plot_polygons_with_holes,
    plot_polygon,
    segmentation_criteria as seg_criteria
)

from pydairlib.analysis.process_lcm_log import get_log_data

state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'
plotting_palette = ["#011f5b", "#9f642d", "#af0000", "#b99aa0", "#666666", "#5583ab"]


def get_grid_maps_from_log(logfile: str, start_time=0, duration=-1):
    log = lcm.EventLog(logfile, "r")
    grid_maps, robot_output = get_log_data(
        log,
        {
            elevation_map_channel: lcmt_grid_map,
            state_channel: lcmt_robot_output
        }, start_time, duration,
        process_grid_maps,
        elevation_map_channel,
        state_channel,
    )
    return grid_maps, robot_output


def profile_worker_wrapper(args):
    sys_info, grid_maps = args
    grid_maps_copy = deepcopy(grid_maps)
    params, name = sys_info
    system = PlaneSegmentationSystem(params)
    system.set_name(name)
    return profile_segmentation(system, grid_maps_copy)


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
            safe_terrain_iou(process_grid_maps[i], process_grid_maps[i+1])
        )

    return {
        'name': system.get_name(),
        'iou': iou,
        'runtime': runtime,
        'segmentations': segmentations,
        'grid_maps': grid_maps
    }


def run_segmentation_comparison_on_log(logfile, duration, start_time=0):
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

    def make_plane_segmentation_system_info(params_folder):
        yamls = [
            os.path.join(params_folder, f'{y}.yaml') for y in
            ['ransac_and_preprocessing', 'no_ransac_with_preprocessing']
        ]
        configs = ['', '_NR']
        system_names = ['EM_cupy' + config for config in configs]
        return zip(yamls, system_names)

    args = [(sys_info, grid_maps) for sys_info in make_plane_segmentation_system_info(params_folder)]
    results = [profile_worker_wrapper(arg) for arg in args]
    results.append(profile_segmentation(s3, deepcopy(grid_maps)))

    return results


def safe_terrain_iou(frame0: GridMap, frame1: GridMap, layer='segmentation'):
    # move frame0 to remove any pixels which the maps do not have in common
    frame0.move(frame1.getPosition())

    frame0_safe = np.nan_to_num(frame0[layer]).astype(bool)
    frame1_safe = np.nan_to_num(frame1[layer]).astype(bool)

    intersection = np.logical_and(frame0_safe,  frame1_safe)
    union = np.logical_or(frame0_safe, frame1_safe)

    if union.sum() == 0:
        return 0

    return float(intersection.sum()) / float(union.sum())


def write_arrays_to_video(sequence, video_out_path):
    with tempfile.TemporaryDirectory() as folder:
        for frame, data in enumerate(sequence):
            im = Image.fromarray(data)
            frame_filename = os.path.join(folder, f"frame_{frame:06d}.png")
            im.save(frame_filename)
        subprocess.run([
            'ffmpeg',
            '-framerate',
            '30',
            '-i',
            os.path.join(folder, f"frame_%06d.png"),
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            video_out_path
        ], check=True)


def process_grid_maps(data_dict, elevation_map_channel, state_channel):
    map_msgs = data_dict[elevation_map_channel]
    robot_output_msgs = data_dict[state_channel]

    plant, _ = make_plant_and_context()
    robot_output = process_state_channel(robot_output_msgs, plant)

    layers = map_msgs[0].layer_names
    grid_maps = [GridMap(layers) for _ in range(len(map_msgs))]
    for i, msg in enumerate(map_msgs):
        grid_maps[i].setTimestamp(int(1e3 * msg.info.utime))
        grid_maps[i].setFrameId(msg.info.parent_frame)
        grid_maps[i].setGeometry(
            length=np.array([msg.info.length_x, msg.info.length_y]),
            resolution=msg.info.resolution,
            position=np.array(msg.info.position)
        )

        grid_maps[i].setStartIndex(
            np.array([msg.outer_start_index, msg.inner_start_index])
        )

        for layer in msg.layers:
            data = np.array(layer.data).transpose()  # convert to column major
            grid_maps[i][layer.name][:] = data
    return grid_maps, robot_output


def save_matrix_plot(title: str, data: np.ndarray, folder: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    im = ax.imshow(data, cmap='viridis')
    return do_perception_fig_layout_and_save(ax, fig, title, folder)


def do_perception_fig_layout_and_save(ax, fig, title: str, folder: str, limits=None):
    ax.set_xticks([])
    ax.set_yticks([])
    if limits is not None:
        plt.xlim(limits['x'])
        plt.ylim(limits['y'])
    ax.set_aspect('equal')

    plt.title(title)

    filename = (title.replace(' ', '_') + '.png').lower()
    plt.savefig(os.path.join(folder, filename), dpi=300, bbox_inches='tight')
    new_limits = {'x': plt.xlim(), 'y': plt.ylim()}
    plt.close(fig)
    return new_limits


def setup_plots():
    matplotlib.rcParams.update(matplotlib.rcParamsDefault)
    font = {'size': 20, 'family': 'serif'}
    matplotlib.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"
    matplotlib.rc('text.latex', preamble=r'\usepackage{underscore}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 1
    matplotlib.rcParams['axes.titlesize'] = 30
    matplotlib.rcParams['xtick.major.size'] = 15
    matplotlib.rcParams['xtick.major.width'] = 1
    matplotlib.rcParams['xtick.minor.size'] = 7
    matplotlib.rcParams['xtick.minor.width'] = 1
    plt.rcParams['axes.prop_cycle'] = plt.cycler(color=plotting_palette)
