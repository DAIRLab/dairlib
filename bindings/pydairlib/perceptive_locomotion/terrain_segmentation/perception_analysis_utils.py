import os
import numpy as np
from grid_map import GridMap
import matplotlib
from matplotlib import pyplot as plt

from pydairlib.analysis.mbp_plotting_utils import process_state_channel
from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context


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