import numpy as np
from grid_map import GridMap
import matplotlib


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
    return grid_maps, robot_output_msgs


def setup_plots():
    matplotlib.rcParams.update(matplotlib.rcParamsDefault)
    font = {'size': 15, 'family': 'serif'}
    matplotlib.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"
    matplotlib.rc('text.latex', preamble=r'\usepackage{underscore}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 1
    matplotlib.rcParams['axes.titlesize'] = 20
    matplotlib.rcParams['xtick.major.size'] = 15
    matplotlib.rcParams['xtick.major.width'] = 1
    matplotlib.rcParams['xtick.minor.size'] = 7
    matplotlib.rcParams['xtick.minor.width'] = 1