import os
import lcm
import time
import tempfile
import subprocess
import matplotlib
import numpy as np
from PIL import Image
from typing import List
from copy import deepcopy
import matplotlib.animation
from grid_map import GridMap
from matplotlib import pyplot as plt, patches

from pydairlib.analysis.mbp_plotting_utils import process_state_channel
from pydairlib.analysis.cassie_plotting_utils import make_plant_and_context

# lcmtypes
from dairlib import(
    lcmt_grid_map,
    lcmt_profiling,
    lcmt_foothold_set,
    lcmt_robot_output,
    lcmt_alip_s2s_mpfc_debug
)

from pydairlib.systems import (
    PlaneSegmentationSystem,
    GridMapSender
)

from pydairlib.perceptive_locomotion.terrain_segmentation import (
    ConvexTerrainDecompositionSystem,
    TerrainSegmentationSystem,
    plot_polygons_with_holes,
    plot_polygon,
    segmentation_criteria as seg_criteria
)

from pydairlib.analysis.process_lcm_log import get_log_data

from pydrake.all import DrakeLcm

from pydrake.systems.all import (
    Diagram,
    Context,
    TriggerType,
    DiagramBuilder,
    LcmPublisherSystem,
)

from pydairlib.geometry.convex_polygon import ConvexPolygonSender


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'
elevation_map_channel = 'CASSIE_ELEVATION_MAP'
mpfc_debug_channel = 'ALIP_S2S_MPFC_DEBUG'
terrain_channel = 'FOOTHOLDS_PROCESSED'

plotting_palette = ["#011f5b", "#9f642d", "#af0000", "#b99aa0", "#666666", "#5583ab"]


def make_dir_if_missing(directory_path: str):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Output directory created: {directory_path}")
        return False
    else:
        print(f"Output directory already exists - skipping creation: {directory_path}")
        return True


def binary_search_closest(arr: List[float], target: float) -> int:
    left, right = 0, len(arr) - 1

    # Handle edge cases
    if target <= arr[0]:
        return 0
    if target >= arr[-1]:
        return len(arr) - 1

    while left <= right:
        mid = (left + right) // 2

        # Perfect match
        if arr[mid] == target:
            return mid

        if arr[mid] < target:
            if mid + 1 < len(arr) and arr[mid + 1] > target:
                # Choose the closer timestamp between mid and mid+1
                if abs(target - arr[mid]) < abs(arr[mid + 1] - target):
                    return mid
                return mid + 1
            left = mid + 1
        else:
            if mid - 1 >= 0 and arr[mid - 1] < target:
                # Choose the closer timestamp between mid-1 and mid
                if abs(target - arr[mid - 1]) < abs(arr[mid] - target):
                    return mid - 1
                return mid
            right = mid - 1


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


def get_elevation_map_profiling(
        logfile: str, start_time: float = 0, duration: float = -1):
    channel = 'ELEVATION_MAP_PROFILING'

    def process_profiling(data):
        _times = [d.process_us * 1e-6 for d in data[channel]]
        return np.array(_times)

    log = lcm.EventLog(logfile, "r")
    times = get_log_data(
        log,
        {
            channel: lcmt_profiling
        },
        start_time, duration,
        process_profiling
    )
    return times


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


def calc_polygon_iou(results):
    """
        Calculate the IoU between the union of the convex polygon terrain
        representation of consecutive safe terrain segmentations
        :param results: The IoUs are appended to results under the key 'poly_iou'
        :return: None
    """
    poly_iou = []
    decomposer = ConvexTerrainDecompositionSystem()
    
    for i in range(len(results['grid_maps']) - 1):
        poly_iou.append(
            convex_polyon_iou(
                results['grid_maps'][i],
                results['grid_maps'][i+1],
                decomposer
            )
        )
        
        # iou of 0 caused by ACD non-robustness - discard
        if poly_iou[-1] == 0:
            poly_iou[-1] = np.nan
            
        if i % 5 == 0:
            print(f'{100.0 * float(i) / float(len(results["grid_maps"])):.2f}')
    
    results[f'poly_iou'] = poly_iou


def hysteresis_comparison(logfile):
    grid_maps, _ = get_grid_maps_from_log(logfile)

    hysts = [0.0, 0.1, 0.3, 0.6]
    results = []
    for h in hysts:
        s3 = TerrainSegmentationSystem(
            {
                'curvature_criterion': seg_criteria.curvature_criterion,
                'inclination_criterion': seg_criteria.inclination_criterion,
            }
        )
        s3.safety_hysteresis = h
        s3.set_name(f'$k_{{hyst}}$ = {h:.1f}')
        results.append(profile_segmentation(s3, grid_maps))

    return results


def make_segmentation_systems():
    params_folder = \
        'bindings/pydairlib/perceptive_locomotion/terrain_segmentation/plane_segmentation_results_params/'
    s3 = TerrainSegmentationSystem(
        {
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion,
        }
    )
    s3.set_name('S3 (Ours)')

    yamls = [
        os.path.join(params_folder, f'{y}.yaml') for y in
        ['ransac_and_preprocessing', 'no_ransac_with_preprocessing']
    ]
    configs = ['', '_NR']
    system_names = ['EM_cupy' + config for config in configs]

    systems = []
    for params, name in zip (yamls, system_names):
        systems.append(PlaneSegmentationSystem(params))
        systems[-1].set_name(name)
    systems.append(s3)
    return systems


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


def build_perception_diagram(lcm_interface: DrakeLcm, profiling=None) -> Diagram:

    builder = DiagramBuilder()

    terrain_segmentation = TerrainSegmentationSystem({
            'curvature_criterion': seg_criteria.curvature_criterion,
            'inclination_criterion': seg_criteria.inclination_criterion
        }, profiling
    )

    convex_decomposition = ConvexTerrainDecompositionSystem(profiling)
    foothold_sender = ConvexPolygonSender()

    foothold_publisher = LcmPublisherSystem.Make(
        channel="FOOTHOLDS_PROCESSED",
        lcm_type=lcmt_foothold_set,
        lcm=lcm_interface,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )
    grid_map_sender = GridMapSender()
    grid_map_publisher = LcmPublisherSystem.Make(
        channel="CASSIE_ELEVATION_MAP",
        lcm_type=lcmt_grid_map,
        lcm=lcm_interface,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )

    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)
    builder.AddSystem(foothold_publisher)
    builder.AddSystem(foothold_sender)
    builder.AddSystem(grid_map_sender)
    builder.AddSystem(grid_map_publisher)

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
    builder.Connect(
        terrain_segmentation.get_output_port(),
        grid_map_sender.get_input_port()
    )
    builder.Connect(
        grid_map_sender.get_output_port(),
        grid_map_publisher.get_input_port()
    )

    builder.ExportInput(
        terrain_segmentation.get_input_port(),
        "grid_map"
    )
    diagram = builder.Build()
    return diagram


def profile_full_perception_pipeline(logfile):
    profiling_results = {
        'segmentation': [],
        'seg_callbacks': [[], [], []],
        'decomposition': [],
        'total': [],
        'plane_fitting': [],
        'num_polygons': []
    }

    lcm_interface = DrakeLcm()
    diagram = build_perception_diagram(lcm_interface, profiling=profiling_results)
    context = diagram.CreateDefaultContext()

    grid_maps, robot_output = get_grid_maps_from_log(logfile, 0, -1)

    for map in grid_maps:
        diagram.get_input_port().FixValue(context, map)
        start = time.time()
        diagram.CalcForcedUnrestrictedUpdate(
            context,
            context.get_mutable_state()
        )
        diagram.ForcedPublish(context)
        end = time.time()
        profiling_results['total'].append(end - start)

    return profiling_results


def get_worst_case_data_by_num_polygons(results):
    data = {
        'segmentation': {},
        'decomposition': {},
        'plane_fitting': {}
    }

    for r in results:
        for key in data:
            for n_poly, val in zip(r['num_polygons'], r[key]):
                if n_poly not in data[key]:
                    data[key][n_poly] = 0
                data[key][n_poly] = np.maximum(val, data[key][n_poly])

    for key in data:
        nums = sorted(set(data[key].keys()))
        data[key] = np.array([data[key][n] for n in nums])

    return data


def safe_terrain_iou(frame0: GridMap, frame1: GridMap, layer='segmentation'):
    # move frame0 to remove any pixels which the maps do not have in common
    frame0.move(frame1.getPosition())
    frame0.convertToDefaultStartIndex()
    frame1.convertToDefaultStartIndex()

    frame0_safe = np.nan_to_num(frame0[layer]).astype(bool)
    frame1_safe = np.nan_to_num(frame1[layer]).astype(bool)

    intersection = np.logical_and(frame0_safe,  frame1_safe)
    union = np.logical_or(frame0_safe, frame1_safe)

    if union.sum() == 0:
        return 0

    return float(intersection.sum()) / float(union.sum())


def convex_polyon_iou(
        frame0: GridMap,
        frame1: GridMap,
        decomp: ConvexTerrainDecompositionSystem, layer='segmentation'):
    # align the grid maps
    frame0.move(frame1.getPosition())
    frame0.convertToDefaultStartIndex()
    frame1.convertToDefaultStartIndex()
    
    polys_0 = decomp.calc_convex_polygons(frame0)
    polys_1 = decomp.calc_convex_polygons(frame1)
    
    contained_in_p0_sampled = np.zeros_like(frame0[layer])
    contained_in_p1_sampled = np.zeros_like(frame1[layer])
    
    dims = frame0.getSize()
    for i in range(dims[0]):
        for j in range(dims[1]):
            index = np.array([i, j], dtype=int)
            pos = np.zeros((3,))
            pos[:2] = frame0.getPosition(index)
            
            violates_p0 = [p.PointViolatesInequalities(pos) for p in polys_0]
            violates_p1 = [p.PointViolatesInequalities(pos) for p in polys_1]
            
            contained_in_p0_sampled[index[0], index[1]] = ~np.all(violates_p0)
            contained_in_p1_sampled[index[0], index[1]] = ~np.all(violates_p1)
    
    intersection = np.logical_and(contained_in_p0_sampled,  contained_in_p1_sampled)
    union = np.logical_or(contained_in_p0_sampled, contained_in_p1_sampled)
    
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
        
        if msg.info.resolution > 0:
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


def save_matrix_plot(title: str, data: np.ndarray, folder: str):
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
