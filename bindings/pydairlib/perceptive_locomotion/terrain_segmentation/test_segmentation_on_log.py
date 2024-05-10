import time
import timeit
from time import sleep

import lcm
import numpy as np
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

from pydairlib.analysis.process_lcm_log import get_log_data

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem

from pydairlib.geometry.convex_polygon import ConvexPolygonSender
from pydairlib.systems.perception import GridMapSender, PlaneSegmentationSystem


from argparse import ArgumentParser


state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'


def process_grid_maps(data_dict):
    map_msgs = data_dict['CASSIE_ELEVATION_MAP']
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


def build_diagram(mode: str, lcm: DrakeLcm) -> Diagram:

    builder = DiagramBuilder()

    terrain_segmentation = PlaneSegmentationSystem(
        'systems/perception/ethz_plane_segmentation/params.yaml'
    ) \
        if mode == 'planar' else TerrainSegmentationSystem()

    convex_decomposition = ConvexTerrainDecompositionSystem()
    foothold_sender = ConvexPolygonSender()

    foothold_publisher = LcmPublisherSystem.Make(
        channel="FOOTHOLDS_PROCESSED",
        lcm_type=lcmt_foothold_set,
        lcm=lcm,
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )
    grid_map_sender = GridMapSender()
    grid_map_publisher = LcmPublisherSystem.Make(
        channel="CASSIE_ELEVATION_MAP",
        lcm_type=lcmt_grid_map,
        lcm=lcm,
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


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfile', type=str)
    args = parser.parse_args()

    lcm_interface = DrakeLcm()
    diagram = build_diagram('curvature', lcm_interface)

    log = lcm.EventLog(args.logfile, "r")
    grid_maps, robot_output = get_log_data(
        lcm_log=log,
        lcm_channels={
            'CASSIE_ELEVATION_MAP': lcmt_grid_map,
            state_channel: lcmt_robot_output
        },
        start_time=3,
        duration=-1,
        data_processing_callback=process_grid_maps
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
            lcm_interface.Publish(
                state_channel,
                robot_output[s].encode()
            )
            s += 1
        sleep(0.1)


if __name__ == '__main__':
    main()
