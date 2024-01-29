import signal
from dairlib import lcmt_robot_output, lcmt_foothold_set, lcmt_grid_map, \
    lcmt_contact

from pydrake.systems.all import (
    Diagram,
    Context,
    DiagramBuilder,
    LcmPublisherSystem,
    LcmSubscriberSystem,
    TriggerType,
)

from pydrake.common.value import AbstractValue

import pydairlib.lcm  # needed for cpp serialization of lcm messages

from pydairlib.geometry.convex_polygon import ConvexPolygonSender

from pydairlib.perceptive_locomotion.ros_diagrams import (
    CassieElevationMappingRosDiagram
)

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import LcmOutputDrivenLoop, OutputVector
from pydairlib.systems.perception import GridMapSender
from pydairlib.systems.robot_lcm_systems import RobotOutputReceiver

import numpy as np


points_topic = "/camera/depth/color/points"
cassie_state_channel = "NETWORK_CASSIE_STATE_DISPATCHER"
elevation_mapping_params = (
    "bindings/pydairlib/perceptive_locomotion/params/elevation_mapping_params"
    ".yaml"
)


def stop(sig, _):
    print(f'caught signal {sig}, shutting down')
    quit(0)


def main():
    signal.signal(signal.SIGINT, stop)
    builder = DiagramBuilder()

    elevation_mapping = CassieElevationMappingRosDiagram(
        elevation_mapping_params,
        points_topic
    )
    plant = elevation_mapping.plant()
    terrain_segmentation = TerrainSegmentationSystem()
    convex_decomposition = ConvexTerrainDecompositionSystem()
    foothold_sender = ConvexPolygonSender()

    contact_subscriber = LcmSubscriberSystem.Make(
        channel="NETWORK_CASSIE_CONTACT_DISPATCHER",
        lcm_type=lcmt_contact,
        lcm=elevation_mapping.lcm(),
        use_cpp_serializer=True
    )

    foothold_publisher = LcmPublisherSystem.Make(
        channel="FOOTHOLDS_PROCESSED",
        lcm_type=lcmt_foothold_set,
        lcm=elevation_mapping.lcm(),
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )
    elevation_map_sender = GridMapSender()
    elevation_map_publisher = LcmPublisherSystem.Make(
        channel="CASSIE_ELEVATION_MAP",
        lcm_type=lcmt_grid_map,
        lcm=elevation_mapping.lcm(),
        publish_triggers={TriggerType.kForced},
        publish_period=0.0,
        use_cpp_serializer=True
    )

    builder.AddSystem(elevation_mapping)
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)
    builder.AddSystem(contact_subscriber)
    builder.AddSystem(foothold_publisher)
    builder.AddSystem(foothold_sender)
    builder.AddSystem(elevation_map_sender)
    builder.AddSystem(elevation_map_publisher)

    builder.Connect(
        contact_subscriber.get_output_port(),
        elevation_mapping.get_input_port_contact()
    )
    builder.Connect(
        elevation_mapping.get_output_port(),
        terrain_segmentation.get_input_port()
    )
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
        elevation_map_sender.get_input_port()
    )
    builder.Connect(
        elevation_map_sender.get_output_port(),
        elevation_map_publisher.get_input_port()
    )
    diagram = builder.Build()
    DrawAndSaveDiagramGraph(
        diagram,
        '../elevation_mapping_and_convex_decomposition'
    )

    driven_loop = LcmOutputDrivenLoop(
        drake_lcm=elevation_mapping.lcm(),
        diagram=diagram,
        lcm_parser=elevation_mapping,
        input_channel=cassie_state_channel,
        is_forced_publish=True
    )

    robot_state = driven_loop.WaitForFirstState(plant)
    elevation_mapping.InitializeElevationMap(
        robot_state,
        driven_loop.get_diagram_mutable_context()
    )

    driven_loop.Simulate()


if __name__ == '__main__':
    main()
