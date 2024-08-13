import signal
import sys

from dairlib import lcmt_robot_output, lcmt_foothold_set, lcmt_grid_map, \
    lcmt_contact, lcmt_alip_mpc_output

from pydrake.systems.all import (
    Diagram,
    Context,
    DiagramBuilder,
    LcmPublisherSystem,
    LcmSubscriberSystem,
    TriggerType,
)

from pydrake.lcm import DrakeLcm

from pydrake.common.value import AbstractValue

import pydairlib.lcm  # needed for cpp serialization of lcm messages

from pydairlib.perceptive_locomotion.ros_diagrams import (
    CassieElevationMappingRosDiagram,
)

from pydairlib.perceptive_locomotion.diagrams import (
    RadioReceiverModule,
    MpfcOutputFromRL
)

from pydairlib.perceptive_locomotion.systems.elevation_map_converter \
    import ElevationMappingConverter, ElevationMapOptions, ElevationMapQueryObject

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import LcmOutputDrivenLoop, OutputVector
from pydairlib.systems.perception import GridMapSender
from pydairlib.systems.robot_lcm_systems import RobotOutputReceiver

import numpy as np

model_path = "test003"
model_hz = 25.0
model_hidden_size = 64
points_topic = "/camera/depth/color/points"
cassie_state_channel = "CASSIE_STATE_SIMULATION"
cassie_out_channel = "NETWORK_CASSIE_OUT"
urdf = "examples/Cassie/urdf/cassie_v2.urdf"

elevation_mapping_params = (
    "bindings/pydairlib/perceptive_locomotion/params"
    "/elevation_mapping_params.yaml"
)

from RLsystem import RLSystem


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

    map_converter = ElevationMappingConverter(
        urdf, ElevationMapOptions()
    )
    actor = RLSystem(model_path=model_path, hidden_size=model_hidden_size)

    # elevation_map_sender = GridMapSender()
    # elevation_map_publisher_local = LcmPublisherSystem.Make(
    #     channel="CASSIE_ELEVATION_MAP",
    #     lcm_type=lcmt_grid_map,
    #     lcm=elevation_mapping.lcm(),
    #     publish_triggers={TriggerType.kPeriodic},
    #     publish_period=1.0 / 30.0,
    #     use_cpp_serializer=True
    # )
    radio = RadioReceiverModule(cassie_out_channel, elevation_mapping.lcm())

    action_sender = MpfcOutputFromRL()
    network_lcm = DrakeLcm("udpm://239.255.76.67:7667?ttl=1")
    action_pub = LcmPublisherSystem.Make(
        channel="ALIP_MPC",
        lcm_type=lcmt_alip_mpc_output,
        lcm=network_lcm,
        publish_triggers={TriggerType.kPeriodic},
        publish_period=1.0 / model_hz,
        use_cpp_serializer=True
    )

    builder.AddSystem(actor)
    builder.AddSystem(radio)
    builder.AddSystem(action_pub)
    builder.AddSystem(action_sender)
    builder.AddSystem(map_converter)
    builder.AddSystem(elevation_mapping)
    # builder.AddSystem(elevation_map_sender)
    # builder.AddSystem(elevation_map_publisher_local)
    #
    # builder.Connect(
    #     elevation_map_sender.get_output_port(),
    #     elevation_map_publisher_local.get_input_port()
    # )
    builder.Connect(
        elevation_mapping.get_output_port_state(),
        actor.get_input_port_by_name('state')
    )
    # builder.Connect(
    #     elevation_mapping.get_output_port_grid_map(),
    #     elevation_map_sender.get_input_port()
    # )
    builder.Connect(
        elevation_mapping.get_output_port_grid_map(),
        map_converter.get_input_port_by_name('elevation')
    )
    builder.Connect(
        elevation_mapping.get_output_port_state(),
        map_converter.get_input_port_by_name('x')
    )
    builder.Connect(
        actor.get_output_port_by_name('fsm'),
        map_converter.get_input_port_by_name('fsm')
    )
    builder.Connect(
        map_converter.get_output_port(),
        actor.get_input_port_by_name('height_map')
    )
    builder.Connect(
        radio.get_output_port(),
        actor.get_input_port_by_name('desired_velocity')
    )
    builder.Connect(
        actor.get_output_port_by_name('actions'),
        action_sender.get_input_port(),
    )
    builder.Connect(
        action_sender.get_output_port(),
        action_pub.get_input_port()
    )

    diagram = builder.Build()
    DrawAndSaveDiagramGraph(
        diagram,
        '../rl_for_hw'
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
