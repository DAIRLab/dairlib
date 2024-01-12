from dairlib import lcmt_robot_output

from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
    ZeroOrderHold,
    LcmSubscriberSystem
)

from pydairlib.perceptive_locomotion.ros_diagrams import (
    ElevationMappingRosDiagram
)

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation. \
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph
from pydairlib.systems.framework import LcmOutputDrivenLoop

import numpy as np


points_topic = "/camera/depth/color/points"
elevation_mapping_params = (
    "bindings/pydairlib/perceptive_locomotion/params/elevation_mapping_params"
    ".yaml"
)


def main():
    builder = DiagramBuilder()

    elevation_mapping = ElevationMappingRosDiagram(
        elevation_mapping_params,
        points_topic
    )
    terrain_segmentation = TerrainSegmentationSystem()
    convex_decomposition = ConvexTerrainDecompositionSystem()
    builder.AddSystem(elevation_mapping)
    builder.AddSystem(terrain_segmentation)
    builder.AddSystem(convex_decomposition)

    builder.Connect(
        elevation_mapping.get_output_port(),
        terrain_segmentation.get_input_port()
    )
    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
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
        input_channel="CASSIE_NETWORK_STATE_DISPATCHER",
        is_forced_publish=True
    )

    driven_loop.Simulate(np.inf)


if __name__ == '__main__':
    main()
