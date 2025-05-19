# standard library imports
import os
from argparse import ArgumentParser

# installed
import lcm
import numpy as np

# lcmtypes
from dairlib import(
    lcmt_grid_map,
    lcmt_robot_output,
)

# pydrake
from pydrake.systems.all import (
    System,
    Diagram,
    Context,
    DiagramBuilder,
)
from pydrake.geometry import Meshcat


# Grid Map
from grid_map import GridMap
from pydairlib.common import MeshcatChromeCapture, write_meshcat_video_from_log

from pydairlib.systems import (
    GridMapVisualizer,
    GridMapReceiver,
    PlantVisualizer,
    RobotOutputReceiver
)

state_channel = 'NETWORK_CASSIE_STATE_DISPATCHER'

def write_grid_map_meshcat_video(logfile: str, savefile: str, layer="elevation", duration=80):
    urdf = "examples/Cassie/urdf/cassie_v2_shells_dark_gray.urdf"
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
        state_channel : lcmt_robot_output,
        'CASSIE_ELEVATION_MAP': lcmt_grid_map,
    }

    ports = {
        state_channel : receivers['state'].get_input_port(),
        'CASSIE_ELEVATION_MAP': receivers['grid_map'].get_input_port(),
    }

    try:
        lcm_log = lcm.EventLog(logfile)
    except FileNotFoundError as e:
        print(f'Cannot find {logfile}')
        exit(-1)

    write_meshcat_video_from_log(
        diagram, lcm_log, meshcat, types, ports, savefile, duration=duration)


def main():
    parser = ArgumentParser(description="Process an input file and write to an output file.")
    parser.add_argument('--input_file', type=str, required=True, help='Path to the input file')
    parser.add_argument('--output_file', type=str, required=True, help='Path to the output file')

    args = parser.parse_args()
    write_grid_map_meshcat_video(args.input_file, args.output_file)


if __name__ == '__main__':
    main()

