# standard library
from argparse import ArgumentParser

#installed
import lcm

from pydairlib.jacktoy import FrankaVisualizerDiagram
from pydairlib.common import write_meshcat_video_from_log

from pydrake.geometry import Meshcat

from dairlib import (
    lcmt_timestamped_saved_traj,
    lcmt_c3_forces,
    lcmt_c3_state,
    lcmt_sample_buffer,
    lcmt_robot_output,
    lcmt_object_state,
)

def write_video(logfile: str, savefile: str, duration=-1):
    visualizer = FrankaVisualizerDiagram()
    meshcat = visualizer.get_meshcat()

    # make channel to type map
    types = {}

    ports = dict(
        (channel, visualizer.get_input_port_for_channel(channel)) for \
         channel in visualizer.get_input_channels()]
    )

    lcm_log = lcm.EventLog(logfile, "r")
    write_meshcat_video_from_log(
        visualizer, lcm_log, meshcat, types, ports, savefile, duration=duration
    )


def main():
    parser = ArgumentParser()
    parser.add_argument('--logfile', type=str, default='')
    parser.add_argument('--savefile', type=str, default='')
    args = parser.parse_args()
    write_video(args.logfile, args.savefile)


if __name__ == '__main__':
    main()
