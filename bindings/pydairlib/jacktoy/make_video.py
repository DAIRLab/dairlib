# standard library
from argparse import ArgumentParser

#installed
import lcm

from pydairlib.jacktoy import FrankaVisualizerDiagram
from pydairlib.common import write_meshcat_video_from_log

from pydrake.geometry import Meshcat

# lcmtypes
import dairlib


def lookup_lcmtype(name: str):
    return getattr(dairlib, name)


def write_video(logfile: str, savefile: str, duration=-1):
    visualizer = FrankaVisualizerDiagram()
    meshcat = visualizer.get_meshcat()

    types = dict([
        (channel, lookup_lcmtype(visualizer.get_lcm_type(channel))) for \
        channel in visualizer.get_input_channels()
    ])

    ports = dict([
        (channel, visualizer.get_input_port_for_channel(channel)) for \
         channel in visualizer.get_input_channels()
    ])

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
