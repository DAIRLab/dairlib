import glob
import io
import os
import sys
import lcm
import numpy as np
from yaml import load, dump
from matplotlib import pyplot as plt

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


import pydairlib.analysis.video_tools as video_tools
from pydairlib.analysis.process_lcm_log import get_log_data
import pydairlib.analysis.mpc_plotting_utils as mpc_plots

from pydairlib.common.plot_styler import PlotStyler
from pydairlib.analysis.cassie_plotting_utils import get_timestamp_of_first_liftoff

# lcmtypes
import dairlib

default_hardware_mpc_debug_channels = {
    'channel_x': 'NETWORK_CASIE_STATE_DISPATCHER',
    'channel_mpc': 'ALIP_MINLP_DEBUG',
    'channel_terrain': 'FOOTHOLDS_PROCESSED'
}
outfolder = '/home/brian/workspace/manuscripts/tech_report/figures/'


class MpcLogTileDriver:
    """
        Class to assist in driving the meshcat visualization at evenly
        spaced time intervals for making motion tiles.
     """
    def __init__(self, fname, channels=None):
        if channels is None:
            self._channels = default_hardware_mpc_debug_channels
        else:
            self._channels = channels

        self._lcmlog = lcm.EventLog(fname, "r")
        self._lcm = lcm.LCM()
        self._tstart = self._lcmlog.read_next_event().timestamp
        self._tcurr = self._tstart
        self.reset()

    def reset(self):
        self._lcmlog.seek(0)
        self._tcurr = self._tstart

    def publish_and_advance(self, seconds):
        next_timestamp = self._tcurr + seconds * 1e6
        self._lcmlog.seek_to_timestamp(next_timestamp)
        channels_seen = []
        event = self._lcmlog.read_next_event()
        while event:
            self._lcm.publish(event.channel, event.data)
            if event.channel not in channels_seen:
                channels_seen.append(event.channel)

            seen = [channel in channels_seen for channel in self._channels.values()]
            if all(seen):
                break
            event = self._lcmlog.read_next_event()

        if not event:
                self.reset()
                self.publish_and_advance(seconds)
        else:
            self._tcurr = event.timestamp


mpc_channels = {
    "ALIP_MINLP_DEBUG": dairlib.lcmt_mpc_debug,
    "FOOTSTEP_TARGET": dairlib.lcmt_footstep_target
}


def init_ps():
    PlotStyler.set_default_styling()


def collect_solve_time_vs_num_footholds(mpc_debug, data=None):
    data = {} if data is None else data
    for i, t in enumerate(mpc_debug.t_mpc):
        if mpc_debug.nfootholds[t] not in data:
            data[mpc_debug.nfootholds[t]] = [mpc_debug.solve_time[i]]
        else:
            data[mpc_debug.nfootholds[t]].append(mpc_debug.solve_time[i])
    return data


def collect_solve_time_vs_constraint_activation(mpc_debug, data=None):
    data = { True: [], False: [] } if data is None else data
    for i, t in enumerate(mpc_debug.t_mpc):
        data[mpc_debug.constraint_activation[t]].append(mpc_debug.solve_time[i])
    return data


def plot_solve_time_vs_nfootholds(logs):
    data = {}
    for log in logs.values():
        data = collect_solve_time_vs_num_footholds(log["mpc_debug"], data)
    for key in data:
        data[key] = np.array(data[key])

    ns = list(data.keys())
    t_min = [1000 * np.min(data[k]) for k in data]
    t_avg = [1000 * np.mean(data[k]) for k in data]
    t_max = [1000 * np.max(data[k]) for k in data]
    t_95 = [1000 * np.percentile(data[k], 90) for k in data]

    t_min = [t for _, t in sorted(zip(ns, t_min))]
    t_max = [t for _, t in sorted(zip(ns, t_max))]
    t_avg = [t for _, t in sorted(zip(ns, t_avg))]
    t_95 = [t for _, t in sorted(zip(ns, t_95))]
    ns = sorted(ns)

    ps = PlotStyler(directory=outfolder)
    ps.plot(ns, t_min)
    ps.plot(ns, t_max)
    ps.plot(ns, t_avg)
    ps.plot(
        ns, t_95, xlabel='Number of Footholds',
        ylabel='MPFC Solve Time (ms)',
        title='MPFC Solve Time vs. \# of Footholds',
        ylim=[0, 60]
    )
    ps.add_legend(['Min.', 'Max.', 'Mean', '90th Percentile'], ncol=2)
    ps.save_fig('solve_time_vs_footholds.png')


def plot_solve_time_vs_constraint_activation(logs):
    data = { False: [], True: [] }

    for log in logs.values():
        data = collect_solve_time_vs_constraint_activation(log["mpc_debug"], data)
    for key in data:
        data[key] = 1000 * np.array(data[key])
    n_slack = len(data[False])
    n_active = len(data[True])

    ps = PlotStyler(directory=outfolder)
    plt.boxplot(data.values())
    plt.gca().set_xticklabels(
        [f'Foothold\nConstraint Slack\n(n = {n_slack})', f'Foothold\nConstraint Active\n(n = {n_active})']
    )
    # plt.ylabel('MPFC Solve Time (ms)')
    plt.title('Solve Time vs. Foothold Constraint Activation')
    ps.save_fig('solve_time_vs_activation')


def solve_time_main():
    init_ps()

    dataset_config = load(
        io.open('bindings/pydairlib/analysis/plot_configs/tr_plots.yaml', 'r'),
        Loader=Loader
    )
    log_parent_folder = sys.argv[1]
    logs = {}

    keys = ['solve_time_vs_constraint_activation', 'solve_time_vs_num_footholds']
    for key in keys:
        logs_key = {}
        for log in dataset_config[key]:
            log_key = f'{log["date"]}-lcmlog-mpc-{log["lognum"]}'
            fname = os.path.join(
                log_parent_folder,
                f'{log["date"]}/lcmlog-mpc-{log["lognum"]}'
            )
            lcmlog = lcm.EventLog(fname, "r")
            mpc_debug, _ = get_log_data(
                lcmlog,
                mpc_channels,
                log["start_time"],
                log["end_time"],
                mpc_plots.mpc_processing_callback,
                "ALIP_MINLP_DEBUG", "FOOTSTEP_TARGET", "FSM"
            )
            logs_key[log_key] = {"mpc_debug" : mpc_debug}
        logs[key] = logs_key

    # plot_solve_time_vs_nfootholds(logs["solve_time_vs_num_footholds"])
    plot_solve_time_vs_constraint_activation(logs["solve_time_vs_constraint_activation"])


def motion_tiles(date, lognum, start_offset, duration, num_tiles, prefix='', frame_edits=None):
    # first do the video
    dataset_config = load(
        io.open('bindings/pydairlib/analysis/plot_configs/tr_plots.yaml', 'r'),
        Loader=Loader
    )
    timing_info = dataset_config["timing_offsets"][date][lognum]
    video_name = timing_info['video_file']
    video_file = f"/home/brian/workspace/data/alip_mpc/alip_mpc_videos/success/{date}/{video_name}.MP4"
    outfolder = "/home/brian/workspace/data/alip_mpc_paper/motion_tiles"
    start_time = timing_info["video_anchor"] + start_offset
    end_time = start_time + duration
    video_tools.extract_frames(
        start_time,
        end_time,
        num_tiles,
        video_file,
        outfolder,
        prefix,
        frame_edits
    )

    # Now do the meshcat
    log_file = f"/home/brian/workspace/data/alip_mpc/alip_mpc_hardware_logs/{date}/lcmlog-mpc-{lognum}"


def motion_tiles_main():
    n = 10
    motion_tiles('05_15_23', '13', 9.1, 0.4 * n, n, 'up', {'aspect': 0.9, 'yshift': -0.1, 'mirror': True})
    motion_tiles('05_15_23', '13', 25.1, 0.4 * n, n, 'down', {'aspect': 0.9, 'yshift': -0.1, 'mirror': False})


def get_mpc_log_starts(logfolder):
    logs = glob.glob(os.path.join(logfolder, 'lcmlog-mpc-[0-9][0-9]'))
    for log in logs:
        lcmlog = lcm.EventLog(log)
        start = get_timestamp_of_first_liftoff(
            lcmlog,
            "NETWORK_CASSIE_STATE_DISPATCHER",
            vel_thresh=0.1
        )
        print(f'{log.split("/")[-1]}: {start:.2f}')


if __name__ == "__main__":
    # get_mpc_log_starts( "/home/brian/workspace/data/alip_mpc/alip_mpc_hardware_logs/05_15_23")
    # motion_tiles_main()
    solve_time_main()

