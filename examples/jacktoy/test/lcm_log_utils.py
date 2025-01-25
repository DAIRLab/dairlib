"""Get messages from desired LCM channels from an LCM log between provided
start and end times.  Uses the virtual environment installed at
dairlib/drake_env.  Requires dairlib/bazel-bin/lcmtypes to be in the
PYTHONPATH, i.e.:

source ~/workspace/dairlib/drake_env/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/sharanya/workspace/dairlib/bazel-bin/lcmtypes

Example usage:

python examples/jacktoy/test/lcm_log_utils.py single /mnt/data2/sharanya/logs/2024/12_16_24/000006/ --interactive --start=50 --end=100
"""

import click
import os
import os.path as op
import shutil
from typing import List, Tuple

from lcm import EventLog
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.ticker as mtick
import numpy as np
from scipy.stats import gaussian_kde
import yaml

import dairlib


# Add to this dictionary to include more LCM channels from which to read.
ALL_CHANNELS_AND_LCMT = {
    'C3_ACTUAL': dairlib.lcmt_c3_state,
    'C3_FINAL_TARGET': dairlib.lcmt_c3_state,
    'SAMPLE_BUFFER': dairlib.lcmt_sample_buffer,
    'SAMPLE_LOCATIONS': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLE_COSTS': dairlib.lcmt_timestamped_saved_traj,
    'C3_TRAJECTORY_ACTOR_CURR_PLAN': dairlib.lcmt_timestamped_saved_traj,
    'C3_TRAJECTORY_ACTOR_BEST_PLAN': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLING_C3_DEBUG': dairlib.lcmt_sampling_c3_debug,
    'SAMPLING_C3_RADIO': dairlib.lcmt_radio_out,
    'FRANKA_STATE': dairlib.lcmt_robot_output,
}
MINIMAL_CHANNELS_AND_LCMT = {
    'SAMPLE_BUFFER': dairlib.lcmt_sample_buffer,
    'SAMPLING_C3_DEBUG': dairlib.lcmt_sampling_c3_debug,
}
CHANNELS_AND_LCMT_TO_SYNC = {key: val for key, val in \
                             ALL_CHANNELS_AND_LCMT.items() if \
                             key not in ['SAMPLING_C3_RADIO', 'FRANKA_STATE']}
TIME_KEY = 'seconds'
MESSAGE_KEY = 'message'

TRAJ_PARAM_POS_TOL_KEY = 'position_success_threshold'
TRAJ_PARAM_RAD_TOL_KEY = 'orientation_success_threshold'

# Labels for the source of repositioning targets.
NO_TARGET_LABEL = 'N/A'
PREV_REPOS_TARGET_LABEL = 'previous repositioning target'
NEW_SAMPLE_TARGET_LABEL = 'new sample target'
BUFFER_SAMPLE_TARGET_LABEL = 'buffer sample target'
PURSUED_TARGET_LABELS = [NO_TARGET_LABEL,
                         PREV_REPOS_TARGET_LABEL,
                         NEW_SAMPLE_TARGET_LABEL,
                         BUFFER_SAMPLE_TARGET_LABEL]

# Labels for the reason behind switching modes.
NO_SWITCH_LABEL = 'N/A'
TO_C3_LOWER_COST_SWITCH_LABEL = 'Switch to Contact-Rich:  lower cost'
TO_C3_REACHED_TARGET_SWITCH_LABEL = 'Switch to Contact-Rich:  reached pursued sample'
TO_REPOS_LOWER_COST_SWITCH_LABEL = 'Switch to Contact-Free:  lower cost'
TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL = 'Switch to Contact-Free:  unproductivity'
TO_C3_XBOX_FORCED_SWITCH_LABEL = 'Switch to Contact-Rich:  Xbox'
MODE_SWITCH_LABELS = [NO_SWITCH_LABEL,
                      TO_C3_LOWER_COST_SWITCH_LABEL,
                      TO_C3_REACHED_TARGET_SWITCH_LABEL,
                      TO_REPOS_LOWER_COST_SWITCH_LABEL,
                      TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL,
                      TO_C3_XBOX_FORCED_SWITCH_LABEL]
MODE_SWITCH_COLORS = ['black', '#f78b8e', '#ffcc80', '#a5c493', '#c495f0',
                      'purple']
CF_COLOR = '#bbbbbb'
CR_COLOR = 'white'
POS_ERROR_COLOR = 'black'
RAD_ERROR_COLOR = 'purple'

# Other success thresholds.
# POS_SUCCESS_THRESHOLDS = np.array([0.01, 0.02, 0.03, 0.04, 0.05])
# RAD_SUCCESS_THRESHOLDS = np.array([0.05, 0.1, 0.2, 0.3, 0.4])
# THRESHOLD_COLORS = ['black', 'red', 'darkorange', 'gold', 'green']
POS_SUCCESS_THRESHOLDS = np.array([0.02, 0.05])
RAD_SUCCESS_THRESHOLDS = np.array([0.1, 0.4])
THRESHOLD_COLORS = ['red', 'blue']

EPS = 1e-5
TIME_SYNCH_THRESH = 0.03
HIST_BINS = 10
CDF_TIME_CUTOFF = 300

global_is_interactive = False


def save_current_figure(filename: str, store_folder: str = None):
    if store_folder is not None:
        plt.savefig(op.join(store_folder, f'{filename}.png'))
        print(f'Wrote plot to {op.join(store_folder, f"{filename}.png")}')
    else:
        plt.savefig(f'examples/jacktoy/test/tmp/{filename}.png')
        print(f'Wrote plot to examples/jacktoy/test/tmp/{filename}.png')

    global global_is_interactive
    if not global_is_interactive:
        plt.close()

def get_shading_masks(bool_array):
    bool_array = bool_array.squeeze()
    assert bool_array.ndim == 1
    bool_array = bool_array.astype(bool)
    right_shifted_yes = np.append(bool_array[0], bool_array[:-1])

    yes_shading_mask = np.ravel(np.column_stack(
        (right_shifted_yes, bool_array)))
    no_shading_mask = np.ravel(np.column_stack(
        (~right_shifted_yes, ~bool_array)))

    return yes_shading_mask, no_shading_mask

# TODO implement and remove
def visualize_sample_buffer(messages_by_channel: dict, log_folder: str = None):
    # First start with just a simple matplotlib plot of the buffer contents.
    sample_buffers = messages_by_channel['SAMPLE_BUFFER'][MESSAGE_KEY]
    states = messages_by_channel['C3_ACTUAL'][MESSAGE_KEY]
    debugs = messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]

    times = messages_by_channel['SAMPLE_BUFFER'][TIME_KEY]

    n_in_buffers = []
    quats, xyzs, ee_xyzs = [], [], []
    # Store orientation error in full rotation units for easier viewing on same
    # axis as meter distance error.
    pos_errors, full_rotation_errors = [], []
    for buffer, state, debug in zip(sample_buffers, states, debugs):
        n_in_buffers.append(buffer.num_in_buffer)
        quats.append(state.state[3:7])
        xyzs.append(state.state[7:10])
        ee_xyzs.append(state.state[:3])

        pos_errors.append(debug.current_pos_error)
        full_rotation_errors.append(debug.current_rot_error / (2*np.pi))

    _fig, axs = plt.subplots(2, 1, figsize=(6, 9), sharex=True)
    axs[0].plot(times, n_in_buffers)
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Number of samples')
    axs[0].set_title('Number of active samples in buffer')
    axs[1].plot(times, pos_errors, label='Position error')
    axs[1].plot(times, full_rotation_errors, label='Rotation error')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Error [m or full rotation]')
    axs[1].set_title('Error between current and goal states')
    plt.legend()
    save_current_figure('sample_buffer', store_folder=log_folder)

# TODO remove
def presentable_plots(messages_by_channel: dict, trajectory_params: dict,
                      log_folder: str = None):
    # Get some information from the trajectory parameters.
    pos_tol = trajectory_params[TRAJ_PARAM_POS_TOL_KEY]
    rad_tol = trajectory_params[TRAJ_PARAM_RAD_TOL_KEY]

    # Get relevant messages over time.
    sample_buffers = messages_by_channel['SAMPLE_BUFFER'][MESSAGE_KEY]
    debugs = messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]

    times = np.array(messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY])

    n_in_buffers, n_goals_achieved, n_since_last_progress = [], [], []
    is_c3_mode_flags, pose_tracking = [], []
    pos_errors, rad_errors = [], []
    target_came_froms, switch_reasons = [], []

    for buffer, debug in zip(sample_buffers, debugs):
        n_in_buffers.append(buffer.num_in_buffer)
        n_goals_achieved.append(debug.detected_goal_changes)
        n_since_last_progress.append(debug.best_progress_steps_ago)

        is_c3_mode_flags.append(debug.is_c3_mode)
        pose_tracking.append(debug.in_pose_tracking_mode)

        pos_errors.append(debug.current_pos_error)
        rad_errors.append(debug.current_rot_error)

        target_came_froms.append(debug.source_of_pursued_target)
        switch_reasons.append(debug.mode_switch_reason)

    is_c3_mode_flags = np.array(is_c3_mode_flags)
    n_goals_achieved = np.array(n_goals_achieved)
    switch_reasons = np.array(switch_reasons)
    pos_errors = np.array(pos_errors)
    rad_errors = np.array(rad_errors)

    # Do some plots per goal.
    n_goals = n_goals_achieved[-1]
    for i in range(n_goals):
        ts = times[n_goals_achieved == i]
        is_c3s = is_c3_mode_flags[n_goals_achieved == i]
        switches = switch_reasons[n_goals_achieved == i]
        pos_es = pos_errors[n_goals_achieved == i]
        rad_es = rad_errors[n_goals_achieved == i]
        inspect_mode_switching_by_goal(
            ts, is_c3s, switches, pos_es, rad_es, pos_tol, rad_tol, i,
            log_folder=log_folder)

def inspect_mode_switching_by_goal(times: np.ndarray,
                                   is_c3_mode_flags: np.ndarray,
                                   switch_reasons: list,
                                   pos_errors: np.ndarray,
                                   rad_errors: np.ndarray,
                                   pos_tol: float, rad_tol: float,
                                   goal_num: int,
                                   log_folder: str = None):
    print(f'Making plot for goal {goal_num}...')

    times = times - times[0]
    double_t = np.repeat(times, 2)
    c3_mask, repos_mask = get_shading_masks(is_c3_mode_flags)
    deg_errors = rad_errors * 180 / np.pi

    # A more presentable plot:  position and rotation errors with mode shading
    # and mode switch lines.
    fig, axs = plt.subplots(1, 1, figsize=(12, 4))
    ax0 = axs
    ax1 = ax0.twinx()
    ax0.fill_between(double_t, 0, 1.05, where=repos_mask, color=CF_COLOR,
                     alpha=0.5, transform=ax0.get_xaxis_transform())
    ax0.fill_between(double_t, 0, 1.05, where=c3_mask, color=CR_COLOR,
                     alpha=0.5, transform=ax0.get_xaxis_transform())

    # Add mode switch lines.
    for i, mode_switch_reason in enumerate(MODE_SWITCH_LABELS):
        if i == 0:
            continue
        switch_ts = np.array(times)[np.array(switch_reasons) == i]
        prefix = ''
        for switch_t in switch_ts:
            ax0.axvline(x=switch_t, color=MODE_SWITCH_COLORS[i],
                        linewidth=4, label=prefix + mode_switch_reason)
            prefix = '_'

    ax0.plot(times, pos_errors, color=POS_ERROR_COLOR, label='Position error')
    ax1.plot(times, deg_errors, color=RAD_ERROR_COLOR,
             label='Orientation error')

    ax0.axhline(y=pos_tol, linestyle='--', color=POS_ERROR_COLOR,
                   label='Position success threshold')
    ax1.axhline(y=rad_tol * 180/np.pi, linestyle='--', color=RAD_ERROR_COLOR,
                label='Orientation success threshold')

    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('Position Error [m]')
    ax0.set_ylim([0, np.max(pos_errors) + 0.01])
    ax0.set_xlim([np.min(times), np.max(times)])
    ax1.set_ylabel('Orientation Error [deg]', color=RAD_ERROR_COLOR)
    ax1.set_ylim([0, np.max(deg_errors) + 10])
    ax1.tick_params(axis='y', labelcolor=RAD_ERROR_COLOR)
    fig.suptitle(f'Errors over Time with Mode Switching for Goal {goal_num}')

    # Legend:  need to add patches manually.
    cf_patch = Patch(facecolor=CF_COLOR, alpha=0.5, edgecolor='black',
                     linewidth=1, label='Contact-free mode')
    cr_patch = Patch(facecolor=CR_COLOR, alpha=0.5, edgecolor='black',
                     linewidth=1, label='Contact-rich mode')
    ax0.legend(
        handles=ax0.get_legend_handles_labels()[0] + \
            ax1.get_legend_handles_labels()[0] + [cf_patch, cr_patch],
        bbox_to_anchor=(1.1, 1), loc='upper left')
    plt.tight_layout()
    save_current_figure(
        f'shading_goal_{goal_num}' if log_folder is not None else \
        'shading_goal', store_folder=log_folder)

# TODO implement and remove
def inspect_lcm_traffic(messages_by_channel: dict):
    buffer_ts = messages_by_channel['SAMPLE_BUFFER'][TIME_KEY]
    franka_ts = messages_by_channel['FRANKA_STATE'][TIME_KEY]
    radio_ts = messages_by_channel['SAMPLING_C3_RADIO'][TIME_KEY]

    buffer_ts = np.array(buffer_ts)
    franka_ts = np.array(franka_ts)
    radio_ts = np.array(radio_ts)

    print(f'Max buffer time: {np.max(buffer_ts)}')
    print(f'Max Franka time: {np.max(franka_ts)}')
    print(f'Max radio time: {np.max(radio_ts)}')

    plt.figure()
    plt.plot(buffer_ts, label='Buffer times')
    plt.plot(franka_ts, label='Franka times')
    plt.plot(radio_ts, label='Radio times')
    plt.legend()
    plt.show()


class ResultsAnalyzer:
    """TODO"""
    def __init__(self, log_filepaths: List[str], channels: List[str],
                 sync_channels: List[str] = None,
                 start_times: List[float] = None, end_times: List[float] = None,
                 save_folder: str = None, verbose: bool = True):
        assert len(channels) > 0, 'Need at least one channel to visualize.'
        if start_times is not None:
            assert len(start_times) == len(log_filepaths), f'Need either no' + \
                f' or all start times.'
        else:
            start_times = [None] * len(log_filepaths)
        if end_times is not None:
            assert len(end_times) == len(log_filepaths), f'Need either no ' + \
                f'or all end times.'
        else:
            end_times = [None] * len(log_filepaths)

        self.log_filepaths = log_filepaths
        self.start_times = start_times
        self.end_times = end_times
        self._channels = channels
        self.save_folder = save_folder

        # Load and stitch together each log file.
        self.t_adj = 0
        self.messages_by_channel = {key: {TIME_KEY: [], MESSAGE_KEY: []}
                                    for key in self._channels}
        for log_file, start, end in zip(log_filepaths, start_times, end_times):
            self._add_messages_from_log(
                log_file, start_time=start, end_time=end, verbose=verbose)

        # Synchronize according to channels.
        self._synchronize_messages(
            sync_channels=sync_channels, visualize=verbose)

    def _get_trajectory_tolerances(self):
        """Get the position and orientation tolerances from the trajectory
        parameters files, ensuring that all logs have the same tolerances.
        Store them as self.pos_tol and self.rad_tol."""
        if hasattr(self, 'pos_tol') and hasattr(self, 'rad_tol'):
            return

        pos_tol = None
        rad_tol = None

        # Get the trajectory parameters.
        for log_filepath in self.log_filepaths:
            log_folder, log_filename = op.split(log_filepath)
            log_number = log_filename.split('-')[-1]

            trajectory_params_filepath = op.join(
                log_folder, f'trajectory_params_{log_number}.yaml')
            with open(trajectory_params_filepath, 'r') as file:
                traj_params = yaml.safe_load(file)

            if pos_tol is None:
                pos_tol = traj_params[TRAJ_PARAM_POS_TOL_KEY]
                rad_tol = traj_params[TRAJ_PARAM_RAD_TOL_KEY]

            else:
                assert pos_tol == traj_params[TRAJ_PARAM_POS_TOL_KEY], \
                    'Position success thresholds do not match: ' + \
                    f'{pos_tol} vs. {traj_params[TRAJ_PARAM_POS_TOL_KEY]}'
                assert rad_tol == traj_params[TRAJ_PARAM_RAD_TOL_KEY], \
                    'Orientation success thresholds do not match: ' + \
                    f'{rad_tol} vs. {traj_params[TRAJ_PARAM_RAD_TOL_KEY]}'

        self.pos_tol = pos_tol
        self.rad_tol = rad_tol

    def _add_messages_from_log(self, log_filepath: str, start_time: float = 0.0,
                              end_time: float = 1e12, verbose: bool = True):
        """Add messages and times for every channel of interest into the
        current self.messages_and_channels dictionary, appending the new data
        to the end as if it were a continuous experiment."""
        start_time = 0.0 if start_time is None else start_time
        end_time = 1e12 if end_time is None else end_time

        start_utime = int(start_time*1e6)
        end_utime = int(end_time*1e6)

        # Open the LCM log.
        log_file = EventLog(log_filepath, 'r')

        # Read through the log file.
        init_utime = log_file.read_next_event().timestamp
        event = log_file.seek_to_timestamp(init_utime + start_utime)
        event = log_file.read_next_event()
        t_init = (event.timestamp - init_utime)*1e-6

        t_of_last_goal_change = 0
        experiment_started = False
        goals_achieved = 0

        while event is not None:
            if event.timestamp - init_utime > end_utime:
                break

            if event.channel in self._channels:
                try:
                    msg_contents = ALL_CHANNELS_AND_LCMT[event.channel].decode(
                        event.data)
                except ValueError:
                    print(f'Failed to decode message from {event.channel}.')
                    breakpoint()
                secs = (event.timestamp - init_utime)*1e-6 - t_init

                # Cut off initial teleop.
                if event.channel == 'SAMPLING_C3_DEBUG':
                    if (not experiment_started) and \
                       (not msg_contents.is_teleop):
                        experiment_started = True
                        start_t = secs
                        goals_achieved = msg_contents.detected_goal_changes
                    if experiment_started and \
                       (msg_contents.detected_goal_changes > goals_achieved):
                        goals_achieved = msg_contents.detected_goal_changes
                        t_of_last_goal_change = secs - start_t + self.t_adj
                        print(f'Goal {goals_achieved} achieved at ' + \
                              f'{t_of_last_goal_change:.2f} s.')

                if experiment_started:
                    self.messages_by_channel[event.channel][TIME_KEY].append(
                        secs - start_t + self.t_adj)
                    self.messages_by_channel[event.channel][MESSAGE_KEY].append(
                        msg_contents)

            event = log_file.read_next_event()

        # Cut off the last goal since it was not achieved.
        i_cutoff = self.messages_by_channel[
            'SAMPLING_C3_DEBUG'][TIME_KEY].index(t_of_last_goal_change)
        self.messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY] = \
            self.messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY][:i_cutoff]
        self.messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY] = \
            self.messages_by_channel[
                'SAMPLING_C3_DEBUG'][MESSAGE_KEY][:i_cutoff]
        self.t_adj += t_of_last_goal_change

        if verbose:
            for channel, contents in self.messages_by_channel.items():
                print(f'Channel: {channel}')
                print(f'\tNum messages: {len(contents[TIME_KEY])}', end = ', ')
                print(f'Time range: {contents[TIME_KEY][0]:.2f} to ' + \
                    f'{contents[TIME_KEY][-1]:.2f}')
            print(f'\nFinished processing log file at {log_filepath}.\n')

    def _synchronize_messages(self, sync_channels: List[str] = None,
                             synchronize_to_channel: str = 'SAMPLING_C3_DEBUG',
                             visualize: bool = True):
        """Synchronize the messages in the sync_channels list so their times
        match to within TIME_SYNC_THRESH of every message in the
        synchronize_to_channel.  If any channel in the sync_channels list cannot
        be synchronized at a given time, the time is discarded.  The result is
        all channels in sync_channels have the same number of messages, and each
        index corresponds across channels."""
        sync_channels = sync_channels if sync_channels is not None else \
            self._channels

        # Detect which channel had the fewest messages.
        min_num_channels = np.inf
        max_num_channels = 0
        min_channel = None
        max_channel = None
        for channel in sync_channels:
            n_msgs = len(self.messages_by_channel[channel][TIME_KEY])
            if n_msgs < min_num_channels:
                min_num_channels = n_msgs
                min_channel = channel
            if n_msgs > max_num_channels:
                max_num_channels = n_msgs
                max_channel = channel
        print(f'{min_channel} had fewest messages at {min_num_channels}')
        print(f'{max_channel} had most messages at {max_num_channels}')
        print(f'Synchronizing...', end=' ', flush=True)

        if visualize:
            _fig, axs = plt.subplots(2, 2, figsize=(12, 9), sharex=True,
                                     gridspec_kw={'height_ratios': [2, 1]})
            axs[0,0].sharey(axs[0,1])
            for channel in sync_channels:
                axs[0,0].plot(self.messages_by_channel[channel][TIME_KEY],
                              marker='o', label=channel)
            axs[0,0].set_title('All messages')
            axs[0,0].set_xlabel('Index')
            axs[0,0].set_ylabel('Time (s)')
            axs[0,0].legend()

            axs[1,0].plot(
                np.array(self.messages_by_channel[min_channel][TIME_KEY]) - \
                np.array(self.messages_by_channel[max_channel][TIME_KEY][
                    :min_num_channels]))
            axs[1,0].set_title(f'Time difference {min_channel} to ' + \
                               f'{max_channel}')
            axs[1,0].set_xlabel('Index')
            axs[1,0].set_ylabel('Time difference (s)')

        # Detect timestamps that are shared between all channels.
        ts = self.messages_by_channel[synchronize_to_channel][TIME_KEY]
        problem_ts = []
        channel_problem_ts = []
        for channel in sync_channels:
            if channel == synchronize_to_channel:
                continue
            channel_ts = np.array(self.messages_by_channel[channel][TIME_KEY])
            for t in ts:
                delta = np.min(np.abs(channel_ts - t))
                if delta > TIME_SYNCH_THRESH and t not in problem_ts:
                    problem_ts.append(t)
                    channel_problem_ts.append(channel)
        for problem_t in problem_ts:
            ts.remove(problem_t)

        # Do some time synchronization to the minimum set of messages.
        for channel in sync_channels:
            new_times = []
            new_msgs = []
            channel_ts = np.array(self.messages_by_channel[channel][TIME_KEY])

            for t in ts:
                i = np.argmin(np.abs(channel_ts - t))
                new_times.append(self.messages_by_channel[channel][TIME_KEY][i])
                new_msgs.append(
                    self.messages_by_channel[channel][MESSAGE_KEY][i])

            self.messages_by_channel[channel][TIME_KEY] = new_times
            self.messages_by_channel[channel][MESSAGE_KEY] = new_msgs

        print(f'Done.')

        if visualize:
            for channel in self._channels:
                axs[0,1].plot(
                    self.messages_by_channel[channel][TIME_KEY], marker='o',
                    label=channel)
            axs[0,1].set_title('Synchronized messages')
            axs[0,1].set_xlabel('Index')
            axs[0,1].legend()

            axs[1,1].plot(
                np.array(self.messages_by_channel[min_channel][TIME_KEY]) - \
                np.array(self.messages_by_channel[max_channel][TIME_KEY]))
            axs[1,1].set_xlabel('Index')
            axs[1,1].set_ylabel('Time difference (s)')
            save_current_figure('time_sync')

    def _extract_information(self):
        """Extracts and stores the following as numpy class attributes, if not
        done already:
            - times (N,)
            - pos_errors (N,)
            - rad_errors (N,)
            - is_c3_mode_flags (N,)
            - n_goals_achieved (N,)
            - n_in_buffers (N,)
            - switch_reasons (N,)
        """
        if hasattr(self, 'times'):
            return

        # Get relevant messages over time.
        sample_buffers = self.messages_by_channel['SAMPLE_BUFFER'][MESSAGE_KEY]
        debugs = self.messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]

        self.times = np.array(
            self.messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY])

        # Things to keep track of for every timestamp.
        n_in_buffers, n_goals_achieved, n_since_last_progress = [], [], []
        is_c3_mode_flags, pose_tracking = [], []
        pos_errors, rad_errors = [], []
        target_came_froms, switch_reasons = [], []

        # Things to keep track of for every goal.
        last_goal = -1
        times_of_new_goals = []
        worst_pos_errors, worst_rad_errors = [], []
        init_pos_errors, init_rad_errors = [], []

        # Things to keep track of for every log.
        goals_per_log = []

        for buffer, debug, t in zip(sample_buffers, debugs, self.times):
            n_in_buffers.append(buffer.num_in_buffer)
            n_goals_achieved.append(debug.detected_goal_changes)
            n_since_last_progress.append(debug.best_progress_steps_ago)

            is_c3_mode_flags.append(debug.is_c3_mode)
            pose_tracking.append(debug.in_pose_tracking_mode)

            pos_errors.append(debug.current_pos_error)
            rad_errors.append(debug.current_rot_error)

            target_came_froms.append(debug.source_of_pursued_target)
            switch_reasons.append(debug.mode_switch_reason)

            if last_goal != debug.detected_goal_changes:
                if last_goal > debug.detected_goal_changes:
                    goals_per_log.append(last_goal + 1)
                last_goal = debug.detected_goal_changes
                times_of_new_goals.append(t)
                init_pos_errors.append(debug.current_pos_error)
                init_rad_errors.append(debug.current_rot_error)
                worst_pos_errors.append(debug.current_pos_error)
                worst_rad_errors.append(debug.current_rot_error)

            if debug.current_pos_error > worst_pos_errors[-1]:
                worst_pos_errors[-1] = debug.current_pos_error
            if debug.current_rot_error > worst_rad_errors[-1]:
                worst_rad_errors[-1] = debug.current_rot_error

        goals_per_log.append(debug.detected_goal_changes + 1)

        self.is_c3_mode_flags = np.array(is_c3_mode_flags)
        self.n_in_buffers = np.array(n_in_buffers)
        self.n_goals_achieved = np.array(n_goals_achieved)
        self.switch_reasons = np.array(switch_reasons)
        self.pos_errors = np.array(pos_errors)
        self.rad_errors = np.array(rad_errors)
        self.times_of_new_goals = np.array(times_of_new_goals)
        self.worst_pos_errors = np.array(worst_pos_errors)
        self.worst_rad_errors = np.array(worst_rad_errors)

        # Correct the goals achieved since they need to be cumulative across all
        # logs, and store how many goals per log.
        for log_i, downhill_i in enumerate(
            np.where(np.diff(self.n_goals_achieved) < 0)[0]):
            self.n_goals_achieved[downhill_i+1:] += goals_per_log[log_i]
        self.goals_per_log = np.array(goals_per_log)

    def _compute_times_to_thesholds(self):
        if hasattr(self, 'times_to_thresholds'):
            return

        # Use all tolerances up to as fine as the used one from the log.
        pos_thresholds = POS_SUCCESS_THRESHOLDS[
            POS_SUCCESS_THRESHOLDS >= self.pos_tol]
        rad_thresholds = RAD_SUCCESS_THRESHOLDS[
            RAD_SUCCESS_THRESHOLDS >= self.rad_tol]

        # Compute the time it took to reach each threshold level for each goal.
        n_goals = self.n_goals_achieved[-1] + 1
        times_to_thresholds = np.zeros((n_goals, len(pos_thresholds)))
        for goal_i, goal_t in enumerate(self.times_of_new_goals):
            for thresh_i, (pos_thresh, rad_thresh) in enumerate(
                zip(pos_thresholds, rad_thresholds)):

                if pos_thresh == self.pos_tol and rad_thresh == self.rad_tol:
                    if goal_i == n_goals - 1:
                        times_to_thresholds[goal_i, thresh_i] = \
                            self.times[-1] - goal_t
                    else:
                        times_to_thresholds[goal_i, thresh_i] = \
                            self.times_of_new_goals[goal_i+1] - goal_t
                    continue

                time_i = np.argmin(
                    np.abs(self.times - self.times_of_new_goals[goal_i])) + 1
                already_set = False
                while (self.pos_errors[time_i] > pos_thresh.item()) or \
                    (self.rad_errors[time_i] > rad_thresh.item()):
                    time_i += 1
                    if time_i == len(self.times):
                        assert thresh_i > 0
                        times_to_thresholds[goal_i, thresh_i] = \
                            times_to_thresholds[goal_i, thresh_i-1]
                        already_set = True
                        break
                if not already_set:
                    times_to_thresholds[goal_i, thresh_i] = \
                        self.times[time_i] - goal_t

        self.times_to_thresholds = times_to_thresholds

    # TODO:  Is there a way we can ensure the errors cross the tolerance lines?
    # Ideally this could happen with actual data (requires synchronization of
    # SAMPLING_C3_DEBUG and C3_ACTUAL/C3_FINAL_TARGET), but we could also just
    # add a final datapoint ensuring thresholds are met.
    def inspect_mode_switching_by_goal(self):
        """Generate mode switching plots for every goal achieved.  The plots
        show the position and orientation errors over time, with shading for
        contact-rich and contact-free modes, with colored vertical separation
        lines indicating the reasons for the mode switching."""
        self._extract_information()
        self._get_trajectory_tolerances()

        n_goals = self.n_goals_achieved[-1] + 1
        for i in range(n_goals):
            ts = self.times[self.n_goals_achieved == i]
            is_c3s = self.is_c3_mode_flags[self.n_goals_achieved == i]
            switches = self.switch_reasons[self.n_goals_achieved == i]
            pos_es = self.pos_errors[self.n_goals_achieved == i]
            rad_es = self.rad_errors[self.n_goals_achieved == i]
            inspect_mode_switching_by_goal(
                ts, is_c3s, switches, pos_es, rad_es, self.pos_tol,
                self.rad_tol, i+1, log_folder=self.save_folder)

    def visualize_time_histograms(self):
        """Generate time-to-goal histograms."""
        self._extract_information()
        self._get_trajectory_tolerances()
        self._compute_times_to_thesholds()

        # Use all tolerances up to as fine as the used one from the log.
        pos_thresholds = POS_SUCCESS_THRESHOLDS[
            POS_SUCCESS_THRESHOLDS >= self.pos_tol]
        rad_thresholds = RAD_SUCCESS_THRESHOLDS[
            RAD_SUCCESS_THRESHOLDS >= self.rad_tol]
        colors = THRESHOLD_COLORS[-len(pos_thresholds):]

        # Compute the time it took to reach each threshold level for each goal.
        times_to_thresholds = self.times_to_thresholds

        # Generate a plot of time histograms per threshold.
        fig, axs = plt.subplot_mosaic(
            [[0, 4], [1, 4], [2, 4], [3, 4]], constrained_layout=True,
            figsize=(9, 9), sharex=True, sharey=False)
        axs[1].sharey(axs[0])
        axs[2].sharey(axs[0])
        axs[3].sharey(axs[0])
        fig.suptitle('Time to Reach Goal')
        handles = []

        counts, bins = np.histogram(times_to_thresholds, bins=HIST_BINS,
                                    range=(0, np.max(times_to_thresholds)))

        for thresh_i, color in enumerate(colors):
            data = times_to_thresholds[:, thresh_i]
            kde = gaussian_kde(data)
            x = np.linspace(min(data), max(data), 100)
            label=f'{pos_thresholds[thresh_i]:.2f}m, ' + \
                f'{rad_thresholds[thresh_i]:.1f}rad'
            axs[thresh_i].hist(data, color=color, alpha=0.5, density=True,
                            bins=bins, label=label)
            handles.append(
                axs[thresh_i].plot(
                    x, kde(x), color=color, linewidth=4, label=label)[0])
            axs[thresh_i].set_ylabel('Probability Density')
            axs[thresh_i].yaxis.set_major_formatter(mtick.PercentFormatter(1.0))

            axs[4].hist(data, color=color, alpha=0.5, density=True, bins=bins)
            axs[4].plot(x, kde(x), color=color, linewidth=4, label=label)

        axs[3].set_xlabel('Time [s]')
        axs[4].set_xlabel('Time [s]')
        axs[4].set_ylabel('Probability Density')
        axs[4].yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
        axs[4].legend(handles=handles, title='Thresholds')
        plt.tight_layout()
        save_current_figure('time_hist', store_folder=self.save_folder)

    def visualize_goal_success(self):
        """Generate a debugging-purposed goal success plot."""
        self._extract_information()
        self._get_trajectory_tolerances()
        self._compute_times_to_thesholds()

        # Use all tolerances up to as fine as the used one from the log.
        pos_thresholds = POS_SUCCESS_THRESHOLDS[
            POS_SUCCESS_THRESHOLDS >= self.pos_tol]
        rad_thresholds = RAD_SUCCESS_THRESHOLDS[
            RAD_SUCCESS_THRESHOLDS >= self.rad_tol]
        colors = THRESHOLD_COLORS[-len(pos_thresholds):]

        # Compute the time it took to reach each threshold level for each goal.
        times_to_thresholds = self.times_to_thresholds

        # Generate debugging plot for goal success.
        fig, axs = plt.subplots(3, 1, figsize=(10, 12))
        axs[0].sharex(axs[1])
        axs[0].plot(self.times, self.pos_errors, label='Position error')
        for pos_thresh, color in zip(pos_thresholds, colors):
            axs[0].axhline(y=pos_thresh, linestyle='--', color=color,
                        label=f'{pos_thresh:.2f}m threshold')
        axs[1].plot(self.times, self.rad_errors, label='Rotation error')
        for rad_thresh, color in zip(rad_thresholds, colors):
            axs[1].axhline(y=rad_thresh, linestyle='--', color=color,
                        label=f'{rad_thresh:.1f}rad threshold')
        axs[0].set_xlabel('Time (s)')
        axs[1].set_xlabel('Time (s)')
        axs[0].set_ylabel('Error [m]')
        axs[1].set_ylabel('Error [rad]')
        axs[0].set_title('Position Error')
        axs[1].set_title('Orientation Error')
        axs[0].legend()
        axs[1].legend()
        for goal_i in range(times_to_thresholds.shape[0]):
            for thresh_i, color in enumerate(colors):
                time = times_to_thresholds[goal_i, thresh_i] + \
                    self.times_of_new_goals[goal_i]
                time_i = np.argmin(np.abs(self.times - time)).item()
                axs[0].axvline(x=time, color=color, alpha=0.5)
                axs[1].axvline(x=time, color=color, alpha=0.5)

        n_goals, n_thresholds = times_to_thresholds.shape
        x = np.arange(n_goals)
        bar_width = 0.2
        offsets = np.linspace(-bar_width * (n_thresholds - 1) / 2,
                              bar_width * (n_thresholds - 1) / 2,
                              n_thresholds)

        for thresh_i, color in enumerate(colors):
            axs[2].bar(x + offsets[thresh_i], times_to_thresholds[:, thresh_i],
                    width=0.2, color=color,
                    label=f'{pos_thresholds[thresh_i]:.2f}m, ' + \
                        f'{rad_thresholds[thresh_i]:.1f}rad')
        axs[2].set_ylabel('Time [s]')
        axs[2].set_title('Time to Reach Goal')
        axs[2].set_xticks(x)
        axs[2].set_xticklabels([f'Goal {i+1}' for i in range(n_goals)])
        axs[2].legend(title="Thresholds",
                    bbox_to_anchor=(1.02, 1), loc='upper left')
        plt.tight_layout()
        save_current_figure('goal_success', store_folder=self.save_folder)

    def visualize_time_to_goal_vs_error(self):
        """Generate time to goal versus orientation and position error plots."""
        self._extract_information()
        self._get_trajectory_tolerances()
        self._compute_times_to_thesholds()

        # Use all tolerances up to as fine as the used one from the log.
        pos_thresholds = POS_SUCCESS_THRESHOLDS[
            POS_SUCCESS_THRESHOLDS >= self.pos_tol]
        rad_thresholds = RAD_SUCCESS_THRESHOLDS[
            RAD_SUCCESS_THRESHOLDS >= self.rad_tol]
        colors = THRESHOLD_COLORS[-len(pos_thresholds):]

        # Compute the time it took to reach each threshold level for each goal.
        times_to_thresholds = self.times_to_thresholds

        # Generate a plot of time to goal versus worst errors incurred over the
        # trajectory.
        fig, axs = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
        for thresh_i, color in enumerate(colors):
            axs[0].scatter(
                self.worst_pos_errors, times_to_thresholds[:, thresh_i],
                color=color, label=f'{pos_thresholds[thresh_i]:.2f}' + \
                    f'm, {rad_thresholds[thresh_i]:.1f}rad')
            axs[1].scatter(
                self.worst_rad_errors*180/np.pi,
                times_to_thresholds[:, thresh_i], color=color,
                label=f'{pos_thresholds[thresh_i]:.2f}m, ' + \
                    f'{rad_thresholds[thresh_i]:.1f}rad')
        axs[1].legend(title='Success Thresholds')
        axs[0].set_xlabel('Position Error [m]')
        axs[1].set_xlabel('Orientation Error [deg]')
        axs[0].set_ylabel('Time to Goal [s]')
        fig.suptitle('Time to Goal vs. Worst Error Over Trajectory')
        save_current_figure('time_vs_error', store_folder=self.save_folder)

    def visualize_cdf(self):
        """Generate CDF plot."""
        self._extract_information()
        self._get_trajectory_tolerances()
        self._compute_times_to_thesholds()

        # Use all tolerances up to as fine as the used one from the log.
        pos_thresholds = POS_SUCCESS_THRESHOLDS[
            POS_SUCCESS_THRESHOLDS >= self.pos_tol]
        rad_thresholds = RAD_SUCCESS_THRESHOLDS[
            RAD_SUCCESS_THRESHOLDS >= self.rad_tol]
        colors = THRESHOLD_COLORS[-len(pos_thresholds):]

        # Compute the time it took to reach each threshold level for each goal.
        times_to_thresholds = self.times_to_thresholds

        # Generate a plot of cumulative distribution functions.
        fig, axs = plt.subplots(1, 1, figsize=(6,4))
        for thresh_i, color in enumerate(colors):
            data = times_to_thresholds[:, thresh_i]
            count, bins_count = np.histogram(
                data, bins=11, range=(0, 1.1*CDF_TIME_CUTOFF))
            pdf = count / sum(count)
            cdf = np.cumsum(pdf)
            bins_count[0] = 0
            cdf = np.concatenate([[0], cdf])
            axs.plot(bins_count, cdf, color=color, linewidth=5,
                     label=f'{pos_thresholds[thresh_i]:.2f}' + \
                        f'm, {rad_thresholds[thresh_i]:.1f}rad')
        axs.legend(title='Pose Success Thresholds')
        axs.set_xlabel('Time Limit [s]')
        axs.set_ylabel('Fraction of Goals Achieved within Time Limit')
        axs.yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
        axs.set_ylim([0, 1])
        axs.set_xlim([0, CDF_TIME_CUTOFF])
        plt.grid()
        fig.suptitle('Time to Goal Cumulative Density')
        save_current_figure('cdf', store_folder=self.save_folder)


@click.group()
def cli():
    pass


@cli.command('multi')
@click.argument('log-folders', type=click.Path(exists=True), nargs=-1,
                required=True)
@click.option('--save-to', type=str, default=None,
              help='Save the results to a folder of provided name')
@click.option('--interactive', is_flag=True,
              help='Run in interactive mode')

def multi_command(log_folders: Tuple[str], save_to: str, interactive: bool):
    channels_and_lcmt = MINIMAL_CHANNELS_AND_LCMT

    # Turn the folders into filepaths.
    log_filepaths = []
    for log_folder in log_folders:
        log_folder = log_folder[:-1] if log_folder[-1] == '/' else log_folder
        log_number = log_folder.split('/')[-1][:6]
        log_filepath = op.join(log_folder, f'simlog-{log_number}')
        log_type = 'simulation'
        if not op.exists(log_filepath):
            log_filepath = op.join(log_folder, f'hwlog-{log_number}')
            log_type = 'hardware'
        if not op.exists(log_filepath):
            raise ValueError(f'Could not find simlog or hwlog in: {log_folder}')
        print(f'Parsing {log_type} log at: {log_filepath}')
        log_filepaths.append(log_filepath)
    print('')

    if save_to is not None:
        save_folder = op.join('examples/jacktoy/test/tmp/', save_to)
        if op.exists(save_folder):
            shutil.rmtree(save_folder)
        os.makedirs(save_folder)
    else:
        save_folder = None

    if interactive:
        global global_is_interactive
        global_is_interactive = True
        plt.ion()

    results_analyzer = ResultsAnalyzer(
        log_filepaths=log_filepaths,
        channels=channels_and_lcmt.keys(),
        save_folder=save_folder,
        verbose=False
    )
    results_analyzer.visualize_cdf()
    results_analyzer.visualize_goal_success()
    results_analyzer.inspect_mode_switching_by_goal()
    results_analyzer.visualize_time_histograms()
    results_analyzer.visualize_time_to_goal_vs_error()
    breakpoint()


@cli.command('single')
@click.argument('log-folder', type=click.Path(exists=True), required=True)
@click.option('--start', type=float, default=0.0,
              help='Start time into the log to begin parsing')
@click.option('--end', type=float, default=1e12,
              help='End time into the log to stop parsing')
@click.option('--buffer-vis', is_flag=True,
              help='Visualize the buffer of samples throughout time range')
@click.option('--inspect-switching', is_flag=True,
              help='Inspect switching between C3 and repositioning')
@click.option('--visualize-goal-completion', is_flag=True,
              help='Visualize the completion of goals')
@click.option('--lcm-traffic-debug', is_flag=True,
              help='Debug LCM traffic issues')
@click.option('--all', is_flag=True,
              help='Run all visualizations')
@click.option('--interactive', is_flag=True,
              help='Run in interactive mode')
@click.option('--present', is_flag=True,
              help='Generate presentable plots')

def single_command(log_folder: str, start: float, end: float, buffer_vis: bool,
                   inspect_switching: bool, visualize_goal_completion: bool,
                   lcm_traffic_debug: bool, all: bool, interactive: bool,
                   present: bool):
    # Turn the folder into a file path.
    log_folder = log_folder[:-1] if log_folder[-1] == '/' else log_folder
    log_number = log_folder.split('/')[-1][:6]
    log_filepath = op.join(log_folder, f'simlog-{log_number}')
    log_type = 'simulation'
    if not op.exists(log_filepath):
        log_filepath = op.join(log_folder, f'hwlog-{log_number}')
        log_type = 'hardware'
    if not op.exists(log_filepath):
        raise ValueError(f'Could not find simlog or hwlog in: {log_folder}')
    print(f'Parsing {log_type} log at: {log_filepath}\n')

    channels_and_lcmt = ALL_CHANNELS_AND_LCMT if lcm_traffic_debug \
        else MINIMAL_CHANNELS_AND_LCMT if present \
        else CHANNELS_AND_LCMT_TO_SYNC

    if interactive:
        global global_is_interactive
        global_is_interactive = True
        plt.ion()

    results_analyzer = ResultsAnalyzer(
        log_filepaths=[log_filepath],
        channels=channels_and_lcmt.keys(),
        start_times=[start], end_times=[end],
        verbose=False
    )

    if visualize_goal_completion or all:
        results_analyzer.visualize_goal_success()

    if inspect_switching or all:
        results_analyzer.inspect_mode_switching_by_goal()

    # # Get the sampling parameters.
    # sampling_params_filepath = op.join(
    #     log_folder, f'sampling_params_{log_number}.yaml')
    # with open(sampling_params_filepath, 'r') as file:
    #     sampling_params = yaml.safe_load(file)

    trajectory_params_filepath = op.join(
        log_folder, f'trajectory_params_{log_number}.yaml')
    with open(trajectory_params_filepath, 'r') as file:
        traj_params = yaml.safe_load(file)


    if lcm_traffic_debug:
        inspect_lcm_traffic(results_analyzer.messages_by_channel)
        exit()

    if present:
        presentable_plots(results_analyzer.messages_by_channel,
                          traj_params, log_folder=log_folder)
        if interactive:
            breakpoint()
        exit()

    # Visualize the buffer of samples.
    if buffer_vis or all:
        visualize_sample_buffer(results_analyzer.messages_by_channel)

    if interactive:
        breakpoint()


if __name__ == '__main__':
    cli()

