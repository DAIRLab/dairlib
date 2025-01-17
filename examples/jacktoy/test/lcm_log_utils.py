"""Get messages from desired LCM channels from an LCM log between provided
start and end times.  Uses the virtual environment installed at
dairlib/drake_env.  Requires dairlib/bazel-bin/lcmtypes to be in the
PYTHONPATH, i.e.:

source ~/workspace/dairlib/drake_env/bin/activate
export PYTHONPATH=$PYTHONPATH:/home/sharanya/workspace/dairlib/bazel-bin/lcmtypes

Example usage:

python examples/jacktoy/test/lcm_log_utils.py /mnt/data2/sharanya/logs/2024/12_16_24/000006/ --buffer-vis --inspect-switching --start=50 --end=100
"""

import click
import os.path as op
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
TO_C3_REACHED_TARGET_SWITCH_LABEL = 'Switch to Contact-Rich:  reached target'
TO_REPOS_LOWER_COST_SWITCH_LABEL = 'Switch to Contact-Free:  lower cost'
TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL = 'Switch to Contact-Free:  unproductivity'
TO_C3_XBOX_FORCED_SWITCH_LABEL = 'Switch to Contact-Rich:  Xbox'
MODE_SWITCH_LABELS = [NO_SWITCH_LABEL,
                      TO_C3_LOWER_COST_SWITCH_LABEL,
                      TO_C3_REACHED_TARGET_SWITCH_LABEL,
                      TO_REPOS_LOWER_COST_SWITCH_LABEL,
                      TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL,
                      TO_C3_XBOX_FORCED_SWITCH_LABEL]
MODE_SWITCH_COLORS = ['black', 'red', 'orange', 'green', 'blue', 'purple']
POS_ERROR_COLOR = 'black'
RAD_ERROR_COLOR = 'purple'

# Other success thresholds.
POS_SUCCESS_THRESHOLDS = np.array([0.01, 0.02, 0.03, 0.04, 0.05])
RAD_SUCCESS_THRESHOLDS = np.array([0.05, 0.1, 0.2, 0.3, 0.4])
THRESHOLD_COLORS = ['black', 'red', 'darkorange', 'gold', 'green']

EPS = 1e-5
TIME_SYNCH_THRESH = 0.03
HIST_BINS = 10

global_is_interactive = False



def get_messages_from_log(log_file_path: str, start_time: float = 0.0,
                          end_time: float = 1e12,
                          channels_and_lcmt: dict = ALL_CHANNELS_AND_LCMT,
                          verbose: bool = True):
    start_utime = int(start_time*1e6)
    end_utime = int(end_time*1e6)

    # Open the LCM log.
    log_file = EventLog(log_file_path, 'r')

    # Prepare to store the messages for every specified channel.
    messages_by_channel = {key: {TIME_KEY: [], MESSAGE_KEY: []}
                           for key in channels_and_lcmt.keys()}

    # Read through the log file.
    init_utime = log_file.read_next_event().timestamp
    event = log_file.seek_to_timestamp(init_utime + start_utime)
    event = log_file.read_next_event()

    while event is not None:
        if event.timestamp - init_utime > end_utime:
            break

        if event.channel in channels_and_lcmt.keys():
            try:
                msg_contents = channels_and_lcmt[event.channel].decode(
                    event.data)
            except ValueError:
                print(f'Failed to decode message from channel: {event.channel}')
                breakpoint()
            secs = (event.timestamp - init_utime)*1e-6

            messages_by_channel[event.channel][TIME_KEY].append(secs)
            messages_by_channel[event.channel][MESSAGE_KEY].append(msg_contents)

        event = log_file.read_next_event()

    if verbose:
        for channel, contents in messages_by_channel.items():
            print(f'Channel: {channel}')
            print(f'\tNum messages: {len(contents[TIME_KEY])}', end = ', ')
            print(f'Time range: {contents[TIME_KEY][0]:.2f} to ' + \
                f'{contents[TIME_KEY][-1]:.2f}')
        print('\nFinished processing log file.\n')

    return messages_by_channel

def synchronize_messages(
        messages_by_channel: dict,
        channels_and_lcmt_to_sync: dict = CHANNELS_AND_LCMT_TO_SYNC,
        synchronize_to_channel: str = 'SAMPLING_C3_DEBUG',
        log_folder: str = None,
        remove_initial_teleop: bool = True):
    # First remove the initial teleop messages if desired.
    if remove_initial_teleop:
        ts = messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY]
        msgs = messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]
        first_auto_idx = [msg.is_teleop for msg in msgs].index(False)

        messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY] = \
            ts[first_auto_idx:]
        messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY] = \
            msgs[first_auto_idx:]
        print(f'Removed initial teleop:  first {first_auto_idx} messages.')

    # Otherwise trim the first message off every channel since the first
    # timestamp can be off by many seconds.
    else:
        for channel in channels_and_lcmt_to_sync.keys():
            messages_by_channel[channel][TIME_KEY] = \
                messages_by_channel[channel][TIME_KEY][1:]
            messages_by_channel[channel][MESSAGE_KEY] = \
                messages_by_channel[channel][MESSAGE_KEY][1:]

    # Detect which channel had the fewest messages.
    min_num_channels = np.inf
    max_num_channels = 0
    min_channel = None
    max_channel = None
    for channel in channels_and_lcmt_to_sync.keys():
        if len(messages_by_channel[channel][TIME_KEY]) < min_num_channels:
            min_num_channels = len(messages_by_channel[channel][TIME_KEY])
            min_channel = channel
        if len(messages_by_channel[channel][TIME_KEY]) > max_num_channels:
            max_num_channels = len(messages_by_channel[channel][TIME_KEY])
            max_channel = channel
    print(f'{min_channel} had fewest messages at {min_num_channels}')
    print(f'{max_channel} had most messages at {max_num_channels}')
    print(f'Synchronizing...', end=' ', flush=True)

    _fig, axs = plt.subplots(2, 2, figsize=(12, 9), sharex=True,
                            gridspec_kw={'height_ratios': [2, 1]})
    axs[0,0].sharey(axs[0,1])
    for channel in channels_and_lcmt_to_sync.keys():
        axs[0,0].plot(messages_by_channel[channel][TIME_KEY], marker='o',
                 label=channel)
    axs[0,0].set_title('All messages')
    axs[0,0].set_xlabel('Index')
    axs[0,0].set_ylabel('Time (s)')
    axs[0,0].legend()

    axs[1,0].plot(
        np.array(messages_by_channel[min_channel][TIME_KEY]) - \
        np.array(messages_by_channel[max_channel][TIME_KEY][:min_num_channels]))
    axs[1,0].set_title(f'Time difference {min_channel} to {max_channel}')
    axs[1,0].set_xlabel('Index')
    axs[1,0].set_ylabel('Time difference (s)')

    # Detect timestamps that are shared between all channels.
    ts = messages_by_channel[synchronize_to_channel][TIME_KEY]
    problem_ts = []
    channel_problem_ts = []
    for channel in channels_and_lcmt_to_sync.keys():
        if channel == synchronize_to_channel:
            continue
        channel_ts = np.array(messages_by_channel[channel][TIME_KEY])
        for t in ts:
            delta = np.min(np.abs(channel_ts - t))
            if delta > TIME_SYNCH_THRESH and t not in problem_ts:
                problem_ts.append(t)
                channel_problem_ts.append(channel)
    for problem_t in problem_ts:
        ts.remove(problem_t)

    # Do some time synchronization to the minimum set of messages.
    for channel in channels_and_lcmt_to_sync.keys():
        new_times = []
        new_msgs = []
        channel_ts = np.array(messages_by_channel[channel][TIME_KEY])

        for t in ts:
            i = np.argmin(np.abs(channel_ts - t))
            new_times.append(messages_by_channel[channel][TIME_KEY][i])
            new_msgs.append(messages_by_channel[channel][MESSAGE_KEY][i])

        messages_by_channel[channel][TIME_KEY] = new_times
        messages_by_channel[channel][MESSAGE_KEY] = new_msgs

    print(f'Done.')

    for channel in channels_and_lcmt_to_sync.keys():
        axs[0,1].plot(messages_by_channel[channel][TIME_KEY], marker='o',
                 label=channel)
    axs[0,1].set_title('Synchronized messages')
    axs[0,1].set_xlabel('Index')
    axs[0,1].legend()

    axs[1,1].plot(
        np.array(messages_by_channel[min_channel][TIME_KEY]) - \
        np.array(messages_by_channel[max_channel][TIME_KEY]))
    axs[1,1].set_xlabel('Index')
    axs[1,1].set_ylabel('Time difference (s)')

    plt.savefig('examples/jacktoy/test/tmp/time_synch.png')
    print(f'Wrote plot to examples/jacktoy/test/tmp/time_synch.png')
    if log_folder is not None:
        plt.savefig(op.join(log_folder, 'time_synch.png'))
        print(f'Wrote plot to {op.join(log_folder, "time_synch.png")}')
    global global_is_interactive
    if not global_is_interactive:
        plt.close()

def save_current_figure(filename: str, log_folder: str = None):
    plt.savefig(f'examples/jacktoy/test/tmp/{filename}.png')
    print(f'Wrote plot to examples/jacktoy/test/tmp/{filename}.png')
    if log_folder is not None:
        plt.savefig(op.join(log_folder, f'{filename}.png'))
        print(f'Wrote plot to {op.join(log_folder, f"{filename}.png")}')
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
    save_current_figure('sample_buffer', log_folder=log_folder)

def inspect_mode_switching(messages_by_channel: dict, log_folder: str = None):
    # Get relevant messages over time.
    sample_buffers = messages_by_channel['SAMPLE_BUFFER'][MESSAGE_KEY]
    sample_locations = messages_by_channel['SAMPLE_LOCATIONS'][MESSAGE_KEY]
    sample_costs = messages_by_channel['SAMPLE_COSTS'][MESSAGE_KEY]
    c3_actuals = messages_by_channel['C3_ACTUAL'][MESSAGE_KEY]
    goal_states = messages_by_channel['C3_FINAL_TARGET'][MESSAGE_KEY]
    c3_actor_plan_currs = messages_by_channel['C3_TRAJECTORY_ACTOR_CURR_PLAN'][
        MESSAGE_KEY]
    c3_actor_plan_bests = messages_by_channel['C3_TRAJECTORY_ACTOR_BEST_PLAN'][
        MESSAGE_KEY]
    debugs = messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]

    times = messages_by_channel['SAMPLE_BUFFER'][TIME_KEY]

    n_in_buffers = []
    is_c3_mode_flags, pose_tracking = [], []
    actual_ees, curr_plan_ees, best_plan_ees, best_buffer_ees = [], [], [], []
    actual_jack_xyzs, goal_jack_xyzs = [], []
    pos_errors, full_rotation_errors = [], []
    curr_costs, best_costs = [], []
    sample_locs = []
    n_samples, n_since_last_progress = [], []
    target_came_froms, switch_reasons = [], []

    for buffer, sample_loc, sample_cost, c3_actual, goal_state, \
        plan_curr, plan_best, debug in zip(
            sample_buffers, sample_locations, sample_costs,
            c3_actuals, goal_states, c3_actor_plan_currs, c3_actor_plan_bests,
            debugs
        ):
        n_in_buffers.append(buffer.num_in_buffer)

        is_c3_mode_flags.append(debug.is_c3_mode)
        pose_tracking.append(debug.in_pose_tracking_mode)

        actual_ees.append(c3_actual.state[:3])
        actual_jack_xyzs.append(c3_actual.state[7:10])
        goal_jack_xyzs.append(goal_state.state[7:10])

        best_buffer_ees.append(buffer.configurations[n_in_buffers[-1]-1][:3])

        curr_plan_x = plan_curr.saved_traj.trajectories[1].datapoints[0][0]
        curr_plan_y = plan_curr.saved_traj.trajectories[1].datapoints[1][0]
        curr_plan_z = plan_curr.saved_traj.trajectories[1].datapoints[2][0]
        curr_plan_ees.append([curr_plan_x, curr_plan_y, curr_plan_z])

        best_plan_x = plan_best.saved_traj.trajectories[1].datapoints[0][0]
        best_plan_y = plan_best.saved_traj.trajectories[1].datapoints[1][0]
        best_plan_z = plan_best.saved_traj.trajectories[1].datapoints[2][0]
        best_plan_ees.append([best_plan_x, best_plan_y, best_plan_z])

        pos_errors.append(debug.current_pos_error)
        full_rotation_errors.append(debug.current_rot_error / (2*np.pi))

        sample_locs.append(
            np.array(sample_loc.saved_traj.trajectories[0].datapoints).T)
        n_samples.append(sample_locs[-1].shape[0])

        curr_costs.append(
            sample_cost.saved_traj.trajectories[0].datapoints[0][0])

        other_costs = np.array(
            sample_cost.saved_traj.trajectories[0].datapoints[0][1:])
        best_costs.append(np.min(other_costs).item())

        n_since_last_progress.append(debug.best_progress_steps_ago)
        target_came_froms.append(debug.source_of_pursued_target)
        switch_reasons.append(debug.mode_switch_reason)

    actual_ees = np.array(actual_ees)
    curr_plan_ees = np.array(curr_plan_ees)
    best_plan_ees = np.array(best_plan_ees)
    best_buffer_ees = np.array(best_buffer_ees)
    is_c3_mode_flags = np.array(is_c3_mode_flags)
    pos_errors = np.array(pos_errors)
    full_rotation_errors = np.array(full_rotation_errors)

    # Make a 2D plot of where the pursued samples come from.
    _fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)
    axs[-1].set_xlabel('Time (s)')
    for i, pursued_sample_source in enumerate(PURSUED_TARGET_LABELS):
        if i == 0:
            continue
        axs[0].plot(times, np.array(target_came_froms) == i,
            label=pursued_sample_source)
    axs[0].legend()
    axs[0].set_title('Source of pursued repositioning targets')

    for i, mode_switch_reason in enumerate(MODE_SWITCH_LABELS):
        if i == 0:
            continue
        axs[1].plot(times, np.array(switch_reasons) == i,
            label=mode_switch_reason, color=MODE_SWITCH_COLORS[i])
    double_t = np.repeat(np.array(times), 2)
    _c3_mask, repos_mask = get_shading_masks(is_c3_mode_flags)
    axs[1].fill_between(double_t, 0, 1.05, where=repos_mask, color='gray',
                        alpha=0.5, transform=axs[1].get_xaxis_transform())
    grey_patch = Patch(color='gray', alpha=0.5, label='Repositioning mode')
    axs[1].legend(handles=axs[1].get_legend_handles_labels()[0] + [grey_patch])
    axs[1].set_title('Reason for mode switching')

    axs[2].plot(times, n_since_last_progress, label='Number of control loops')
    axs[2].plot(times, pose_tracking, label='Pose tracking')
    axs[2].legend()
    axs[2].set_title('Number of control loops since last progress')

    axs[3].plot(times, pos_errors, label='Position error')
    axs[3].plot(times, full_rotation_errors, label='Rotation error')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Error [m or full rotation]')
    axs[3].legend()
    axs[3].set_title('Error between current and goal states')
    save_current_figure('sample_sources', log_folder=log_folder)

def presentable_plots(messages_by_channel: dict, trajectory_params: dict,
                      log_folder: str = None):
    # Get some information from the trajectory parameters.
    pos_tol = trajectory_params['position_success_threshold']
    rad_tol = trajectory_params['orientation_success_threshold']

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
    ax0.fill_between(double_t, 0, 1.05, where=repos_mask, color='gray',
                     alpha=0.5, transform=ax0.get_xaxis_transform())
    ax0.fill_between(double_t, 0, 1.05, where=c3_mask, color='white',
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
    ax1.axhline(y=rad_tol, linestyle='--', color=RAD_ERROR_COLOR,
                   label='Orientation success threshold')

    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('Position Error [m]')
    ax0.set_ylim([-0.01, np.max(pos_errors) + 0.01])
    ax0.set_xlim([np.min(times), np.max(times)])
    ax1.set_ylabel('Orientation Error [deg]', color=RAD_ERROR_COLOR)
    ax1.set_ylim([-10, np.max(deg_errors) + 10])
    ax1.tick_params(axis='y', labelcolor=RAD_ERROR_COLOR)
    fig.suptitle(f'Errors over Time with Mode Switching for Goal {goal_num}')

    # Legend:  need to add patches manually.
    grey_patch = Patch(color='gray', alpha=0.5, label='Contact-free mode')
    white_patch = Patch(facecolor='white', edgecolor='black', linewidth=1,
                        alpha=0.5, label='Contact-rich mode')
    ax0.legend(
        handles=ax0.get_legend_handles_labels()[0] + \
            ax1.get_legend_handles_labels()[0] + [grey_patch, white_patch],
        bbox_to_anchor=(1.1, 1), loc='upper left')
    plt.tight_layout()
    save_current_figure(
        f'shading_goal_{goal_num}' if log_folder is not None else \
        'shading_goal', log_folder=log_folder)

def visualize_goal_success(messages_by_channel: dict, trajectory_params: dict,
                           log_folder: str = None):
    states = messages_by_channel['C3_ACTUAL'][MESSAGE_KEY]
    goal_states = messages_by_channel['C3_FINAL_TARGET'][MESSAGE_KEY]
    debugs = messages_by_channel['SAMPLING_C3_DEBUG'][MESSAGE_KEY]

    times = messages_by_channel['SAMPLING_C3_DEBUG'][TIME_KEY]

    times_of_new_goals = []
    worst_pos_errors, worst_rad_errors = [], []
    init_pos_errors, init_rad_errors = [], []
    quats, xyzs, ee_xyzs = [], [], []
    goal_quats, goal_xyzs = [], []
    pos_errors, rad_errors = [], []
    for state, goal, debug, t in zip(states, goal_states, debugs, times):
        quats.append(np.array(state.state[3:7]))
        xyzs.append(np.array(state.state[7:10]))
        ee_xyzs.append(np.array(state.state[:3]))
        goal_quats.append(np.array(goal.state[3:7]))
        goal_xyzs.append(np.array(goal.state[7:10]))

        pos_errors.append(debug.current_pos_error)
        rad_errors.append(debug.current_rot_error)

        if len(times_of_new_goals) < debug.detected_goal_changes + 1:
            times_of_new_goals.append(t)
            init_pos_errors.append(debug.current_pos_error)
            init_rad_errors.append(debug.current_rot_error)
            worst_pos_errors.append(debug.current_pos_error)
            worst_rad_errors.append(debug.current_rot_error)

        if debug.current_pos_error > worst_pos_errors[-1]:
            worst_pos_errors[-1] = debug.current_pos_error
        if debug.current_rot_error > worst_rad_errors[-1]:
            worst_rad_errors[-1] = debug.current_rot_error

    times = np.array(times)
    pos_errors = np.array(pos_errors)
    rad_errors = np.array(rad_errors)

    # Cut off the last goal since it was not achieved.
    init_pos_errors = np.array(init_pos_errors[:-1])
    init_rad_errors = np.array(init_rad_errors[:-1])
    worst_pos_errors = np.array(worst_pos_errors[:-1])
    worst_rad_errors = np.array(worst_rad_errors[:-1])

    # Now inspect goal completion.
    n_goals = len(times_of_new_goals)

    pos_tol = trajectory_params['position_success_threshold']
    rad_tol = trajectory_params['orientation_success_threshold']

    pos_thresholds = POS_SUCCESS_THRESHOLDS[POS_SUCCESS_THRESHOLDS >= pos_tol]
    rad_thresholds = RAD_SUCCESS_THRESHOLDS[RAD_SUCCESS_THRESHOLDS >= rad_tol]
    colors = THRESHOLD_COLORS[-len(pos_thresholds):]

    times_to_thresholds = np.zeros((n_goals-1, len(pos_thresholds)))
    for goal_i, goal_t in enumerate(times_of_new_goals[:-1]):
        for thresh_i, (pos_thresh, rad_thresh) in enumerate(
            zip(pos_thresholds, rad_thresholds)):

            if pos_thresh == pos_tol and rad_thresh == rad_tol:
                times_to_thresholds[goal_i, thresh_i] = \
                    times_of_new_goals[goal_i+1] - goal_t
                continue

            time_i = np.argmin(np.abs(times - times_of_new_goals[goal_i])) + 1
            already_set = False
            while (pos_errors[time_i] > pos_thresh.item()) or \
                  (rad_errors[time_i] > rad_thresh.item()):
                time_i += 1
                if time_i == len(times):
                    assert thresh_i > 0
                    times_to_thresholds[goal_i, thresh_i] = \
                        times_to_thresholds[goal_i, thresh_i-1]
                    already_set = True
                    break
            if not already_set:
                times_to_thresholds[goal_i, thresh_i] = times[time_i] - goal_t

    # Generate plots.
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    axs[0].sharex(axs[1])
    axs[0].plot(times, pos_errors, label='Position error')
    for pos_thresh, color in zip(pos_thresholds, colors):
        axs[0].axhline(y=pos_thresh, linestyle='--', color=color,
                       label=f'{pos_thresh:.2f}m threshold')
    axs[1].plot(times, rad_errors, label='Rotation error')
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
                times_of_new_goals[goal_i]
            time_i = np.argmin(np.abs(times - time)).item()
            axs[0].axvline(x=time, color=color, alpha=0.5)
            axs[1].axvline(x=time, color=color, alpha=0.5)

    # Make third plot for time to reach each goal.
    n_goals, n_thresholds = times_to_thresholds.shape
    x = np.arange(n_goals)
    bar_width = 0.2
    offsets = np.linspace(-bar_width * (n_thresholds - 1) / 2,
                           bar_width * (n_thresholds - 1) / 2,
                          n_thresholds)

    # Plot each threshold as a separate set of bars.
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
    save_current_figure('goal_success', log_folder=log_folder)

    # A histogram of times.
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
    save_current_figure('time_hist', log_folder=log_folder)

    fig, axs = plt.subplots(1, 2, figsize=(10, 6), sharey=True)
    for thresh_i, color in enumerate(colors):
        axs[0].scatter(worst_pos_errors, times_to_thresholds[:, thresh_i],
                       color=color, label=f'{pos_thresholds[thresh_i]:.2f}' + \
                        f'm, {rad_thresholds[thresh_i]:.1f}rad')
        axs[1].scatter(worst_rad_errors*180/np.pi,
                       times_to_thresholds[:, thresh_i], color=color,
                       label=f'{pos_thresholds[thresh_i]:.2f}m, ' + \
                        f'{rad_thresholds[thresh_i]:.1f}rad')
    axs[1].legend(title='Success Thresholds')
    axs[0].set_xlabel('Position Error [m]')
    axs[1].set_xlabel('Orientation Error [deg]')
    axs[0].set_ylabel('Time to Goal [s]')
    fig.suptitle('Time to Goal vs. Worst Error Over Trajectory')
    save_current_figure('time_vs_error', log_folder=log_folder)

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



@click.command()
@click.argument('log-folder', type=str, required=True)
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

def main_command(log_folder: str, start: float, end: float, buffer_vis: bool,
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

    # Get the sampling parameters.
    sampling_params_filepath = op.join(
        log_folder, f'sampling_params_{log_number}.yaml')
    with open(sampling_params_filepath, 'r') as file:
        sampling_params = yaml.safe_load(file)

    # Get the trajectory parameters.
    trajectory_params_filepath = op.join(
        log_folder, f'trajectory_params_{log_number}.yaml')
    with open(trajectory_params_filepath, 'r') as file:
        trajectory_params = yaml.safe_load(file)

    if interactive:
        global global_is_interactive
        global_is_interactive = True
        plt.ion()

    channels_and_lcmt = ALL_CHANNELS_AND_LCMT if lcm_traffic_debug \
        else MINIMAL_CHANNELS_AND_LCMT if present \
        else CHANNELS_AND_LCMT_TO_SYNC

    # Get the messages from the log.
    messages_by_channel = get_messages_from_log(
        log_filepath, start_time=start, end_time=end,
        channels_and_lcmt=channels_and_lcmt)
    if lcm_traffic_debug:
        inspect_lcm_traffic(messages_by_channel)
        exit()

    synchronize_messages(messages_by_channel,
                         channels_and_lcmt_to_sync=channels_and_lcmt)

    if present:
        presentable_plots(messages_by_channel, trajectory_params,
                          log_folder=log_folder)
        if interactive:
            breakpoint()
        exit()

    # Visualize the buffer of samples.
    if buffer_vis or all:
        visualize_sample_buffer(messages_by_channel)

    # Inspect switching of modes.
    if inspect_switching or all:
        inspect_mode_switching(messages_by_channel)

    # Visualize goal completion.
    if visualize_goal_completion or all:
        visualize_goal_success(messages_by_channel, trajectory_params)

    if interactive:
        breakpoint()


if __name__ == '__main__':
    main_command()

