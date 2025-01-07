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
import numpy as np
import yaml

from pydrake.common.eigen_geometry import Quaternion

import dairlib


# Add to this dictionary to include more LCM channels from which to read.
CHANNELS_AND_LCMT = {
    'C3_ACTUAL': dairlib.lcmt_c3_state,
    'C3_FINAL_TARGET': dairlib.lcmt_c3_state,
    'SAMPLE_BUFFER': dairlib.lcmt_sample_buffer,
    'SAMPLE_LOCATIONS': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLE_COSTS': dairlib.lcmt_timestamped_saved_traj,
    'C3_TRAJECTORY_ACTOR_CURR_PLAN': dairlib.lcmt_timestamped_saved_traj,
    'C3_TRAJECTORY_ACTOR_BEST_PLAN': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLING_CONTROLLER_DEBUG': dairlib.lcmt_sampling_controller_debug,
}
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
TO_C3_LOWER_COST_SWITCH_LABEL = 'Repos->C3 due to lower cost'
TO_C3_REACHED_TARGET_SWITCH_LABEL = 'Repos->C3 because reached target'
TO_REPOS_LOWER_COST_SWITCH_LABEL = 'C3->Repos due to lower cost'
TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL = 'C3->Repos due to unproductivity'
TO_C3_XBOX_FORCED_SWITCH_LABEL = 'Repos->C3 due to Xbox'
MODE_SWITCH_LABELS = [NO_SWITCH_LABEL,
                      TO_C3_LOWER_COST_SWITCH_LABEL,
                      TO_C3_REACHED_TARGET_SWITCH_LABEL,
                      TO_REPOS_LOWER_COST_SWITCH_LABEL,
                      TO_REPOS_UNPRODUCTIVE_SWITCH_LABEL,
                      TO_C3_XBOX_FORCED_SWITCH_LABEL]
MODE_SWITCH_COLORS = ['black', 'red', 'orange', 'green', 'blue', 'purple']

# Other success thresholds.
POS_SUCCESS_THRESHOLDS = np.array([0.01, 0.02, 0.03, 0.04, 0.05])
RAD_SUCCESS_THRESHOLDS = np.array([0.05, 0.1, 0.2, 0.3, 0.4])
THRESHOLD_COLORS = ['black', 'red', 'darkorange', 'gold', 'green']

EPS = 1e-5



def get_messages_from_log(log_file_path: str, start_time: float = 0.0,
                          end_time: float = 1e12,
                          channels_and_lcmt: dict = CHANNELS_AND_LCMT,
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

# TODO:  This does a very naive thing but should really get addressed better.
def ensure_same_number_messages(messages_by_channel: dict,
                                channels_and_lcmt: dict = CHANNELS_AND_LCMT):
    min_num_channels = np.inf
    for channel in channels_and_lcmt.keys():
        if len(messages_by_channel[channel][TIME_KEY]) < min_num_channels:
            min_num_channels = len(messages_by_channel[channel][TIME_KEY])

    for channel in channels_and_lcmt.keys():
        if len(messages_by_channel[channel][TIME_KEY]) > min_num_channels:
            print(f'Trimming {channel} messages from ' + \
                  f'{len(messages_by_channel[channel][TIME_KEY])} to ' + \
                    f'{min_num_channels}')
            messages_by_channel[channel][TIME_KEY] = \
                messages_by_channel[channel][TIME_KEY][:min_num_channels]
            messages_by_channel[channel][MESSAGE_KEY] = \
                messages_by_channel[channel][MESSAGE_KEY][:min_num_channels]
    print('')

def angular_difference_from_quats(q1: np.ndarray, q2: np.ndarray) -> float:
    assert q1.shape == q2.shape == (4,), f'q1: {q1.shape}, q2: {q2.shape}'
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    quat1 = Quaternion(q1.reshape(4,1))
    quat2 = Quaternion(q2.reshape(4,1))

    quat_diff = quat1.inverse().multiply(quat2)
    return 2*np.arccos(quat_diff.w()) % np.pi

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

def visualize_sample_buffer(messages_by_channel: dict):
    # First start with just a simple matplotlib plot of the buffer contents.
    sample_buffers = messages_by_channel['SAMPLE_BUFFER'][MESSAGE_KEY]
    states = messages_by_channel['C3_ACTUAL'][MESSAGE_KEY]
    goal_states = messages_by_channel['C3_FINAL_TARGET'][MESSAGE_KEY]

    times = messages_by_channel['SAMPLE_BUFFER'][TIME_KEY]

    n_in_buffers = []
    quats, xyzs, ee_xyzs = [], [], []
    goal_quats, goal_xyzs = [], []
    # Store orientation error in full rotation units for easier viewing on same
    # axis as meter distance error.
    pos_errors, full_rotation_errors = [], []
    for buffer, state, goal in zip(sample_buffers, states, goal_states):
        n_in_buffers.append(buffer.num_in_buffer)
        quats.append(state.state[3:7])
        xyzs.append(state.state[7:10])
        ee_xyzs.append(state.state[:3])
        goal_quats.append(goal.state[3:7])
        goal_xyzs.append(goal.state[7:10])

        pos_errors.append(np.linalg.norm(
            np.array(xyzs[-1]) - np.array(goal_xyzs[-1])))
        full_rotation_errors.append(angular_difference_from_quats(
            np.array(quats[-1]), np.array(goal_quats[-1])) / (2*np.pi))

    fig, axs = plt.subplots(2, 1, figsize=(6, 9), sharex=True)
    axs[0].plot(times, n_in_buffers)
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Number of samples')
    axs[0].set_title('Number of active samples in buffer')
    axs[1].plot(times, pos_errors, label='Position error')
    axs[1].plot(times, full_rotation_errors, label='Rotation error')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Error [m or full rotation]')
    axs[1].set_ylim([0, 1.05])
    axs[1].set_title('Error between current and goal states')
    plt.legend()
    plt.savefig('examples/jacktoy/test/tmp/sample_buffer.png')
    print(f'Wrote plot to examples/jacktoy/test/tmp/sample_buffer.png')

def inspect_mode_switching(messages_by_channel: dict):
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
    debugs = messages_by_channel['SAMPLING_CONTROLLER_DEBUG'][MESSAGE_KEY]

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

        quat = c3_actual.state[3:7]
        xyz = c3_actual.state[7:10]
        goal_quat = goal_state.state[3:7]
        goal_xyz = goal_state.state[7:10]
        pos_errors.append(np.linalg.norm(
            np.array(xyz) - np.array(goal_xyz)))
        full_rotation_errors.append(angular_difference_from_quats(
            np.array(quat), np.array(goal_quat)) / (2*np.pi))

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

    plt.savefig('examples/jacktoy/test/tmp/sample_sources.png')
    print(f'Wrote plot to examples/jacktoy/test/tmp/sample_sources.png')

def visualize_goal_success(messages_by_channel: dict, trajectory_params: dict):
    states = messages_by_channel['C3_ACTUAL'][MESSAGE_KEY]
    goal_states = messages_by_channel['C3_FINAL_TARGET'][MESSAGE_KEY]

    times = messages_by_channel['SAMPLE_BUFFER'][TIME_KEY]

    times_of_new_goals = []
    quats, xyzs, ee_xyzs = [], [], []
    goal_quats, goal_xyzs = [], []
    pos_errors, rad_errors = [], []
    for state, goal, t in zip(states, goal_states, times):
        quats.append(np.array(state.state[3:7]))
        xyzs.append(np.array(state.state[7:10]))
        ee_xyzs.append(np.array(state.state[:3]))
        goal_quats.append(np.array(goal.state[3:7]))
        goal_xyzs.append(np.array(goal.state[7:10]))

        pos_errors.append(np.linalg.norm(
            np.array(xyzs[-1]) - np.array(goal_xyzs[-1])).item())
        rad_errors.append(angular_difference_from_quats(
            np.array(quats[-1]), np.array(goal_quats[-1])).item())

        if (len(goal_quats) == 1) or \
           (goal_quats[-1] != goal_quats[-2]).any() or \
           (goal_xyzs[-1] != goal_xyzs[-2]).any():
            times_of_new_goals.append(t)

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

            time_i = times.index(goal_t)
            while (pos_errors[time_i] > pos_thresh.item()) or \
                  (rad_errors[time_i] > rad_thresh.item()):
                time_i += 1
                if time_i == len(times):
                    print(f'Goal {goal_i} not reached.')
                    breakpoint()
            times_to_thresholds[goal_i, thresh_i] = times[time_i] - goal_t

    # Generate plots.
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    axs[0].plot(times, pos_errors, label='Position error')
    for pos_thresh, color in zip(pos_thresholds, colors):
        axs[0].axhline(y=pos_thresh, linestyle='--', color=color,
                       label=f'{pos_thresh:.2f}m threshold')
    # axs[1].plot(times, rad_errors, label='Rotation error')
    axs[1].plot(rad_errors, label='Rotation error')
    for rad_thresh, color in zip(rad_thresholds, colors):
        axs[1].axhline(y=rad_thresh, linestyle='--', color=color,
                       label=f'{rad_thresh:.1f}rad threshold')
    axs[0].set_xlabel('Time (s)')
    axs[1].set_xlabel('Time index')
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
            time_i = times.index(time)
            axs[0].axvline(x=time, color=color, alpha=0.5)
            axs[1].axvline(x=time_i, color=color, alpha=0.5)

    # Make third plot for time to reach each goal.
    n_goals, n_thresholds = times_to_thresholds.shape
    x = np.arange(n_goals)
    bar_width = 0.2
    offsets = np.linspace(-bar_width * (n_thresholds - 1) / 2,
                           bar_width * (n_thresholds - 1) / 2,
                          n_thresholds)

    # Plot each threshold as a separate set of bars
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

    plt.savefig('examples/jacktoy/test/tmp/goal_success.png')
    print(f'Wrote plot to examples/jacktoy/test/tmp/goal_success.png')



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

def main_command(log_folder: str, start: float, end: float, buffer_vis: bool,
                 inspect_switching: bool, visualize_goal_completion: bool):
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

    # Get the messages from the log.
    messages_by_channel = get_messages_from_log(
        log_filepath, start_time=start, end_time=end)
    ensure_same_number_messages(messages_by_channel)

    # Visualize the buffer of samples.
    if buffer_vis:
        visualize_sample_buffer(messages_by_channel)

    # Inspect switching of modes.
    if inspect_switching:
        inspect_mode_switching(messages_by_channel)

    # Visualize goal completion.
    if visualize_goal_completion:
        visualize_goal_success(messages_by_channel, trajectory_params)


if __name__ == '__main__':
    main_command()

