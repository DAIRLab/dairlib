"""LCM log processing script.  Generates plots and videos.  Requires the
following packages:

pip install matplotlib click opencv-python tqdm PyYAML lcm drake scipy

This script can be run in 3 ways:
 1. Single mode:  analyze a single log from filepath.
 2. Multi mode:  analyze multiple logs from filepath.
 3. Yaml mode:  analyze 1+ logs from experiment type, where the yaml file
      specifies the log filepath(s) from the experiment type.

Yaml mode is recommended and can be run via, e.g.:
  python examples/sampling_c3/process_lcm_logs.py yaml letter_3
"""

import click
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import os.path as op
import sys
import tempfile
import tqdm
import yaml

from lcm import EventLog
from matplotlib.patches import Patch
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import HalfSpace, MeshcatVisualizer, StartMeshcat, \
  ClippingRange, DepthRange, DepthRenderCamera, RenderCameraCore, \
  MakeRenderEngineVtk, RenderEngineVtkParams
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor
from pydrake.trajectories import PiecewisePolynomial, \
  PiecewiseQuaternionSlerp, StackedTrajectory
from pydrake.visualization import VideoWriter


"""Import dairlib for LCM type definitions."""
DAIRLIB_DIR = op.abspath(op.dirname(op.dirname(op.dirname(__file__))))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib


CHANNEL_LCMT = {
  'SAMPLING_C3_DEBUG': dairlib.lcmt_sampling_c3_debug,
  'C3_FINAL_TARGET': dairlib.lcmt_c3_state,
  'C3_ACTUAL': dairlib.lcmt_c3_state,
}


LCM_TIME_KEY = 'lcm_timestamp'
MSG_KEY = 'lcm_message'

DT_WARNING = 1.0

GOAL_ALPHA = 0.3

CAM_RPY = RollPitchYaw(np.pi, 0, np.pi/2)
CAM_P = np.array([0.45, 0, 0.9])
CAM_FOV = np.pi/6
VIDEO_PIXELS = [320, 640]
VIDEO_FPS = 30

EXPERIMENT_YAML = op.join(
  DAIRLIB_DIR, 'examples', 'sampling_c3', 'process_lcm_logs.yaml')
with open(EXPERIMENT_YAML, 'r') as f:
  EXPERIMENT_YAML = yaml.safe_load(f)


def get_log_filepath_and_type(log_folder: str) -> Tuple[str, str]:
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

  return log_filepath, log_type


def get_log_output_folder(log_folder_or_log: str) -> str:
  if 'simlog' in log_folder_or_log or 'hwlog' in log_folder_or_log:
    log_folder = op.dirname(log_folder_or_log)
  else:
    log_folder = log_folder_or_log
  log_folder = log_folder[:-1] if log_folder[-1] == '/' else log_folder
  date = log_folder.split('/')[-2]
  log_num = int(log_folder.split('/')[-1])
  output_folder = op.abspath(
    op.join(DAIRLIB_DIR, '..', 'log_outputs', f'{date}_log_{log_num}'))
  if not op.exists(output_folder):
    os.makedirs(output_folder)
  return output_folder


def inspect_debug_timestamps(channel: str, msg_dicts: List[dict]) -> None:
  lcm_ts, msg_ts = [], []
  for msg_dict in msg_dicts:
    lcm_ts.append(msg_dict[LCM_TIME_KEY] * 1e-6)
    msg_ts.append(CHANNEL_LCMT[channel].decode(msg_dict[MSG_KEY]).utime * 1e-6)

  plt.figure(figsize=(10, 6))

  # First subplot: LCM and message timestamps
  plt.subplot(4, 1, 1)
  plt.plot(lcm_ts, label='LCM Timestamp')
  plt.plot(msg_ts, label='Message Timestamp')
  plt.xlabel('Message Index')
  plt.ylabel('Timestamp (s)')
  plt.legend()
  plt.title('Timestamps')

  # Second subplot: Difference between LCM and message timestamps
  plt.subplot(4, 1, 2)
  diff = [l - m for l, m in zip(lcm_ts, msg_ts)]
  plt.plot(diff, label='LCM - Message Timestamp')
  plt.xlabel('Message Index')
  plt.ylabel('Difference (s)')
  plt.legend()
  plt.title('Timestamp Difference')

  # Third subplot: Gaps between just LCM timestamps
  plt.subplot(4, 1, 3)
  lcm_gaps = [lcm_ts[i] - lcm_ts[i - 1] for i in range(1, len(lcm_ts))]
  plt.plot(lcm_gaps, label='LCM Gaps')
  plt.xlabel('Message Index')
  plt.ylabel('Gap (s)')
  plt.legend()

  # Fourth subplot: Number of detected goals
  if channel == 'SAMPLING_C3_DEBUG':
    num_goals = [
      CHANNEL_LCMT[channel].decode(msg[MSG_KEY]).detected_goal_changes \
      for msg in msg_dicts]
    ts_abs = [msg[LCM_TIME_KEY] * 1e-6 for msg in msg_dicts]
    ts = [t - ts_abs[0] for t in ts_abs]
    plt.subplot(4, 1, 4)
    plt.plot(ts, num_goals, label='Detected Goals')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Number of Goals')
    plt.legend()

  plt.tight_layout()
  plt.show()


def channel_to_collect(channel: str) -> bool:
  if channel in CHANNEL_LCMT.keys():
    return True
  if channel.startswith('OBJECT_') and channel.endswith('_STATE_SIMULATION'):
    return True
  return False


def synchronize_msgs(msg_dicts: List[dict]) -> List[dict]:
  synced_msg_dicts = {key: [] for key in msg_dicts.keys()}

  # Determine the channel with fewest messages.
  min_channel = min(msg_dicts, key=lambda k: len(msg_dicts[k]))
  min_count = len(msg_dicts[min_channel])
  print(f'Channel with fewest messages: {min_channel} ({min_count} messages)')
  other_channels = msg_dicts.keys() - {min_channel}

  # Find the nearest message to every message in the synchronization channel.
  synced_msg_dicts = {key: [] for key in msg_dicts.keys()}
  synced_msg_dicts[min_channel] = msg_dicts[min_channel]
  for channel in other_channels:
    for msg_dict in synced_msg_dicts[min_channel]:
      lcm_time = msg_dict[LCM_TIME_KEY]
      nearest = min(
        msg_dicts[channel], key=lambda x: abs(x[LCM_TIME_KEY] - lcm_time))
      if abs(nearest[LCM_TIME_KEY] - lcm_time) > DT_WARNING * 1e6:
        print(f'WARNING: {channel=} nearest message is ' + \
              f'{abs(nearest[LCM_TIME_KEY] - lcm_time) * 1e-6:.2f} secs away')
      synced_msg_dicts[channel].append(nearest)
    print(f'Synced {channel} to {min_channel}')

  print(f'\nAfter synchronization:')
  for channel, msgs in synced_msg_dicts.items():
    print(f'  {channel}: {len(msgs)} messages')

  return synced_msg_dicts


def add_to_msg_dict(msgs_by_channel: dict, more_msgs_by_channel: dict) -> None:
  for channel, msgs in more_msgs_by_channel.items():
    if channel in msgs_by_channel:
      msgs_by_channel[channel] += msgs
    else:
      msgs_by_channel[channel] = msgs


def load_params_from_yaml(log_filepath: str, yaml_prefix: str) -> dict:
  folder = op.dirname(log_filepath)
  log_num = folder[-6:]
  params_file = op.join(folder, f'{yaml_prefix}_{log_num}.yaml')
  if not op.exists(params_file):
    raise ValueError(f'Could not find params file: {params_file}')
  with open(params_file, 'r') as f:
    params = yaml.safe_load(f)
  return params


def get_object_names_and_models(log_filepath: str)->Tuple[List[str], List[str]]:
  params = load_params_from_yaml(log_filepath, 'sampling_c3_controller_params')
  names = params['base_names']
  models = [op.join(DAIRLIB_DIR, rel_path.replace('_controller.sdf', '.sdf')) \
            for rel_path in params['object_models']]
  return names, models


def get_tolerances(log_filepath: str) -> Tuple[float, float]:
  params = load_params_from_yaml(log_filepath, 'goal_params')
  position_tolerance = params['position_success_threshold']
  orientation_tolerance = params['orientation_success_threshold']
  return position_tolerance, orientation_tolerance


def position_error(curr_config: np.ndarray, goal_config: np.ndarray) -> float:
  return np.linalg.norm(curr_config[-3:] - goal_config[-3:])


def orientation_error(curr_config: np.ndarray, goal_config: np.ndarray) -> float:
  curr_wxyz = curr_config[:4]
  goal_wxyz = goal_config[:4]
  curr_rot = R.from_quat(curr_wxyz, scalar_first=True)
  goal_rot = R.from_quat(goal_wxyz, scalar_first=True)
  diff_rot = goal_rot * curr_rot.inv()
  return diff_rot.magnitude()


def get_shading_masks(bool_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
  bool_array = bool_array.squeeze()
  assert bool_array.ndim == 1
  bool_array = bool_array.astype(bool)
  right_shifted_yes = np.append(bool_array[0], bool_array[:-1])

  yes_shading_mask = np.ravel(np.column_stack((right_shifted_yes, bool_array)))
  no_shading_mask = np.ravel(np.column_stack((~right_shifted_yes, ~bool_array)))

  return yes_shading_mask, no_shading_mask


def plot_errors_for_goal(
    goal: int, times: np.ndarray, completed_goals: np.ndarray,
    position_errors: np.ndarray, orientation_errors: np.ndarray,
    within_tolerance: np.ndarray, in_pose_tracking_mode: np.ndarray,
    in_c3_mode: np.ndarray, object_names: list, pos_threshold: float,
    rot_threshold: float, save_to: str = None, known_success: bool = True
) -> None:
  outcome = 'Success' if known_success or goal+1 in completed_goals else \
    'Failure'
  start = np.where(completed_goals == goal)[0][0]
  end = np.where(completed_goals == goal)[0][-1] + 1

  ts = times[start:end] - times[start]
  pos_errors = position_errors[start:end]
  rot_errors = orientation_errors[start:end]
  within_tol = within_tolerance[start:end]
  in_pose_tracking_mode = in_pose_tracking_mode[start:end]
  in_c3_mode = in_c3_mode[start:end]

  # Some variables for doing C3/repositioning shading.
  double_t = np.repeat(ts, 2)
  c3_mask, repos_mask = get_shading_masks(in_c3_mode)

  fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
  for i, obj_name in enumerate(object_names):
    axs[0].plot(ts, pos_errors[:, i], label=obj_name)
    axs[1].plot(ts, rot_errors[:, i], label=obj_name)
  for ax in axs:
    ax.fill_between(double_t, 0, 1.05, where=c3_mask, color='white',
                    alpha=0.5, transform=ax.get_xaxis_transform())
    ax.fill_between(double_t, 0, 1.05, where=repos_mask, color='gray',
                    alpha=0.5, transform=ax.get_xaxis_transform())
  axs[0].axhline(y=pos_threshold, linestyle='--', color='black',
                 label=f'Success threshold')
  axs[1].axhline(y=rot_threshold, linestyle='--', color='black',
                 label=f'Success threshold')
  try:
    pose_tracking_t = ts[np.where(in_pose_tracking_mode)[0][0]]
    axs[0].axvline(x=pose_tracking_t, linestyle='--', color='purple',
                  label='Starting full pose tracking')
    axs[1].axvline(x=pose_tracking_t, linestyle='--', color='purple',
                  label='Starting full pose tracking')
  except IndexError:
    pass
  axs[0].set_ylabel('Position Error (m)', fontsize=14)
  axs[1].set_ylabel('Orientation Error (rad)', fontsize=14)
  axs[1].set_xlabel('Time (s)', fontsize=14)

  # Legend:  need to add patches manually.
  c3_patch = Patch(facecolor='white', alpha=0.5, edgecolor='black',
                   linewidth=1, label='Contact-rich mode')
  rp_patch = Patch(facecolor='gray', alpha=0.5, edgecolor='black',
                   linewidth=1, label='Contact-free mode')
  axs[0].legend(
    handles=axs[0].get_legend_handles_labels()[0] + [c3_patch, rp_patch],
    fontsize=14, bbox_to_anchor=(1.01, 1), loc='upper left')

  axs[0].set_xlim([ts[0], ts[-1]])
  axs[0].set_ylim([0, None])
  axs[1].set_ylim([0, None])
  plt.suptitle(f'Goal {goal}: {outcome}', fontsize=16)
  plt.tight_layout()

  if save_to is None:
    save_to = f'/tmp/goal_{goal}.png'
  else:
    save_to = op.join(save_to, f'goal_{goal}.png')
  plt.savefig(save_to)
  plt.close(fig)
  print(f'Wrote figure to {save_to}')


def build_object_urdf(obj_filepath: str, i: int, actual: bool) -> str:
    obj_name = op.splitext(op.basename(obj_filepath))[0]
    r = (50 * (i + 1)) % 256 / 255.0
    g = (100 * (i + 1)) % 256 / 255.0
    b = (150 * (i + 1)) % 256 / 255.0
    a = 1.0 if actual else GOAL_ALPHA
    contents = f"""
    <robot name="{obj_name}">
      <link name="{obj_name}">
        <visual>
          <geometry>
            <mesh filename="{obj_filepath}"/>
          </geometry>
          <material>
            <color rgba="{r} {g} {b} {a}" />
          </material>
        </visual>
      </link>
    </robot>
    """
    # Write the string to a tmp file
    with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as tmp_file:
      tmp_file.write(contents.encode())
      return tmp_file.name


def pretty_dict_print(d: dict) -> None:
  """Print a dictionary in a pretty format."""
  print(f'\n')
  for key, value in d.items():
    val = value
    if type(value) == float:
      val = f'{value:.2f}'
    print(f"{key}: {val}")


def log_config_from_subfolders(log_subfolders: List[str], log_dir: str
                                ) -> Tuple[List[str], List[float], List[float]]:
  log_folders, start_times, end_times = [], [], []
  for subfolder in log_subfolders:
    if ':' in subfolder:
      times = subfolder.split(':')[1:]
      if len(times) == 1:
        start_times.append(float(times[0]))
        end_times.append(None)
      elif len(times) == 2:
        start_times.append(float(times[0]))
        end_times.append(float(times[1]))
      else:
        raise ValueError(f'Invalid time format in subfolder: {subfolder}')
      subfolder = subfolder.split(':')[0]
    else:
      start_times.append(None)
      end_times.append(None)
    log_folder = op.join(log_dir, subfolder)
    assert op.exists(log_folder), f'Log folder does not exist: {log_folder}'
    log_folders.append(log_folder)
  return log_folders, start_times, end_times


class LogAnalyzer:
  def __init__(self, log_filepaths: List[str], start_times: List[float] = None,
               end_times: List[float] = None, make_plots: bool = True,
               make_videos: bool = True, cut_off_teleop: bool = True,
               cut_off_last_goal: bool = True):
    n_logs = len(log_filepaths)
    start_times = [None]*n_logs if start_times is None else start_times
    end_times = [None]*n_logs if end_times is None else end_times
    assert len(start_times) == len(log_filepaths)
    assert len(end_times) == len(log_filepaths)

    self.log_filepaths = log_filepaths
    self.start_times = start_times
    self.end_times = end_times
    self.make_plots = make_plots
    self.make_videos = make_videos
    self.cut_off_teleop = cut_off_teleop
    self.cut_off_last_goal = cut_off_last_goal

    self.msgs_by_channel = {}
    for log, start_time, end_time in zip(log_filepaths, start_times, end_times):
      self._read_log_messages(log, start_time, end_time)

    self._report_statistics()

  def _read_log_messages(self, log_filepath: str, start_time: float = None,
                         end_time: float = None):
    start_utime = 0 if start_time is None else int(start_time * 1e6)
    end_utime = 1e12 if end_time is None else int(end_time * 1e6)

    log_file = EventLog(log_filepath, 'r')
    event = log_file.read_next_event()
    init_utime = event.timestamp
    t_init = 1e-6 * init_utime if event is not None else 0
    t_start = t_init
    log_file.seek_to_timestamp(init_utime + start_utime)
    event = log_file.read_next_event()

    messages_by_channel = {}
    before_experiment = True if self.cut_off_teleop else False
    after_experiment = False

    while event is not None:
      # Detect if beyond hard cut-off.
      if event.timestamp > init_utime + end_utime:
        after_experiment = True
      # Detect if the experiment started.
      if self.cut_off_teleop and before_experiment:
        if event.channel == 'SAMPLING_C3_DEBUG':
          if not CHANNEL_LCMT[event.channel].decode(event.data).is_teleop:
            print(f'Cut off {1e-6*event.timestamp - t_init:.2f} secs of ' + \
                  f'pre-experiment')
            t_start = 1e-6 * event.timestamp
            before_experiment = False
      # Detect if the experiment ended.
      if self.cut_off_teleop and \
        (not before_experiment and not after_experiment):
        if event.channel == 'SAMPLING_C3_DEBUG':
          if CHANNEL_LCMT[event.channel].decode(event.data).is_teleop:
            after_experiment = True
      # Collect messages during the experiment.
      if not self.cut_off_teleop or \
        (not before_experiment and not after_experiment):
        message = {LCM_TIME_KEY: event.timestamp, MSG_KEY: event.data}
        if event.channel in messages_by_channel.keys():
          messages_by_channel[event.channel].append(message)
        elif channel_to_collect(event.channel):
          messages_by_channel[event.channel] = [message]
        t_end = 1e-6 * event.timestamp
      t_final = 1e-6 * event.timestamp
      event = log_file.read_next_event()

    if self.cut_off_teleop:
      print(f'Cut off {t_final - t_end:.2f} secs of post-experiment\n')
    duration = t_end - t_start
    for channel, msgs in messages_by_channel.items():
      print(f'{channel}: {len(msgs)} messages ({len(msgs)/duration:.2f} Hz)')
    print(f'\nLength of log: {t_final - t_init:.2f} secs')
    print(f'Length of experiment: {duration:.2f} secs\n')

    # inspect_debug_timestamps(
    #   'SAMPLING_C3_DEBUG', messages_by_channel['SAMPLING_C3_DEBUG'])
    synced_msgs_by_channel = synchronize_msgs(messages_by_channel)
    add_to_msg_dict(self.msgs_by_channel, synced_msgs_by_channel)

    self._extract_info_from_messages(log_filepath, synced_msgs_by_channel)

  def _extract_info_from_messages(
      self, log_filepath: str, msgs_by_channel: dict):
    list_of_objects, list_of_models = get_object_names_and_models(log_filepath)
    pos_threshold, rot_threshold = get_tolerances(log_filepath)

    n_objects = len(list_of_objects)
    n_timestamps = len(msgs_by_channel['SAMPLING_C3_DEBUG'])

    times = np.array([
      (msg[LCM_TIME_KEY]-msgs_by_channel['SAMPLING_C3_DEBUG'][0][LCM_TIME_KEY]
       ) * 1e-6 for msg in msgs_by_channel['SAMPLING_C3_DEBUG']])
    current_by_object = np.zeros((n_timestamps, n_objects, 7))
    goals_by_object = np.zeros((n_timestamps, n_objects, 7))
    ee_positions = np.zeros((n_timestamps, 3))
    completed_goals = np.zeros((n_timestamps), dtype=int)
    in_pose_tracking_mode = np.zeros((n_timestamps), dtype=bool)
    in_c3_mode = np.zeros((n_timestamps), dtype=bool)

    position_errors_by_object = np.zeros((n_timestamps, n_objects))
    orientation_errors_by_object = np.zeros((n_timestamps, n_objects))
    within_tolerance = np.zeros((n_timestamps, n_objects))

    for i in range(n_timestamps):
      goal = CHANNEL_LCMT['C3_FINAL_TARGET'].decode(
        msgs_by_channel['C3_FINAL_TARGET'][i][MSG_KEY])
      curr = CHANNEL_LCMT['C3_ACTUAL'].decode(
        msgs_by_channel['C3_ACTUAL'][i][MSG_KEY])
      for obj_i in range(n_objects):
        current_by_object[i, obj_i, :] = curr.state[3+7*obj_i:3+7*(obj_i+1)]
        goals_by_object[i, obj_i, :] = goal.state[3+7*obj_i:3+7*(obj_i+1)]
        position_errors_by_object[i, obj_i] = position_error(
          current_by_object[i, obj_i], goals_by_object[i, obj_i])
        orientation_errors_by_object[i, obj_i] = orientation_error(
          current_by_object[i, obj_i], goals_by_object[i, obj_i])
        within_tolerance[i, obj_i] = \
          position_errors_by_object[i, obj_i] <= pos_threshold and \
          orientation_errors_by_object[i, obj_i] <= rot_threshold
      ee_positions[i, :] = curr.state[:3]

      debug = CHANNEL_LCMT['SAMPLING_C3_DEBUG'].decode(
        msgs_by_channel['SAMPLING_C3_DEBUG'][i][MSG_KEY])
      completed_goals[i] = debug.detected_goal_changes
      in_pose_tracking_mode[i] = debug.in_pose_tracking_mode
      in_c3_mode[i] = debug.is_c3_mode

    if self.cut_off_last_goal:
      uncompleted_goal = completed_goals[-1]
      last_idx = np.where(completed_goals == uncompleted_goal)[0][0]

      print(f'Cutting off last (uncompleted) goal (' + \
            f'{times[-1] - times[last_idx]:.2f} secs)')

      times = times[:last_idx]
      current_by_object = current_by_object[:last_idx]
      goals_by_object = goals_by_object[:last_idx]
      ee_positions = ee_positions[:last_idx]
      completed_goals = completed_goals[:last_idx]
      in_pose_tracking_mode = in_pose_tracking_mode[:last_idx]
      in_c3_mode = in_c3_mode[:last_idx]
      position_errors_by_object = position_errors_by_object[:last_idx]
      orientation_errors_by_object = orientation_errors_by_object[:last_idx]
      within_tolerance = within_tolerance[:last_idx]

    if self.make_plots:
      for goal in np.unique(completed_goals):
        plot_errors_for_goal(
          goal, times, completed_goals, position_errors_by_object,
          orientation_errors_by_object, within_tolerance, in_pose_tracking_mode,
          in_c3_mode, list_of_objects, pos_threshold, rot_threshold,
          save_to=get_log_output_folder(log_filepath),
          known_success=self.cut_off_last_goal)

    if self.make_videos:
      Visualizer(times, completed_goals, current_by_object, goals_by_object,
                 ee_positions, list_of_models, log_filepath)

    self._save_info_from_log(
      times, current_by_object, goals_by_object, ee_positions, completed_goals,
      in_pose_tracking_mode, position_errors_by_object,
      orientation_errors_by_object, within_tolerance, list_of_objects,
      log_filepath)

  def _save_info_from_log(
      self, times: np.ndarray, current_by_object: np.ndarray,
      goals_by_object: np.ndarray, ee_positions: np.ndarray,
      completed_goals: np.ndarray, in_pose_tracking_mode: np.ndarray,
      position_errors_by_object: np.ndarray,
      orientation_errors_by_object: np.ndarray, within_tolerance: np.ndarray,
      list_of_objects: List[str], log_filepath: str):
    """Sizes:
      - times: (n_timestamps,)
      - current_by_object: (n_timestamps, n_objects, 7)
      - goals_by_object: (n_timestamps, n_objects, 7)
      - ee_positions: (n_timestamps, 3)
      - completed_goals: (n_timestamps,)
      - in_pose_tracking_mode: (n_timestamps,)
      - position_errors_by_object: (n_timestamps, n_objects)
      - orientation_errors_by_object: (n_timestamps, n_objects)
      - within_tolerance: (n_timestamps, n_objects)
      - log_indices: (n_timestamps)
    """
    log_idx = self.log_filepaths.index(log_filepath)
    log_indices = np.ones((len(times),), dtype=int) * log_idx

    n_objects = len(list_of_objects)

    if not hasattr(self, 'times'):
      self.times = np.zeros((0))
      self.current_by_object = np.zeros((0, n_objects, 7))
      self.goals_by_object = np.zeros((0, n_objects, 7))
      self.ee_positions = np.zeros((0, 3))
      self.completed_goals = np.zeros((0))
      self.in_pose_tracking_mode = np.zeros((0))
      self.position_errors_by_object = np.zeros((0, n_objects))
      self.orientation_errors_by_object = np.zeros((0, n_objects))
      self.within_tolerance = np.zeros((0, n_objects))
      self.log_indices = np.zeros((0))
      self.list_of_objects = np.array(list_of_objects)

    else:
      for obj in list_of_objects:
        assert obj in self.list_of_objects, f'Object lists do not match ' + \
          f'across logs: {list_of_objects=} for {log_filepath}, but got ' + \
          f'{self.list_of_objects=} in {self.log_filepaths[log_idx-1]}'
      for obj in self.list_of_objects:
        assert obj in list_of_objects, f'Object lists do not match across ' + \
          f'logs: {list_of_objects=} for {log_filepath}, but got ' + \
          f'{self.list_of_objects=} in {self.log_filepaths[log_idx-1]}'

      # Avoid gaps in time and completed goals when combining logs.
      avg_dt = np.mean(np.diff(self.times))
      times = times - (times[0] - self.times[-1]) + avg_dt
      completed_goals = completed_goals + self.completed_goals[-1] + 1

    # Concatenate from all logs.
    self.times = np.concatenate((self.times, times), axis=0)
    self.current_by_object = np.concatenate(
      (self.current_by_object, current_by_object), axis=0)
    self.goals_by_object = np.concatenate(
      (self.goals_by_object, goals_by_object), axis=0)
    self.ee_positions = np.concatenate(
      (self.ee_positions, ee_positions), axis=0)
    self.completed_goals = np.concatenate(
      (self.completed_goals, completed_goals), axis=0)
    self.in_pose_tracking_mode = np.concatenate(
      (self.in_pose_tracking_mode, in_pose_tracking_mode), axis=0)
    self.position_errors_by_object = np.concatenate(
      (self.position_errors_by_object, position_errors_by_object), axis=0)
    self.orientation_errors_by_object = np.concatenate(
      (self.orientation_errors_by_object, orientation_errors_by_object), axis=0)
    self.within_tolerance = np.concatenate(
      (self.within_tolerance, within_tolerance), axis=0)
    self.log_indices = np.concatenate((self.log_indices, log_indices), axis=0)

  def _report_statistics(self):
    # Get the indices of new completed goal changes.
    completed_goal_indices = np.where(np.diff(self.completed_goals) == 1)[0] + 1
    finished_goal_ts = self.times[completed_goal_indices]
    # Include the first and last timestamp to get the start of the first goal
    # and end of the last goal.
    avg_dt = np.mean(np.diff(self.times))
    t_splits = np.concatenate(
      (np.array([0]), finished_goal_ts,
       np.array([self.times[-1] + avg_dt])))

    assert t_splits.shape[0] == len(np.unique(self.completed_goals)) + 1

    times_to_goal = t_splits[1:] - t_splits[:-1]
    stats = {
      "num achieved goals": len(times_to_goal),
      "mean time to goal": np.mean(times_to_goal).item(),
      "standard deviation": np.std(times_to_goal).item(),
      "min time to goal": np.min(times_to_goal).item(),
      "max time to goal": np.max(times_to_goal).item(),
      "all times to goal": times_to_goal
    }
    pretty_dict_print(stats)


class Visualizer:
  def __init__(self, times: np.ndarray, completed_goals: np.ndarray,
               currs_by_object: np.ndarray, goals_by_object: np.ndarray,
               ee_positions: np.ndarray, object_models: List[str],
               log_filepath: str):
    self.times = times
    self.completed_goals = completed_goals
    self.currs_by_object = currs_by_object
    self.goals_by_object = goals_by_object
    self.ee_positions = ee_positions
    self.object_models = object_models

    self.video_folder = get_log_output_folder(log_filepath)

    self.build_diagram()
    self.make_video()
    for goal in np.unique(completed_goals):
      self.make_video(goal=goal)

  def build_diagram(self):
    builder = DiagramBuilder()
    mbp_config = MultibodyPlantConfig(time_step=0)
    plant, scene_graph = AddMultibodyPlant(mbp_config, builder)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)

    for i, object_model in enumerate(self.object_models):
      print(object_model)
      obj_file = object_model.replace('.sdf', '.obj')
      parser.AddModels(build_object_urdf(obj_file, i, actual=True))[0]
      parser.AddModels(build_object_urdf(obj_file, i, actual=False))[0]

    plant.RegisterVisualGeometry(
      plant.world_body(), RigidTransform(p=np.array([0, 0, -0.029])),
      HalfSpace(), 'table', np.array([0.5, 0.5, 0.5, 0.5]))
    plant.Finalize()
    plant.set_name('plant')

    # Get position names:
    # plant.GetPositionNames(add_model_instance_prefix=False)

    # Add a vtk renderer; necessary to add video writers not with the
    # VideoWriter.AddToBuilder method.
    if not scene_graph.HasRenderer('vtk'):
      scene_graph.AddRenderer(
        'vtk', MakeRenderEngineVtk(RenderEngineVtkParams()))

    # Add a world-locked camera and video writer.
    intrinsics = CameraInfo(
      width=VIDEO_PIXELS[1], height=VIDEO_PIXELS[0], fov_y=CAM_FOV)
    clip = ClippingRange(0.01, 10.0)
    camera = DepthRenderCamera(
      RenderCameraCore("vtk", intrinsics, clip, RigidTransform()),
      DepthRange(0.01, 10.0)
    )
    sensor = RgbdSensor(
      plant.GetBodyFrameIdOrThrow(plant.world_body().index()),
      RigidTransform(rpy=CAM_RPY, p=CAM_P),
      camera
    )
    builder.AddSystem(sensor)
    builder.Connect(scene_graph.GetOutputPort('query'),
                    sensor.GetInputPort('geometry_query'))
    video_writer = VideoWriter(
      filename=op.join(self.video_folder, 'tmp.mp4'), fps=VIDEO_FPS,
      backend="cv2")
    builder.AddSystem(video_writer)
    video_writer.ConnectRgbdSensor(builder=builder, sensor=sensor)

    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Build the diagram.
    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_context()
    diagram.ForcedPublish(context)

    # Save things for later.
    self.plant = plant
    self.video_writer = video_writer
    self.diagram = diagram
    self.simulator = simulator

  def speed_up_video(self, video_filepath: str, speed_factor: int):
    new_video_path = video_filepath.replace('.mp4', f'_{speed_factor}x.mp4')

    cap = cv2.VideoCapture(video_filepath)
    if not cap.isOpened():
      raise IOError(f"Cannot open video file: {video_filepath}")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(new_video_path, fourcc, VIDEO_FPS,
                          (VIDEO_PIXELS[1], VIDEO_PIXELS[0]))
    frame_idx = 0
    while True:
      ret, frame = cap.read()
      if not ret:
        break
      if frame_idx % speed_factor == 0:
        out.write(frame)
      frame_idx += 1

    cap.release()
    out.release()
    print(f"Sped-up video saved to: {new_video_path}")

  def make_video(self, goal: int = -1):
    video_name = 'full.mp4' if goal==-1 else f'goal_{goal}.mp4'
    video_filepath = op.join(self.video_folder, video_name)
    self.video_writer._filename = video_filepath

    indices = np.where(self.completed_goals == goal)[0] if goal >= 0 else \
      np.arange(len(self.times))
    ts = self.times[indices]
    currs_by_obj = self.currs_by_object[indices]
    goals_by_obj = self.goals_by_object[indices]

    system_traj = StackedTrajectory()
    for obj_i in range(currs_by_obj.shape[1]):
      quat_traj = PiecewiseQuaternionSlerp(
        breaks=ts,
        quaternions=[
          Quaternion(q/np.linalg.norm(q)) for q in currs_by_obj[:, obj_i, :4]])
      pos_traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        breaks=ts,
        samples=currs_by_obj[:, obj_i, 4:7].T,
        sample_dot_at_start=np.zeros(3),
        sample_dot_at_end=np.zeros(3)
      )
      system_traj.Append(quat_traj)
      system_traj.Append(pos_traj)
      system_traj.Append(PiecewisePolynomial.ZeroOrderHold(
        breaks=ts, samples=goals_by_obj[:, obj_i, :].T))
      
    for t in tqdm.tqdm(np.arange(ts[0], ts[-1], 1/VIDEO_FPS)):
      context = self.simulator.get_context()
      plant_context = self.plant.GetMyMutableContextFromRoot(context)
      self.plant.SetPositions(plant_context, system_traj.value(t))
      self.diagram.ForcedPublish(context)

      vw_context = self.video_writer.GetMyContextFromRoot(context)
      self.video_writer._publish(vw_context)
    self.video_writer.Save()
    print(f'Wrote video to {video_filepath}')

    self.speed_up_video(video_filepath, 4)
    self.speed_up_video(video_filepath, 10)


def multi_command(log_folders: str, start_times: List[float],
                  end_times: List[float], skip_plots: bool, videos: bool):
  log_filepaths = []
  log_type = None

  n_logs = len(log_folders)
  start_times = [None]*n_logs if start_times is None else start_times
  end_times = [None]*n_logs if end_times is None else end_times
  for log_folder, start_t, end_t in zip(log_folders, start_times, end_times):
    log_filepath, new_log_type = get_log_filepath_and_type(log_folder)
    log_type = new_log_type if log_type is None else log_type
    assert log_type == new_log_type, f'Cannot combine logs of different ' + \
      f'types: {log_type=}, {new_log_type=}'
    log_filepaths.append(log_filepath)

  LogAnalyzer(log_filepaths, start_times=start_times, end_times=end_times,
              make_plots=not skip_plots, make_videos=videos)


@click.group()
def cli():
    pass


@cli.command('single')
@click.argument('log-folder', type=click.Path(exists=True), required=True)
@click.option('--start-time', type=float, default=0,
              help='start time in seconds')
@click.option('--end-time', type=float, default=0, help='end time in seconds')
@click.option('--skip-plots', is_flag=True, help='skip plot generation')
@click.option('--videos', is_flag=True, help='run video generation')
def single_command(log_folder: str, start_time: float, end_time: float,
                   skip_plots: bool, videos: bool):
  log_filepath, _log_type = get_log_filepath_and_type(log_folder)
  la = LogAnalyzer(
    [log_filepath], start_times=[start_time], end_times=[end_time],
    make_plots=not skip_plots, make_videos=videos, cut_off_teleop=True,
    cut_off_last_goal=True
  )
  breakpoint()


@cli.command('multi')
@click.argument('log-folders', type=click.Path(exists=True), nargs=-1,
                required=True)
@click.option('--skip-plots', is_flag=True, help='skip plot generation')
@click.option('--videos', is_flag=True, help='run video generation')
def multi_dummy_command(log_folders: str, skip_plots: bool, videos: bool):
  multi_command(log_folders, None, None, skip_plots, videos)


@cli.command('yaml')
@click.argument('experiment', type=str, required=True)
@click.argument('log-dir', type=click.Path(exists=True),
                default='/mnt/data2/anything/logs/2025', required=False)
@click.option('--skip-plots', is_flag=True, help='skip plot generation')
@click.option('--videos', is_flag=True, help='run video generation')
def yaml_command(experiment: str, log_dir: str, skip_plots: bool, videos: bool):
  assert experiment in EXPERIMENT_YAML.keys(), f'Unknown experiment not in ' + \
    f'yaml: {experiment}'
  log_subfolders = EXPERIMENT_YAML[experiment]
  log_folders, start_times, end_times = log_config_from_subfolders(
    log_subfolders, log_dir)
  multi_command(log_folders, start_times, end_times, skip_plots, videos)


if __name__ == '__main__':
  cli()
