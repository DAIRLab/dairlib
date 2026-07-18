"""LCM log processing script.  Generates plots and videos."""

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

DT_WARNING = 0.5

GOAL_ALPHA = 0.3

CAM_RPY = RollPitchYaw(np.pi, 0, np.pi/2)
CAM_P = np.array([0.45, 0, 0.9])
CAM_FOV = np.pi/6
VIDEO_PIXELS = [320, 640]
VIDEO_FPS = 30


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
    lcm_ts.append(msg_dict[LCM_TIME_KEY])
    msg_ts.append(CHANNEL_LCMT[channel].decode(msg_dict[MSG_KEY]).utime)

  plt.figure(figsize=(10, 6))

  # First subplot: LCM and message timestamps
  plt.subplot(2, 1, 1)
  plt.plot(lcm_ts, label='LCM Timestamp')
  plt.plot(msg_ts, label='Message Timestamp')
  plt.xlabel('Message Index')
  plt.ylabel('Timestamp (us)')
  plt.legend()
  plt.title('Timestamps')

  # Second subplot: Difference between LCM and message timestamps
  plt.subplot(2, 1, 2)
  diff = [1e-6 * (l - m) for l, m in zip(lcm_ts, msg_ts)]
  plt.plot(diff, label='LCM - Message Timestamp')
  plt.xlabel('Message Index')
  plt.ylabel('Difference (s)')
  plt.legend()
  plt.title('Timestamp Difference')

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


def plot_errors_for_goal(
    goal: int, times: np.ndarray, completed_goals: np.ndarray,
    position_errors: np.ndarray, orientation_errors: np.ndarray,
    within_tolerance: np.ndarray, in_pose_tracking_mode: np.ndarray,
    object_names: list, pos_threshold: float, rot_threshold: float,
    save_to: str = None) -> None:
  outcome = 'Success' if goal+1 in completed_goals else 'Failure'
  start = np.where(completed_goals == goal)[0][0]
  end = np.where(completed_goals == goal)[0][-1] + 1

  ts = times[start:end] - times[start]
  pos_errors = position_errors[start:end]
  rot_errors = orientation_errors[start:end]
  within_tol = within_tolerance[start:end]
  in_pose_tracking_mode = in_pose_tracking_mode[start:end]

  fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
  for i, obj_name in enumerate(object_names):
    axs[0].plot(ts, pos_errors[:, i], label=obj_name)
    axs[1].plot(ts, rot_errors[:, i], label=obj_name)
  axs[0].axhline(y=pos_threshold, linestyle='--', color='black',
                 label=f'{pos_threshold:.2f}m threshold')
  axs[1].axhline(y=rot_threshold, linestyle='--', color='black',
                 label=f'{rot_threshold:.2f} rad threshold')
  try:
    pose_tracking_t = ts[np.where(in_pose_tracking_mode)[0][0]]
    axs[0].axvline(x=pose_tracking_t, linestyle='--', color='purple',
                  label='Starting full pose tracking')
    axs[1].axvline(x=pose_tracking_t, linestyle='--', color='purple',
                  label='Starting full pose tracking')
  except IndexError:
    pass
  axs[0].set_ylabel('Position Error (m)')
  axs[1].set_ylabel('Orientation Error (rad)')
  axs[1].set_xlabel('Time (s)')
  axs[0].legend()
  axs[1].legend()
  axs[0].set_xlim([ts[0], ts[-1]])
  axs[0].set_ylim([0, None])
  axs[1].set_ylim([0, None])
  plt.suptitle(f'Goal {goal}: {outcome}')

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


class LogAnalyzer:
  def __init__(self, log_filepaths: List[str], make_videos: bool = True):
    self.log_filepaths = log_filepaths
    self.make_videos = make_videos

    self.msgs_by_channel = {}
    for log in log_filepaths:
      self._read_log_messages(log)

  def _read_log_messages(self, log_filepath: str, cut_off_teleop: bool = True):
    log_file = EventLog(log_filepath, 'r')
    event = log_file.read_next_event()

    messages_by_channel = {}
    t_init = 1e-6 * event.timestamp if event is not None else 0
    before_experiment = True if cut_off_teleop else False
    after_experiment = False

    while event is not None:
      # Detect if the experiment started.
      if before_experiment:
        if event.channel == 'SAMPLING_C3_DEBUG':
          if not CHANNEL_LCMT[event.channel].decode(event.data).is_teleop:
            print(f'Cut off {1e-6*event.timestamp - t_init:.2f} secs of ' + \
                  f'pre-experiment teleop')
            t_start = 1e-6 * event.timestamp
            before_experiment = False
      # Detect if the experiment ended.
      if not before_experiment and not after_experiment:
        if event.channel == 'SAMPLING_C3_DEBUG':
          if CHANNEL_LCMT[event.channel].decode(event.data).is_teleop:
            after_experiment = True
      # Collect messages during the experiment.
      if not before_experiment and not after_experiment:
        message = {LCM_TIME_KEY: event.timestamp, MSG_KEY: event.data}
        if event.channel in messages_by_channel.keys():
          messages_by_channel[event.channel].append(message)
        elif channel_to_collect(event.channel):
          messages_by_channel[event.channel] = [message]
        t_end = 1e-6 * event.timestamp
      t_final = 1e-6 * event.timestamp
      event = log_file.read_next_event()

    print(f'Cut off {t_final - t_end:.2f} secs of post-experiment teleop\n')
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

    for goal in np.unique(completed_goals):
      plot_errors_for_goal(
        goal, times, completed_goals, position_errors_by_object,
        orientation_errors_by_object, within_tolerance, in_pose_tracking_mode,
        list_of_objects, pos_threshold, rot_threshold,
        save_to=get_log_output_folder(log_filepath))

    if self.make_videos:
      Visualizer(times, completed_goals, current_by_object, goals_by_object,
                 ee_positions, list_of_models, log_filepath)


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



@click.group()
def cli():
    pass


@cli.command('single')
@click.argument('log-folder', type=click.Path(exists=True), required=True)
@click.option('--skip-videos', is_flag=True, help='skip video generation')
def single_command(log_folder: str, skip_videos: bool):
  log_filepath, _log_type = get_log_filepath_and_type(log_folder)
  LogAnalyzer([log_filepath], make_videos=not skip_videos)


if __name__ == '__main__':
  cli()
