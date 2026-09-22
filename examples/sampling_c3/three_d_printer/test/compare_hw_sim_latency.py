"""Compares the printer's command-tracking behavior across hardware and sim logs.

The real printer driver queues a quarter second of motion ahead in Klipper, so
the head trails its command by roughly half a second.  Simulation models that
with sim_params.yaml's `actuator_delay` and `command_time_constant`; this script
measures what a log actually did, so the two can be compared directly.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/compare_hw_sim_latency.py \
      ~/3d_printer/logs/2026/09_17_26/000000 \
      ~/3d_printer/logs/2026/09_21_26/000000
"""

import click
import numpy as np
import os.path as op
import sys

from lcm import EventLog
from typing import Dict, List, Optional, Tuple

"""Import dairlib for LCM type definitions."""
DAIRLIB_DIR = op.abspath(
  op.dirname(op.dirname(op.dirname(op.dirname(op.dirname(__file__))))))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib


# The sim and hardware run the same controller on differently-named channels.
HW_CHANNELS = {
  'state': 'PRINTER_STATE',
  'command': 'PRINTER_INPUT',
}
SIM_CHANNELS = {
  'state': 'PRINTER_STATE_SIMULATION',
  'command': 'PRINTER_INPUT_SIMULATION',
}
SHARED_CHANNELS = {
  'actual': 'C3_ACTUAL',
  'goal': 'C3_FINAL_TARGET',
}

# Resampling grid for everything but the model fit.  Fast enough to see the
# hardware's 30 Hz command staircase, slow enough that a >90 degree heading
# change between samples means a real reversal rather than sensor noise.
DT = 0.02
FIT_DT = 0.01

# Everything is low-passed at this cutoff before speeds are taken.  The
# hardware's position readback is a bursty staircase -- it reports no motion on
# most 30 Hz ticks and then jumps -- so raw differences there are meaningless.
LOWPASS_HZ = 4.0

# A command is "moving" when its low-passed xy speed exceeds this, in m/s.
MOVING_SPEED = 0.005

# Grid searched by the first-order-plus-dead-time fit, in seconds.
FIT_DELAYS = np.arange(0.0, 0.70, 0.01)
FIT_TIME_CONSTANTS = np.arange(0.0, 0.60, 0.01)


def get_log_filepath_and_type(log_folder: str) -> Tuple[str, str]:
  """Same layout as process_lcm_logs.py: <folder>/{sim,hw}log-<number>."""
  log_folder = log_folder.rstrip('/')
  log_number = op.basename(log_folder)[:6]
  for prefix, log_type in (('simlog', 'simulation'), ('hwlog', 'hardware')):
    log_filepath = op.join(log_folder, f'{prefix}-{log_number}')
    if op.exists(log_filepath):
      return log_filepath, log_type
  raise ValueError(f'Could not find simlog or hwlog in: {log_folder}')


def read_channels(log_filepath: str, channels: List[str]) -> Dict[str, list]:
  events = {channel: [] for channel in channels}
  for event in EventLog(log_filepath, 'r'):
    if event.channel in events:
      events[event.channel].append((event.timestamp * 1e-6, event.data))
  return events


def decode_robot_output(events: list, reverse_axes: bool) -> Tuple:
  """Returns (times, positions) for an lcmt_robot_output channel.

  The simulation publishes the end effector's state straight off the plant,
  which orders the joints z, y, x; hardware reports x, y, z.  `reverse_axes`
  puts both in x, y, z.
  """
  times = np.array([time for time, _ in events])
  positions = np.array(
    [dairlib.lcmt_robot_output.decode(data).position for _, data in events])
  return times, positions[:, ::-1] if reverse_axes else positions


def decode_c3_state(events: list) -> Tuple:
  times = np.array([time for time, _ in events])
  states = np.array(
    [dairlib.lcmt_c3_state.decode(data).state for _, data in events])
  return times, states


def resample(times: np.ndarray, values: np.ndarray,
             grid: np.ndarray) -> np.ndarray:
  return np.stack(
    [np.interp(grid, times, values[:, i]) for i in range(values.shape[1])],
    axis=1)


def lowpass(values: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
  """Zero-phase moving average, so speeds are not shifted in time."""
  width = max(1, int(round(1.0 / (cutoff_hz * dt))))
  kernel = np.ones(width) / width
  padded = np.pad(values, ((width, width), (0, 0)), mode='edge')
  return np.stack(
    [np.convolve(padded[:, i], kernel, mode='same')[width:-width]
     for i in range(values.shape[1])], axis=1)


def fit_first_order_plus_dead_time(
    command_times: np.ndarray, commands: np.ndarray, state_times: np.ndarray,
    states: np.ndarray) -> Tuple[float, float, float, float]:
  """Grid searches state(t) = firstorder(tau)[command(t - delay)].

  Returns (delay, time_constant, rms, rms_without_model), all in seconds/meters.
  """
  start = max(command_times[0], state_times[0]) + 0.5
  end = min(command_times[-1], state_times[-1]) - 0.5
  grid = np.arange(start, end, FIT_DT)
  commanded = resample(command_times, commands, grid)
  measured = resample(state_times, states, grid)

  best = (np.inf, 0.0, 0.0)
  for delay in FIT_DELAYS:
    steps = int(round(delay / FIT_DT))
    if steps > 0:
      delayed = np.vstack(
        [np.repeat(commanded[:1], steps, axis=0), commanded[:-steps]])
    else:
      delayed = commanded
    for time_constant in FIT_TIME_CONSTANTS:
      if time_constant <= 0:
        filtered = delayed
      else:
        alpha = FIT_DT / (time_constant + FIT_DT)
        filtered = np.empty_like(delayed)
        value = measured[0].copy()
        for i in range(len(delayed)):
          value = value + alpha * (delayed[i] - value)
          filtered[i] = value
      rms = np.sqrt(np.mean(np.sum((filtered - measured) ** 2, axis=1)))
      if rms < best[0]:
        best = (rms, delay, time_constant)

  rms_without_model = np.sqrt(
    np.mean(np.sum((commanded - measured) ** 2, axis=1)))
  return best[1], best[2], best[0], rms_without_model


def heading_reversals(velocities: np.ndarray,
                      moving: np.ndarray) -> int:
  """Counts samples where the direction of travel flips by more than 90 deg."""
  norms = np.linalg.norm(velocities, axis=1, keepdims=True)
  unit = velocities / np.maximum(norms, 1e-12)
  dots = np.sum(unit[:-1] * unit[1:], axis=1)
  both_moving = moving[:-1] & moving[1:]
  return int(((dots < 0) & both_moving).sum())


def velocity_power_fraction(velocities: np.ndarray, dt: float, low_hz: float,
                            high_hz: float) -> float:
  centered = velocities[:, :2] - velocities[:, :2].mean(axis=0)
  frequencies = np.fft.rfftfreq(len(centered), dt)
  power = (np.abs(np.fft.rfft(centered, axis=0)) ** 2).sum(axis=1)
  power[0] = 0.0  # Drop DC; it carries no information about jitter.
  in_band = (frequencies >= low_hz) & (frequencies < high_hz)
  return power[in_band].sum() / power.sum()


def describe_speeds(label: str, velocities: np.ndarray) -> None:
  xy = np.linalg.norm(velocities[:, :2], axis=1)
  z = np.abs(velocities[:, 2])
  print(f'    {label:11s} xy mean {xy.mean() * 1e3:6.2f}  '
        f'p50 {np.percentile(xy, 50) * 1e3:6.2f}  '
        f'p90 {np.percentile(xy, 90) * 1e3:7.2f} mm/s   |   '
        f'z mean {z.mean() * 1e3:6.2f}  '
        f'p90 {np.percentile(z, 90) * 1e3:6.2f} mm/s')


def analyze(log_folder: str) -> None:
  log_filepath, log_type = get_log_filepath_and_type(log_folder)
  is_simulation = log_type == 'simulation'
  channels = dict(SIM_CHANNELS if is_simulation else HW_CHANNELS)
  channels.update(SHARED_CHANNELS)

  events = read_channels(log_filepath, list(channels.values()))
  missing = [name for name, channel in channels.items() if not events[channel]]
  if missing:
    print(f'{log_folder}: no messages on {missing}; skipping.\n')
    return

  state_times, states = decode_robot_output(events[channels['state']],
                                            reverse_axes=is_simulation)
  command_times, commands = decode_robot_output(events[channels['command']],
                                                reverse_axes=False)

  print(f'=== {log_folder}  ({log_type}, {state_times[-1] - state_times[0]:.0f} s, '
        f'state {len(state_times) / (state_times[-1] - state_times[0]):.0f} Hz, '
        f'command {len(command_times) / (command_times[-1] - command_times[0]):.0f} Hz)')

  # 1. How far behind its command the printer runs.
  delay, time_constant, rms, rms_without_model = \
    fit_first_order_plus_dead_time(command_times, commands, state_times, states)
  print(f'  dead time {delay * 1e3:5.0f} ms, time constant '
        f'{time_constant * 1e3:5.0f} ms  '
        f'(fit residual {rms * 1e3:5.2f} mm vs {rms_without_model * 1e3:5.2f} mm '
        f'with no model)')

  grid = np.arange(
    max(state_times[0], command_times[0]) + 0.5,
    min(state_times[-1], command_times[-1]) - 0.5, DT)
  measured = lowpass(resample(state_times, states, grid), DT, LOWPASS_HZ)
  commanded = lowpass(resample(command_times, commands, grid), DT, LOWPASS_HZ)
  measured_velocity = np.gradient(measured, DT, axis=0)
  commanded_velocity = np.gradient(commanded, DT, axis=0)
  commanded_xy_speed = np.linalg.norm(commanded_velocity[:, :2], axis=1)
  moving = commanded_xy_speed > MOVING_SPEED

  # 2. What that lag costs in tracking error.
  error_xy = np.linalg.norm((commanded - measured)[:, :2], axis=1)[moving]
  print(f'  |command - state| xy while commanded moving: '
        f'p50 {np.percentile(error_xy, 50) * 1e3:5.2f}  '
        f'p90 {np.percentile(error_xy, 90) * 1e3:6.2f} mm  '
        f'({moving.sum()} of {len(moving)} samples moving)')

  # 3. How fast the head actually travels.
  describe_speeds('achieved', measured_velocity)
  describe_speeds('commanded', commanded_velocity)
  above_cap = np.mean(commanded_xy_speed > 0.100) * 100
  print(f'    commanded xy speed above 100 mm/s: {above_cap:4.1f}% of the time')

  # 4. and 5. How much of the motion is back-and-forth rather than progress.
  duration = grid[-1] - grid[0]
  path_length = np.linalg.norm(np.diff(commanded, axis=0), axis=1).sum()
  reversals = heading_reversals(commanded_velocity, moving)
  print(f'  commanded heading reversals (>90 deg between samples): '
        f'{reversals} = {reversals / duration:.2f} /s = '
        f'{reversals / max(path_length, 1e-9):5.1f} per meter '
        f'(commanded path {path_length * 1e3:.0f} mm)')
  print(f'  commanded-velocity power in 0.5-5 Hz: '
        f'{velocity_power_fraction(commanded_velocity, DT, 0.5, 5.0) * 100:4.1f}%')

  # 6. Whether any of it moved the object toward the goal.
  actual_times, actual_states = decode_c3_state(events[channels['actual']])
  _goal_times, goal_states = decode_c3_state(events[channels['goal']])
  count = min(len(actual_states), len(goal_states))
  # lcmt_c3_state packs the object's quaternion in 0:7 and its position in 7:10.
  object_positions = actual_states[:count, 7:10]
  goal_error = np.linalg.norm(object_positions - goal_states[:count, 7:10],
                              axis=1)
  object_grid = np.arange(actual_times[0], actual_times[count - 1], 0.05)
  object_path = lowpass(
    resample(actual_times[:count], object_positions, object_grid), 0.05, 2.0)
  object_path_length = np.linalg.norm(np.diff(object_path, axis=0),
                                      axis=1).sum()
  object_duration = object_grid[-1] - object_grid[0]
  print(f'  object travelled {object_path_length * 1e3:.0f} mm '
        f'({object_path_length * 1e3 / object_duration:.2f} mm/s); '
        f'distance to goal {goal_error[0] * 1e3:.0f} -> '
        f'{goal_error[-1] * 1e3:.0f} mm (best {goal_error.min() * 1e3:.0f} mm)')
  print()


@click.command()
@click.argument('log-folders', type=click.Path(exists=True), nargs=-1,
                required=True)
def main(log_folders: Tuple[str]):
  """Reports command-tracking latency and jitter for each LOG_FOLDER."""
  for log_folder in log_folders:
    analyze(log_folder)


if __name__ == '__main__':
  main()
