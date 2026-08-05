"""Diagnostic for the sampling-C3 three_d_printer controller getting "stuck"
repeatedly switching repositioning targets, and for abrupt end-effector jumps.

The controller's own std::cout log lines (e.g. "Repos -> Repos: Switching to
new sample") aren't available after the fact for an already-recorded LCM log.
This script reconstructs the same information -- and more -- purely from what
the controller publishes on LCM every control loop (SAMPLING_C3_DEBUG plus the
sample cost/location/buffer and commanded-trajectory channels), and prints a
chronological report plus (optionally) diagnostic plots.

See examples/sampling_c3/process_lcm_logs.py for the sibling log-analysis
script this one borrows idioms from (log-folder resolution, EventLog usage).
This script is intentionally standalone (plain `lcm`/`dairlib`/numpy/click) so
it runs with a system python3 without a full bazel/pydrake build.

Usage:
    python3 diagnose_repositioning_stall.py <log_folder_or_file> [options]
"""

import os.path as op
import sys
from collections import Counter
from typing import Dict, List, Optional, Tuple

import click
import numpy as np

from lcm import EventLog

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib  # noqa: E402  (must come after sys.path.append above)


MODE_SWITCH_REASON = {
    0: 'no switch',
    1: 'Switching to C3 because lower in cost',
    2: 'Switching to C3 because reached repositioning target',
    3: 'Repositioning because found good sample',
    4: 'Repositioning after not making progress in C3',
    5: 'Forcing into C3 mode (xbox)',
}
PURSUED_TARGET_SOURCE = {
    0: 'none (in C3 mode)',
    1: 'previous repositioning target',
    2: 'new sample',
    3: 'sample from buffer',
}
# SampleIndex::kCurrentLocation, SampleIndex::kCurrentReposTarget.
CURRENT_LOCATION_INDEX = 0
CURRENT_REPOS_TARGET_INDEX = 1

CHANNEL_LCMT = {
    'SAMPLING_C3_DEBUG': dairlib.lcmt_sampling_c3_debug,
    'C3_ACTUAL': dairlib.lcmt_c3_state,
    'C3_TARGET': dairlib.lcmt_c3_state,
    'C3_FINAL_TARGET': dairlib.lcmt_c3_state,
    'SAMPLE_COSTS': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLE_LOCATIONS': dairlib.lcmt_timestamped_saved_traj,
    'SAMPLE_BUFFER': dairlib.lcmt_sample_buffer,
    'UNSUCCESSFUL_SAMPLE_BUFFER': dairlib.lcmt_sample_buffer,
    'TRACKING_TRAJECTORY_ACTOR': dairlib.lcmt_timestamped_saved_traj,
}


# ----------------------------------------------------------------------------
# Log loading.
# ----------------------------------------------------------------------------

def get_log_filepath(log_folder_or_file: str) -> str:
  """Resolves a log folder (containing simlog-XXXXXX/hwlog-XXXXXX) or a
  direct path to a log file into a concrete file path."""
  if op.isfile(log_folder_or_file):
    return log_folder_or_file
  log_folder = log_folder_or_file[:-1] if log_folder_or_file.endswith('/') \
      else log_folder_or_file
  log_number = op.basename(log_folder)[:6]
  for prefix in ('simlog', 'hwlog'):
    candidate = op.join(log_folder, f'{prefix}-{log_number}')
    if op.exists(candidate):
      print(f'Parsing {prefix} at: {candidate}\n')
      return candidate
  raise ValueError(f'Could not find simlog or hwlog in: {log_folder}')


def read_log(log_filepath: str) -> Dict[int, Dict[str, object]]:
  """Reads every event in the log whose channel is one we care about and
  groups the decoded messages by utime.  All the channels here are published
  by the same LeafSystem forced-publish event once per control loop, so they
  share an identical utime -- no nearest-neighbor synchronization needed."""
  log = EventLog(log_filepath, 'r')
  loops: Dict[int, Dict[str, object]] = {}
  event = log.read_next_event()
  n_events = 0
  while event is not None:
    lcmtype = CHANNEL_LCMT.get(event.channel)
    if lcmtype is not None:
      msg = lcmtype.decode(event.data)
      loops.setdefault(msg.utime, {})[event.channel] = msg
    n_events += 1
    event = log.read_next_event()
  print(f'Read {n_events} total LCM events from {log_filepath}')
  return loops


# ----------------------------------------------------------------------------
# Small decode helpers.
# ----------------------------------------------------------------------------

def get_traj_block(msg, name: str) -> Tuple[np.ndarray, np.ndarray]:
  """Pulls a named LcmTrajectory block (as encoded by LcmTrajectory /
  lcmt_saved_traj) out of a lcmt_timestamped_saved_traj message, returning
  (datapoints [num_datatypes x num_points], time_vec [num_points])."""
  saved_traj = msg.saved_traj
  for i in range(saved_traj.num_trajectories):
    if saved_traj.trajectory_names[i] == name:
      block = saved_traj.trajectories[i]
      return np.array(block.datapoints), np.array(block.time_vec)
  raise KeyError(f'No trajectory named "{name}" in saved_traj')


def state_name_map(c3_state_msg) -> Dict[str, int]:
  return {name: i for i, name in enumerate(c3_state_msg.state_names)}


def get_ee_position(c3_state_msg) -> np.ndarray:
  idx = state_name_map(c3_state_msg)
  return np.array([c3_state_msg.state[idx['end_effector_x']],
                    c3_state_msg.state[idx['end_effector_y']],
                    c3_state_msg.state[idx['end_effector_z']]])


def get_object_position(c3_state_msg, object_index: int = 0) -> np.ndarray:
  idx = state_name_map(c3_state_msg)
  return np.array([c3_state_msg.state[idx[f'object_x_{object_index}']],
                    c3_state_msg.state[idx[f'object_y_{object_index}']],
                    c3_state_msg.state[idx[f'object_z_{object_index}']]])


def point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) \
    -> float:
  """Distance from point p to the closest point on segment [a, b]."""
  ab = b - a
  denom = float(np.dot(ab, ab))
  if denom < 1e-12:
    return float(np.linalg.norm(p - a))
  t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
  closest = a + t * ab
  return float(np.linalg.norm(p - closest))


# ----------------------------------------------------------------------------
# Per-loop record assembly.
# ----------------------------------------------------------------------------

class LoopRecord:
  """One control loop's worth of synchronized data."""
  __slots__ = (
      'utime', 't', 'is_c3_mode', 'mode_switch_reason', 'source',
      'detected_goal_changes', 'lowest_cost', 'current_pos_error',
      'ee_position', 'object_position', 'goal_position',
      'sample_costs', 'sample_locations',
      'num_in_buffer', 'num_in_unsuccessful_buffer',
      'commanded_ee_position',
  )


def build_loop_records(
    loops: Dict[int, Dict[str, object]], object_index: int = 0,
    start_t: Optional[float] = None, end_t: Optional[float] = None,
) -> List[LoopRecord]:
  utimes = sorted(u for u, d in loops.items() if 'SAMPLING_C3_DEBUG' in d)
  if not utimes:
    raise ValueError('No SAMPLING_C3_DEBUG messages found in log.')
  t0 = utimes[0] * 1e-6

  records = []
  skipped = 0
  for u in utimes:
    t = u * 1e-6 - t0
    if start_t is not None and t < start_t:
      continue
    if end_t is not None and t > end_t:
      continue
    d = loops[u]
    debug = d['SAMPLING_C3_DEBUG']

    r = LoopRecord()
    r.utime = u
    r.t = t
    r.is_c3_mode = bool(debug.is_c3_mode)
    r.mode_switch_reason = debug.mode_switch_reason
    r.source = debug.source_of_pursued_target
    r.detected_goal_changes = debug.detected_goal_changes
    r.lowest_cost = debug.lowest_cost
    r.current_pos_error = debug.current_pos_error

    if 'C3_ACTUAL' in d:
      r.ee_position = get_ee_position(d['C3_ACTUAL'])
      r.object_position = get_object_position(d['C3_ACTUAL'], object_index)
    else:
      r.ee_position = None
      r.object_position = None

    if 'C3_FINAL_TARGET' in d:
      r.goal_position = get_object_position(
          d['C3_FINAL_TARGET'], object_index)
    else:
      r.goal_position = None

    if 'SAMPLE_COSTS' in d and 'SAMPLE_LOCATIONS' in d:
      try:
        costs, _ = get_traj_block(d['SAMPLE_COSTS'], 'sample_costs')
        locs, _ = get_traj_block(d['SAMPLE_LOCATIONS'], 'sample_locations')
        r.sample_costs = costs[0]  # 1 x N -> N
        r.sample_locations = locs  # 3 x N
      except KeyError:
        r.sample_costs = None
        r.sample_locations = None
    else:
      r.sample_costs = None
      r.sample_locations = None

    r.num_in_buffer = d['SAMPLE_BUFFER'].num_in_buffer \
        if 'SAMPLE_BUFFER' in d else None
    r.num_in_unsuccessful_buffer = \
        d['UNSUCCESSFUL_SAMPLE_BUFFER'].num_in_buffer \
        if 'UNSUCCESSFUL_SAMPLE_BUFFER' in d else None

    if 'TRACKING_TRAJECTORY_ACTOR' in d:
      try:
        pos, _ = get_traj_block(
            d['TRACKING_TRAJECTORY_ACTOR'], 'end_effector_position_target')
        r.commanded_ee_position = pos[:, 0]  # first (nearest-time) knot
      except KeyError:
        r.commanded_ee_position = None
    else:
      r.commanded_ee_position = None
      skipped += 1

    records.append(r)

  if skipped:
    print(f'Note: {skipped} loops missing TRACKING_TRAJECTORY_ACTOR.')
  return records


# ----------------------------------------------------------------------------
# 1. Event timeline reconstruction.
# ----------------------------------------------------------------------------

def print_event_timeline(records: List[LoopRecord]) -> None:
  print('\n' + '=' * 78)
  print('EVENT TIMELINE (mode switches + repositioning-target reassignments)')
  print('=' * 78)
  print('Note: a repositioning-target reassignment while already '
        'repositioning ("Repos -> Repos") does not by itself distinguish '
        '"previous target now in collision" from "found a better new '
        'sample" -- the LCM data can\'t tell these apart, both just show up '
        'as source_of_pursued_target -> new sample with mode_switch_reason '
        '== no switch.\n')

  def cost_suffix(cost: float) -> str:
    # lowest_cost_ is only tracked in C3 mode; it reads back as the sentinel
    # -1 while repositioning (reset by ResetProgressMetrics()).
    return f', lowest_cost={cost:.3g}' if cost >= 0 else ''

  prev = None
  n_printed = 0
  for r in records:
    if prev is not None:
      if r.mode_switch_reason != 0:
        print(f'[t={r.t:7.3f}s] {MODE_SWITCH_REASON[r.mode_switch_reason]}'
              f'  (source={PURSUED_TARGET_SOURCE[r.source]}'
              f'{cost_suffix(r.lowest_cost)})')
        n_printed += 1
      elif (not r.is_c3_mode and not prev.is_c3_mode and r.source in (2, 3)
            and (prev.source != r.source or prev.is_c3_mode != r.is_c3_mode)):
        label = 'Repos -> Repos: switching to new repositioning target' \
            if r.source == 2 else \
            'Repos -> Repos: switching to a buffered sample'
        print(f'[t={r.t:7.3f}s] {label}{cost_suffix(r.lowest_cost) or ""}')
        n_printed += 1
    prev = r
  print(f'\n{n_printed} events out of {len(records)} loops '
        f'({len(records) / max(records[-1].t, 1e-9):.1f} loops/sec average).')


# ----------------------------------------------------------------------------
# 2. Stuck-window detection.
# ----------------------------------------------------------------------------

def is_switch_event(prev: LoopRecord, r: LoopRecord) -> bool:
  if r.mode_switch_reason != 0:
    return True
  if not r.is_c3_mode and not prev.is_c3_mode and r.source in (2, 3) \
      and (prev.source != r.source or prev.is_c3_mode != r.is_c3_mode):
    return True
  return False


def find_stuck_windows(
    records: List[LoopRecord], window_sec: float, min_switches: int,
    merge_gap_sec: float,
) -> List[Tuple[float, float, int]]:
  """Flags loops where the trailing `window_sec` contains at least
  `min_switches` switch events, then merges nearby flagged stretches
  (within `merge_gap_sec` of each other) into episodes."""
  switch_times = []
  prev = None
  for r in records:
    if prev is not None and is_switch_event(prev, r):
      switch_times.append(r.t)
    prev = r
  switch_times = np.array(switch_times)

  flagged_times = []
  for t in switch_times:
    count = np.sum((switch_times >= t - window_sec) & (switch_times <= t))
    if count >= min_switches:
      flagged_times.append(t)

  if not flagged_times:
    return []

  episodes = []
  start = flagged_times[0]
  end = flagged_times[0]
  count_in_episode = 1
  for t in flagged_times[1:]:
    if t - end <= merge_gap_sec:
      end = t
      count_in_episode += 1
    else:
      n_switches = int(np.sum((switch_times >= start) & (switch_times <= end)))
      episodes.append((start, end, n_switches))
      start = t
      end = t
      count_in_episode = 1
  n_switches = int(np.sum((switch_times >= start) & (switch_times <= end)))
  episodes.append((start, end, n_switches))
  return episodes


def print_stuck_windows(episodes: List[Tuple[float, float, int]]) -> None:
  print('\n' + '=' * 78)
  print('STUCK-EPISODE DETECTION (dense repositioning-target churn)')
  print('=' * 78)
  if not episodes:
    print('No episodes found at the current --switch-window / '
          '--min-switches thresholds.')
    return
  for i, (start, end, n_switches) in enumerate(episodes):
    duration = max(end - start, 1e-9)
    print(f'Episode {i}: t=[{start:.3f}, {end:.3f}]s '
          f'(duration {duration:.2f}s), {n_switches} switch events '
          f'({n_switches / duration:.2f} switches/sec).')


# ----------------------------------------------------------------------------
# 3. Geometric "sample in the way" check.
# ----------------------------------------------------------------------------

def analyze_sample_quality(
    records: List[LoopRecord], episodes: List[Tuple[float, float, int]],
    in_the_way_threshold: float,
) -> None:
  print('\n' + '=' * 78)
  print('SAMPLE QUALITY (distance of the lowest-cost candidate to the '
        'object -> goal line)')
  print('=' * 78)
  if not episodes:
    print('No stuck episodes to analyze; skipping.')
    return

  for i, (start, end, _) in enumerate(episodes):
    n_loops = 0
    n_in_the_way = 0
    worst = None  # (distance, t)
    for r in records:
      if not (start <= r.t <= end):
        continue
      if r.sample_costs is None or r.sample_locations is None:
        continue
      if r.object_position is None or r.goal_position is None:
        continue
      n_candidates = r.sample_costs.shape[0]
      if n_candidates <= CURRENT_REPOS_TARGET_INDEX + 1:
        continue
      other_costs = r.sample_costs[CURRENT_REPOS_TARGET_INDEX:]
      best_local = int(np.argmin(other_costs)) + CURRENT_REPOS_TARGET_INDEX
      best_pos = r.sample_locations[:, best_local]
      dist = point_segment_distance(
          best_pos, r.object_position, r.goal_position)
      n_loops += 1
      if dist <= in_the_way_threshold:
        n_in_the_way += 1
      if worst is None or dist < worst[0]:
        worst = (dist, r.t)

    print(f'Episode {i}: {n_loops} loops with sample data; '
          f'{n_in_the_way} ({100 * n_in_the_way / max(n_loops, 1):.0f}%) '
          f'had their lowest-cost candidate within '
          f'{in_the_way_threshold:.3f}m of the object->goal line.')
    if worst is not None:
      print(f'  Closest approach: {worst[0]:.4f}m at t={worst[1]:.3f}s.')


# ----------------------------------------------------------------------------
# 4. Abrupt EE jump detection.
# ----------------------------------------------------------------------------

def detect_jumps(
    records: List[LoopRecord], jump_threshold: float,
) -> List[dict]:
  jumps = []
  prev = None
  for r in records:
    if prev is not None and prev.commanded_ee_position is not None \
        and r.commanded_ee_position is not None:
      dt = r.t - prev.t
      disp = np.linalg.norm(
          r.commanded_ee_position - prev.commanded_ee_position)
      if disp >= jump_threshold:
        actual_disp = None
        if prev.ee_position is not None and r.ee_position is not None:
          actual_disp = float(
              np.linalg.norm(r.ee_position - prev.ee_position))
        jumps.append({
            't': r.t, 'dt': dt, 'commanded_disp': float(disp),
            'implied_speed': float(disp / max(dt, 1e-6)),
            'actual_disp': actual_disp,
            'prev_is_c3_mode': prev.is_c3_mode, 'is_c3_mode': r.is_c3_mode,
            'mode_switch_reason': r.mode_switch_reason,
            'source': r.source, 'prev_source': prev.source,
        })
    prev = r
  return jumps


def classify_jump(j: dict) -> str:
  if j['prev_is_c3_mode'] != j['is_c3_mode']:
    return ('mode flip (C3<->repositioning) -- the two execution '
            'trajectories may not be continuous with each other')
  if not j['is_c3_mode'] and j['source'] in (2, 3) \
      and j['prev_source'] != j['source']:
    return ('repositioning target reassigned mid-move -- consistent with '
            'the piecewise-linear waypoint-height jump')
  if j['dt'] > 0.15:
    return (f'preceded by an unusually long control loop ({j["dt"]:.3f}s) '
            f'-- consistent with predicted-state extrapolation drift')
  return 'no obvious mode/target-switch or loop-timing correlate'


def print_jumps(jumps: List[dict]) -> None:
  print('\n' + '=' * 78)
  print('ABRUPT END-EFFECTOR JUMPS (in the commanded trajectory, '
        'TRACKING_TRAJECTORY_ACTOR)')
  print('=' * 78)
  if not jumps:
    print('No jumps found at the current --jump-threshold.')
    return
  for j in jumps:
    actual = f'{j["actual_disp"]:.4f}m' if j['actual_disp'] is not None \
        else 'n/a'
    print(f'[t={j["t"]:7.3f}s] commanded jump of {j["commanded_disp"]:.4f}m '
          f'over dt={j["dt"]:.3f}s (implied speed '
          f'{j["implied_speed"]:.3f} m/s); actual-state displacement: '
          f'{actual}')
    print(f'    mode: {"C3" if j["prev_is_c3_mode"] else "repositioning"} -> '
          f'{"C3" if j["is_c3_mode"] else "repositioning"}, '
          f'mode_switch_reason={MODE_SWITCH_REASON[j["mode_switch_reason"]]}, '
          f'source={PURSUED_TARGET_SOURCE[j["source"]]}')
    print(f'    Likely cause: {classify_jump(j)}')


# ----------------------------------------------------------------------------
# 5. Buffer churn.
# ----------------------------------------------------------------------------

def print_buffer_churn(
    records: List[LoopRecord], episodes: List[Tuple[float, float, int]],
) -> None:
  print('\n' + '=' * 78)
  print('SAMPLE BUFFER CHURN')
  print('=' * 78)
  if not episodes:
    print('No stuck episodes to analyze; skipping.')
    return

  for i, (start, end, _) in enumerate(episodes):
    counts = [r.num_in_buffer for r in records
              if start <= r.t <= end and r.num_in_buffer is not None]
    unsuccessful_counts = [
        (r.t, r.num_in_unsuccessful_buffer) for r in records
        if start <= r.t <= end and r.num_in_unsuccessful_buffer is not None]
    if counts:
      print(f'Episode {i}: sample buffer occupancy ranged '
            f'[{min(counts)}, {max(counts)}] over {len(counts)} loops.')
    growth_events = [
        (t, n) for (t, n), (_, n_prev) in
        zip(unsuccessful_counts[1:], unsuccessful_counts[:-1])
        if n > n_prev]
    if growth_events:
      print(f'  Unsuccessful-sample-buffer grew {len(growth_events)} '
            f'time(s) during this episode (expected only when the '
            f'controller actually re-entered C3): '
            + ', '.join(f't={t:.3f}s(->{n})' for t, n in growth_events))
    else:
      print('  Unsuccessful-sample-buffer did not grow during this episode '
            '-- expected if the controller stayed in repositioning mode '
            'the whole time (it only records samples C3 was actually '
            'attempted from).')


# ----------------------------------------------------------------------------
# 6. Plots (optional).
# ----------------------------------------------------------------------------

def make_plots(records: List[LoopRecord], jumps: List[dict], output_dir: str) \
    -> None:
  import matplotlib.pyplot as plt

  ts = np.array([r.t for r in records])
  is_c3 = np.array([r.is_c3_mode for r in records], dtype=float)
  lowest_cost = np.array([r.lowest_cost for r in records])
  commanded = np.array([
      r.commanded_ee_position if r.commanded_ee_position is not None
      else [np.nan] * 3 for r in records])
  actual = np.array([
      r.ee_position if r.ee_position is not None else [np.nan] * 3
      for r in records])

  fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

  axs[0].fill_between(ts, 0, is_c3, step='post', alpha=0.5, label='is_c3_mode')
  axs[0].set_ylabel('C3 mode')
  axs[0].set_ylim([-0.1, 1.1])

  axs[1].plot(ts, lowest_cost, label='lowest_cost')
  axs[1].set_ylabel('Lowest sample cost')
  axs[1].set_yscale('log')

  for i, label in enumerate(['x', 'y', 'z']):
    axs[2].plot(ts, commanded[:, i], label=f'commanded {label}')
  axs[2].set_ylabel('Commanded EE pos (m)')
  axs[2].legend(fontsize=8)

  for i, label in enumerate(['x', 'y', 'z']):
    axs[3].plot(ts, actual[:, i], label=f'actual {label}')
  for j in jumps:
    axs[3].axvline(j['t'], color='red', linestyle='--', alpha=0.5)
  axs[3].set_ylabel('Actual EE pos (m)')
  axs[3].set_xlabel('Time (s)')
  axs[3].legend(fontsize=8)

  plt.tight_layout()
  save_path = op.join(output_dir, 'repositioning_stall_diagnosis.png')
  plt.savefig(save_path, dpi=150)
  print(f'\nWrote diagnostic plot to {save_path}')


# ----------------------------------------------------------------------------
# CLI.
# ----------------------------------------------------------------------------

@click.command()
@click.argument('log_folder_or_file', type=click.Path(exists=True))
@click.option('--object-index', default=0, show_default=True,
              help='Which object (by index) to analyze goal-path distance '
                   'for.')
@click.option('--start', 'start_t', default=None, type=float,
              help='Only analyze loops at or after this time (s, relative '
                   'to log start).')
@click.option('--end', 'end_t', default=None, type=float,
              help='Only analyze loops at or before this time (s, relative '
                   'to log start).')
@click.option('--switch-window', default=3.0, show_default=True,
              help='Sliding window (s) used to detect dense repositioning '
                   'churn.')
@click.option('--min-switches', default=3, show_default=True,
              help='Minimum switch events within --switch-window to flag a '
                   'stuck episode.')
@click.option('--merge-gap', default=1.0, show_default=True,
              help='Merge stuck-episode detections within this many seconds '
                   'of each other.')
@click.option('--in-the-way-threshold', default=0.03, show_default=True,
              help='Distance (m) from the object->goal line within which a '
                   'candidate sample is flagged as "in the way".')
@click.option('--jump-threshold', default=0.03, show_default=True,
              help='Minimum per-loop commanded EE displacement (m) to flag '
                   'as an abrupt jump.')
@click.option('--plot', is_flag=True, help='Save diagnostic plots.')
@click.option('--plot-dir', default='/tmp', show_default=True,
              help='Directory to save plots to.')
def main(log_folder_or_file, object_index, start_t, end_t, switch_window,
          min_switches, merge_gap, in_the_way_threshold, jump_threshold,
          plot, plot_dir):
  """Diagnose stuck repositioning / abrupt EE jumps in a sampling-C3 log."""
  log_filepath = get_log_filepath(log_folder_or_file)
  loops = read_log(log_filepath)
  records = build_loop_records(
      loops, object_index=object_index, start_t=start_t, end_t=end_t)
  print(f'Assembled {len(records)} synchronized control-loop records '
        f'spanning t=[{records[0].t:.3f}, {records[-1].t:.3f}]s.')

  reason_counts = Counter(r.mode_switch_reason for r in records)
  print('\nMode-switch-reason counts over the whole (filtered) log:')
  for reason, count in sorted(reason_counts.items()):
    print(f'  {MODE_SWITCH_REASON[reason]}: {count}')

  print_event_timeline(records)

  episodes = find_stuck_windows(
      records, window_sec=switch_window, min_switches=min_switches,
      merge_gap_sec=merge_gap)
  print_stuck_windows(episodes)

  analyze_sample_quality(records, episodes, in_the_way_threshold)

  jumps = detect_jumps(records, jump_threshold)
  print_jumps(jumps)

  print_buffer_churn(records, episodes)

  if plot:
    make_plots(records, jumps, plot_dir)


if __name__ == '__main__':
  main()
