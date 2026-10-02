"""Scores the ramp's contact load on the cone, per goal, in cone demo sim logs.

new_ramp.urdf is 14 convex pieces, and the floor of its trough (piece 11) is
only 25 mm wide, so its seams with the side walls (pieces 10 and 12) run along
the trough at y = 57.75 and 82.9 mm.  Under point contact a cone corner sitting
on a seam could be pushed out through piece 11's side face, a face buried
against the wall that the one-piece hardware jig doesn't have, and the wall
pushed back.  Those opposing pairs held the cone with 0.7-20 kN (a 34.5 g cone
needs 0.3 N), so while goal 3 (mid-ramp) was the target the finger bent on
every push and the cone never moved: 0 of 22 runs (10_01/18-37, 10_02/00-11)
that got there reached it.  The same buried faces at the floor/slope seam
(x = 157.7 mm, the 0 -> 35 deg kink) pinned the apex at ~20 kN in each of the
6 tip-down-ramp flips in 10_01/28, 37 and 10_02/08, 10-12: the apex slid down
into the kink, its load jumped from < 50 N to ~20 kN in one 0.2 s sample, and
the base pushes then vaulted the cone over the pinned apex.  With the ramp
pieces rigid hydroelastic, every cone-ramp contact is hydroelastic and a buried
face only takes pressure where the cone sinks into it.

Per goal index that became the target (C3_FINAL_TARGET's object position
matched against the log's goal_params sequence), sampled at 5 Hz:
  load     the cone-ramp contact load: the sum of the force magnitudes of the
           point pairs and hydroelastic surfaces (independent of either sign
           convention), p50 / p90 / max, and the share of samples > 100 N
  stalled  seconds of finger bend > 8 mm while the cone's base and apex both
           moved < 5 mm/s, counting stretches of 0.4 s or more
  model    the share of samples with point-pair and with hydroelastic
           cone-ramp contacts
Per log, also each tip-down-ramp flip (run_sim_batch.py's rule: axis x > 0.3
and z < -0.7, here held 1 s): the goal then, the peak load in the 5 s before
it, and the share of that load within 8 mm of the apex (point pairs by
contact point, hydroelastic surfaces by centroid).

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_ramp_squeeze.py \\
      ~/3d_printer/logs/2026/10_02_26/0000{14..16}
"""

import glob
import multiprocessing
import os.path as op
import sys

import click
import numpy as np
import yaml
from lcm import EventLog

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'external', 'drake+',
                        'lcmtypes'))
sys.path.append(op.dirname(__file__))
import dairlib  # noqa: E402
import drake  # noqa: E402
from score_finger_load_guard import CONE_HEIGHT, find_log, rotation  # noqa: E402,E501

SAMPLE_DT = 0.2
STALL_BEND = 0.008
STALL_SPEED = 0.005
STALL_MIN_SAMPLES = 2
APEX_RADIUS = 0.008
FLIP_HOLD = 1.0
FLIP_LOOKBACK = 5.0


def goal_sequence(log):
  found = glob.glob(op.join(op.dirname(log), 'goal_params_*.yaml'))
  if not found:
    raise click.ClickException(f'no goal_params_*.yaml next to {log}')
  goals = yaml.safe_load(open(found[0]))
  return np.array([g[0] for g in goals['fixed_target_position_sequence']])


def read_log(log):
  sequence = goal_sequence(log)
  goal, last, t0 = None, -np.inf, None
  clean, defl, samples = [], [], []
  for event in EventLog(log, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'C3_FINAL_TARGET':
      target = np.array(dairlib.lcmt_c3_state.decode(event.data).state[7:9])
      dist = np.linalg.norm(sequence[:, :2] - target, axis=1)
      goal = int(np.argmin(dist)) if dist.min() < 0.003 else None
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      clean.append((t, *dairlib.lcmt_object_state.decode(
          event.data).position[:7]))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      p = dairlib.lcmt_robot_output.decode(event.data).position
      defl.append((t, np.hypot(p[0], p[1])))
    elif (ch == 'CONTACT_RESULTS' and goal is not None and clean and
          t - last >= SAMPLE_DT):
      last = t
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      quat, base = np.array(clean[-1][1:5]), np.array(clean[-1][5:8])
      apex = base + CONE_HEIGHT * rotation(quat)[:, 0]
      load = apex_load = 0.0
      point = hydro = False
      for c in msg.point_pair_contact_info:
        if not cone_ramp(c.body1_name, c.body2_name):
          continue
        force = np.linalg.norm(c.contact_force)
        load += force
        if np.linalg.norm(np.array(c.contact_point) - apex) < APEX_RADIUS:
          apex_load += force
        point = True
      for c in msg.hydroelastic_contacts:
        if not cone_ramp(c.body1_name, c.body2_name):
          continue
        force = np.linalg.norm(c.force_C_W)
        load += force
        if np.linalg.norm(np.array(c.centroid_W) - apex) < APEX_RADIUS:
          apex_load += force
        hydro = True
      samples.append((t, goal, load, apex_load, point, hydro))
  if not samples or not defl:
    raise click.ClickException(
        f'{log} has no CONTACT_RESULTS or FINGER_DEFLECTION_SIMULATION while '
        'a goal was the target: this scores compliant-finger sim logs only')
  return np.array(clean), np.array(defl), np.array(samples, dtype=float)


def cone_ramp(body1, body2):
  bodies = body1 + '|' + body2
  return 'cone' in bodies and 'ramp_link' in bodies


def pose_at(clean, t):
  row = clean[min(np.searchsorted(clean[:, 0], t), len(clean) - 1)]
  axis = rotation(row[1:5])[:, 0]
  return row[5:8], row[5:8] + CONE_HEIGHT * axis, axis


def stalled_seconds(stalled):
  total, run = 0.0, 0
  for s in list(stalled) + [False]:
    if s:
      run += 1
      continue
    if run >= STALL_MIN_SAMPLES:
      total += run * SAMPLE_DT
    run = 0
  return total


def flips(clean):
  """Onsets of tip-down-ramp flips held FLIP_HOLD s."""
  axes = np.array([rotation(row[1:5])[:, 0] for row in clean])
  flipped = (axes[:, 0] > 0.3) & (axes[:, 2] < -0.7)
  onsets, start = [], None
  for t, f in zip(clean[:, 0], flipped):
    if f and start is None:
      start = t
    elif not f:
      start = None
    if start is not None and t - start >= FLIP_HOLD:
      if not onsets or onsets[-1] != start:
        onsets.append(start)
  return onsets


def percentiles(x, q=(50, 90, 100)):
  return ' / '.join(f'{v:.1f}' for v in np.percentile(x, q))


def score_log(path):
  log = find_log(path)
  clean, defl, samples = read_log(log)
  record_path = op.join(op.dirname(log), 'run_record.yaml')
  record = (yaml.safe_load(open(record_path)) if op.exists(record_path)
            else {})
  goals = samples[:, 1].astype(int)
  lines = [f'{log}: furthest goal target {goals.max()}, run ended '
           f'{record.get("reason", "?")} at {record.get("stopped_at", "?")} s']
  for goal in sorted(set(goals)):
    rows = samples[goals == goal]
    bend = np.interp(rows[:, 0], defl[:, 0], defl[:, 1])
    speed = []
    for t in rows[:, 0]:
      base0, apex0, _ = pose_at(clean, t - SAMPLE_DT)
      base1, apex1, _ = pose_at(clean, t + SAMPLE_DT)
      speed.append(max(np.linalg.norm(base1 - base0),
                       np.linalg.norm(apex1 - apex0)) / (2 * SAMPLE_DT))
    stalled = (bend > STALL_BEND) & (np.array(speed) < STALL_SPEED)
    base_x = [1e3 * pose_at(clean, t)[0][0] for t in rows[:, 0]]
    lines.append(
        f'  goal {goal}: {len(rows) * SAMPLE_DT:.0f} s, base x '
        f'{percentiles(base_x, (10, 50, 90))} mm; load '
        f'{percentiles(rows[:, 2])} N, {np.mean(rows[:, 2] > 100):.0%} > 100 N;'
        f' stalled {stalled_seconds(stalled):.1f} s; point '
        f'{np.mean(rows[:, 4]):.0%}, hydro {np.mean(rows[:, 5]):.0%}')
  for onset in flips(clean):
    before = samples[(samples[:, 0] >= onset - FLIP_LOOKBACK) &
                     (samples[:, 0] < onset)]
    goal = (int(samples[np.searchsorted(samples[:, 0], onset) - 1, 1])
            if onset > samples[0, 0] else None)
    if len(before):
      peak = np.argmax(before[:, 2])
      lines.append(f'  flip at {onset:.1f} s (goal {goal}): peak load '
                   f'{before[peak, 2]:.1f} N in the {FLIP_LOOKBACK:.0f} s '
                   f'before, {before[peak, 3] / max(before[peak, 2], 1e-9):.0%}'
                   f' at the apex')
    else:
      lines.append(f'  flip at {onset:.1f} s (goal {goal})')
  return '\n'.join(lines)


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--jobs', default=6, show_default=True)
def main(logs, jobs):
  with multiprocessing.Pool(min(jobs, len(logs))) as pool:
    for text in pool.map(score_log, logs):
      print(text)


if __name__ == '__main__':
  main()
