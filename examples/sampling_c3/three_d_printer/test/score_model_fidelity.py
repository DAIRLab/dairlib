#!/usr/bin/env python3
"""Do SC3's models predict what the cone really does, and which C3 bouts work?

Per goal of each compliant-sim log, two views:

  bouts   Every stretch in C3 mode, classed by where the EE started it
          relative to the true cone:  'low' is behind the base (body x below
          -4 mm) within 12 mm of the base centre's height, the push that moves
          a lying cone over a step edge; 'high' is behind the base above that;
          'other' is anywhere else.  Per class:  how many bouts, how many moved
          the cone at least --bout_mm toward -x (the goal-3/4 direction), and
          the median travel.  Also how the bouts ended (mode_switch_reason).
  loops   Every C3 loop's predicted progress toward the goal over the plan
          horizon -- from the C3 plan (C3_TRAJECTORY_OBJECT_CURR_PLAN) and from
          the cost rollout (DYNAMICALLY_FEASIBLE_CURR_PLAN) -- against the true
          cone's over the next --true_window seconds:  xy distance and
          tracked-axis misalignment, p50 / p90, and the correlation of each
          prediction with the truth.

Goal windows come from C3_FINAL_TARGET changes.  Reads OBJECT_STATE_SIMULATION_
CLEAN, so compliant-sim logs only; about a minute per log.

Usage:
  python3 score_model_fidelity.py ~/3d_printer/logs/2026/10_05_26/0000{07..12}
"""

import glob
import multiprocessing
import os.path as op
import sys
from collections import Counter

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
from score_finger_load_guard import (PRINTER_Z_TO_EE_CENTRE,  # noqa: E402
                                     decode_debug, find_log, rotation)

LOW_BODY_X = -0.004
LOW_DZ = 0.012
SAMPLE_DT = 0.05
REASONS = {1: 'to C3 cost', 2: 'to C3 reached', 3: 'repos cost',
           4: 'unproductive', 5: 'xbox', 6: 'jam', 7: 'goal changed'}


def axis(quat_wxyz):
  return rotation(np.asarray(quat_wxyz) / np.linalg.norm(quat_wxyz))[:, 0]


def read(folder):
  log = find_log(folder)
  clean, ee, debug, targets = [], [], [], []
  plans, rollouts = {}, {}
  t0 = None
  for event in EventLog(log, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      if not clean or t - clean[-1][0] >= SAMPLE_DT:
        clean.append((t, *dairlib.lcmt_object_state.decode(
            event.data).position[:7]))
    elif ch == 'PRINTER_STATE_SIMULATION':
      if not ee or t - ee[-1][0] >= SAMPLE_DT:
        msg = dairlib.lcmt_robot_output.decode(event.data)
        q = dict(zip(msg.position_names, msg.position))
        ee.append((t, q['x_axis_joint'], q['y_axis_joint'],
                   q['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE))
    elif ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      debug.append((t, msg.utime, msg.is_c3_mode, msg.mode_switch_reason))
    elif ch == 'C3_FINAL_TARGET':
      msg = dairlib.lcmt_c3_state.decode(event.data)
      targets.append((t, *msg.state[3:10]))
    elif ch in ('C3_TRAJECTORY_OBJECT_CURR_PLAN',
                'DYNAMICALLY_FEASIBLE_CURR_PLAN'):
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      blocks = {tr.trajectory_name: np.array(tr.datapoints)
                for tr in msg.saved_traj.trajectories}
      if ('object_position_target_0' in blocks and
          'object_orientation_target_0' in blocks):
        store = plans if ch.startswith('C3') else rollouts
        store[msg.utime] = (blocks['object_position_target_0'][:3, [0, -1]],
                            blocks['object_orientation_target_0'][:, [0, -1]])
  return (np.array(clean), np.array(ee), np.array(debug), np.array(targets),
          plans, rollouts)


def goal_windows(targets, end):
  """(start, end, goal position, goal axis) per distinct final target."""
  windows = []
  for row in targets:
    pos, quat = row[5:8], row[1:5]
    if not windows or np.linalg.norm(pos - windows[-1][2]) > 1e-6:
      if windows:
        windows[-1][1] = row[0]
      windows.append([row[0], end, pos, axis(quat)])
  return windows


def score(args):
  folder, bout_mm, true_window = args
  clean, ee, debug, targets, plans, rollouts = read(folder)
  end = clean[-1, 0]
  windows = goal_windows(targets, end)

  def at(series, t):
    return series[np.clip(np.searchsorted(series[:, 0], t), 0,
                          len(series) - 1)]

  def progress(p0, q0, p1, q1, goal_pos, goal_axis):
    d = (np.linalg.norm((p0 - goal_pos)[:2]) -
         np.linalg.norm((p1 - goal_pos)[:2]))
    mis = lambda q: np.degrees(np.arccos(np.clip(axis(q) @ goal_axis, -1, 1)))
    return 1e3 * d, mis(q0) - mis(q1)

  out = []
  for k, (a, b, goal_pos, goal_axis) in enumerate(windows):
    in_goal = (debug[:, 0] >= a) & (debug[:, 0] < b)
    rows = debug[in_goal]
    if len(rows) < 2:
      continue
    # Bouts.
    bouts, i = [], 0
    while i < len(rows):
      if rows[i, 2] != 1:
        i += 1
        continue
      j = i
      while j + 1 < len(rows) and rows[j + 1, 2] == 1:
        j += 1
      ta, tb = rows[i, 0], rows[j, 0]
      reason = int(rows[j + 1, 3]) if j + 1 < len(rows) else -1
      c0, e0 = at(clean, ta), at(ee, ta)
      body = rotation(c0[1:5] / np.linalg.norm(c0[1:5])).T @ (e0[1:4] -
                                                               c0[5:8])
      if body[0] < LOW_BODY_X:
        cls = 'low' if abs(e0[3] - c0[7]) < LOW_DZ else 'high'
      else:
        cls = 'other'
      dx = 1e3 * (at(clean, tb + 0.5)[5] - c0[5])
      bouts.append((cls, dx, tb - ta, reason))
      i = j + 1
    # Loops.
    loops = []
    for t, utime, is_c3, _ in rows:
      if not is_c3 or t > b - true_window or utime not in plans or \
         utime not in rollouts:
        continue
      (pp, pq), (rp, rq) = plans[utime], rollouts[utime]
      c0, c1 = at(clean, t), at(clean, t + true_window)
      loops.append((*progress(pp[:, 0], pq[:, 0], pp[:, 1], pq[:, 1],
                              goal_pos, goal_axis),
                    *progress(rp[:, 0], rq[:, 0], rp[:, 1], rq[:, 1],
                              goal_pos, goal_axis),
                    *progress(c0[5:8], c0[1:5], c1[5:8], c1[1:5], goal_pos,
                              goal_axis)))
    out.append((k, a, b, bouts, np.array(loops)))
  return folder, out


@click.command()
@click.argument('folders', nargs=-1, required=True)
@click.option('--bout_mm', default=5.0, show_default=True,
              help='Travel toward -x that counts as a bout that worked.')
@click.option('--true_window', default=1.25, show_default=True,
              help='Seconds of true motion compared with each prediction.')
@click.option('--goals', default='3,4', show_default=True,
              help='Goal indices to report.')
@click.option('--jobs', default=6, show_default=True)
def main(folders, bout_mm, true_window, goals, jobs):
  wanted = [int(g) for g in goals.split(',')]
  folders = [f for p in folders for f in sorted(glob.glob(p))]
  with multiprocessing.Pool(jobs) as pool:
    results = pool.map(score, [(f, bout_mm, true_window) for f in folders])
  pooled = {g: {'bouts': [], 'loops': []} for g in wanted}
  for folder, per_goal in results:
    for k, a, b, bouts, loops in per_goal:
      if k not in wanted:
        continue
      pooled[k]['bouts'] += bouts
      if len(loops):
        pooled[k]['loops'].append(loops)
      line = f'{op.basename(folder.rstrip("/"))} goal {k} ({b - a:4.0f} s):'
      for cls in ('low', 'high', 'other'):
        dx = np.array([d for c, d, _, _ in bouts if c == cls])
        if len(dx):
          line += (f' {cls} {np.mean(dx <= -bout_mm):.2f} of {len(dx)}'
                   f' (p50 {np.median(dx):+.1f} mm)')
      print(line)
  for k in wanted:
    bouts = pooled[k]['bouts']
    if not bouts:
      continue
    print(f'\n== goal {k}: {len(bouts)} C3 bouts, duration p50 '
          f'{np.median([d for _, _, d, _ in bouts]):.1f} s; ended by '
          + ', '.join(f'{REASONS.get(r, r)} {n}' for r, n in
                      Counter(r for *_, r in bouts).most_common()))
    for cls in ('low', 'high', 'other'):
      dx = np.array([d for c, d, _, _ in bouts if c == cls])
      if len(dx):
        print(f'   start {cls:5s} {len(dx):4d} bouts ({len(dx) / len(bouts):.0%})'
              f'  worked {np.mean(dx <= -bout_mm):.2f}  travel p50 '
              f'{np.median(dx):+.1f} mm')
    if pooled[k]['loops']:
      L = np.vstack(pooled[k]['loops'])
      f = lambda v: f'{np.median(v):+6.1f} / {np.percentile(v, 90):+6.1f}'
      corr = lambda v: np.corrcoef(v, L[:, 4])[0, 1]
      print(f'   {len(L)} C3 loops, progress p50 / p90 [mm]:  plan {f(L[:, 0])}'
            f'  rollout {f(L[:, 2])}  true {f(L[:, 4])};  corr with true:'
            f' plan {corr(L[:, 0]):+.2f}, rollout {corr(L[:, 2]):+.2f}')
      print(f'   axis [deg]:  plan {f(L[:, 1])}  rollout {f(L[:, 3])}  true '
            f'{f(L[:, 5])}')


if __name__ == '__main__':
  sys.exit(main())
