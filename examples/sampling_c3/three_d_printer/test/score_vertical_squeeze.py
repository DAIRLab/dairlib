"""Scores vertical squeezes: the EE pressing the cone down into the plate or
ramp (issue #13).

The printer's z axis is stiff and the finger gives only sideways, so an EE
target commanded into the top of a braced cone presses it hard; the cone then
pops out sideways or flips.  This lists every squeeze in compliant-sim logs and
rates them per run and per arm, and can replay the press latch
(HoldEEPlanAbovePressedObject in reposition.h) on logs recorded without it.

A squeeze is a stretch where the EE's downward force on the cone exceeds 30 N
(CONTACT_RESULTS, EE-cone point pairs and hydroelastic contacts, sampled at
20 Hz, gaps under 0.5 s merged).  Per squeeze:
  peak       peak downward force [N]; >= 80 N is the primary measure's tier.
  kind       sustained if the gantry was held more than 3 mm above its
             commanded height (the executed plan's knot 0) at some point in
             the squeeze, impact otherwise.
  mode       share of the squeeze's control loops in C3 (the rest
             repositioning), and the goal index.
  where      on the true cone at the peak: apex (top 20% of the axis), rim
             (bottom 20%), face; with the cone upright (axis z >= 0.9), lying
             (< 0.45) or tilted.
  bend       finger bend at the peak [mm].
  loss       a run loss (tally_cone_runs.py's onset) began between 1 s before
             the squeeze and 5 s after it.

Per run: squeezes >= 30 N and >= 80 N per minute, and the press latch's
stretches from sc3_stdout.txt: how many, how long, and the stalls, stretches
over 2 s during which the cone moved under 5 mm.

Per arm (run_sim_batch.py's --arm name, else w_G and ee_press_latch from the
log's yaml copies): rates with 90% run-bootstrap intervals.

--replay_press_latch, for logs recorded with the latch off: runs the latch on
each loop's executed plan (TRACKING_TRAJECTORY_ACTOR) against the estimated
cone (C3_ACTUAL), with the controller's cone mesh.  A squeeze stays uncovered
if, over the loops from 0.6 s before it to 0.3 s before its end (the printer's
command latency), the median of (gantry height 0.45 s later - lowest of knots
0-2) exceeds 0.5 mm, where the sim's z axis saturates, or 2 mm.  It also
reports how often the latch would raise the plan by more than 1 mm during good
toe tips (tally_cone_runs.py's tip+) and during goal-3 pushes that moved the
cone 5 mm or more up the slope.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_vertical_squeeze.py \\
      ~/3d_printer/logs/2026/10_02_26/0000{29..48} [--episodes] \\
      [--replay_press_latch] [--group name] [--jobs 6]
"""

import multiprocessing
import os.path as op
import re
import sys

import click
import numpy as np
import yaml
from lcm import EventLog
from scipy.optimize import nnls

sys.path.append(op.dirname(__file__))
import tally_cone_runs as T  # noqa: E402
from score_finger_load_guard import (PRINTER_Z_TO_EE_CENTRE,  # noqa: E402
                                     decode_debug, find_log, rotation)
from score_ramp_jams import ee_plan  # noqa: E402
import dairlib  # noqa: E402
import drake  # noqa: E402

SQUEEZE_FORCE = 30.0
PRIMARY_FORCE = 80.0
MERGE_GAP = 0.5
SAMPLE_DT = 0.05
SUSTAINED_MM = 3.0
LOSS_BEFORE, LOSS_AFTER = 1.0, 5.0
EE_RADIUS = 0.010
UPRIGHT_AXIS_Z, LYING_AXIS_Z = 0.9, 0.45
# The printer's command latency in the sim (sim_params.yaml actuator_delay plus
# about one command_time_constant).
LATENCY = 0.45
WINDOW_BEFORE, WINDOW_END = 0.6, 0.3
STALL_SECONDS, STALL_TRAVEL = 2.0, 0.005
# The latch's committed defaults (sampling_c3_options.h).
MIN_NORMAL_Z, RELEASE_GAP = 0.5, 0.005
CONE_OBJ = op.join(T.DAIRLIB_DIR, 'examples', 'sampling_c3', 'urdf', 'cone',
                   'cone.obj')


class HexCone:
  """The controller's cone collision mesh, a convex hexagonal pyramid: signed
  distance from a point and the outward gradient, in the body frame."""

  def __init__(self, path=CONE_OBJ):
    vertices, normals, faces = [], [], []
    for line in open(path):
      parts = line.split()
      if not parts:
        continue
      if parts[0] == 'v':
        vertices.append([float(x) for x in parts[1:4]])
      elif parts[0] == 'vn':
        normals.append([float(x) for x in parts[1:4]])
      elif parts[0] == 'f':
        faces.append(int(parts[1].split('//')[1]) - 1)
    self.vertices = np.array(vertices)
    self.normals = np.array([normals[i] for i in sorted(set(faces))])
    self.normals /= np.linalg.norm(self.normals, axis=1)[:, None]
    self.offsets = (self.normals @ self.vertices.T).max(axis=1)
    self.height = self.vertices[:, 0].max()
    self.base_radius = np.hypot(self.vertices[:, 1], self.vertices[:, 2]).max()
    self._hull = np.vstack([self.vertices.T, 1e3 * np.ones(len(vertices))])

  def query(self, p):
    s = self.normals @ p - self.offsets
    if s.max() <= 0:
      return s.max(), self.normals[s.argmax()]
    w, _ = nnls(self._hull, np.r_[p, 1e3])
    d = p - self.vertices.T @ w
    n = np.linalg.norm(d)
    return n, d / max(n, 1e-12)

  def gap(self, p_w, rot, pos):
    """EE-surface gap [m] and the world normal's z at an EE centre."""
    d, n = self.query(rot.T @ (p_w - pos))
    return d - EE_RADIUS, (rot @ n)[2]


CONE = HexCone()


def downward_force(msg):
  """The EE's downward force on the cone [N], and where it pushes hardest."""
  total, best, point = 0.0, 0.0, None
  pairs = [(c.body1_name, c.body2_name, np.array(c.contact_force),
            np.array(c.contact_point), 2) for c in msg.point_pair_contact_info]
  pairs += [(c.body1_name, c.body2_name, np.array(c.force_C_W),
             np.array(c.centroid_W), 1) for c in msg.hydroelastic_contacts]
  for b1, b2, f, p, acts_on in pairs:
    names = (b1, b2)
    if not (T.has(names, 'cone') and T.has(names, 'end_effector')):
      continue
    cone_is = 1 if 'cone' in b1 else 2
    down = -f[2] if cone_is == acts_on else f[2]
    total += down
    if down > best:
      best, point = down, p
  return total, point


def read_log(log):
  loops, clean, gantry, bend, force, clock = {}, [], [], [], [], []
  t0, last = None, -np.inf
  for event in EventLog(log, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'C3_ACTUAL':
      msg = dairlib.lcmt_c3_state.decode(event.data)
      s = np.array(msg.state)
      loops.setdefault(msg.utime, {})['state'] = (t, s[:3], s[3:7], s[7:10])
    elif ch == 'TRACKING_TRAJECTORY_ACTOR':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      knots = ee_plan(msg)
      if knots is not None:
        loops.setdefault(msg.utime, {})['plan'] = knots
    elif ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      loops.setdefault(msg.utime, {})['debug'] = (
          bool(msg.is_c3_mode), int(msg.detected_goal_changes))
      clock.append(t - msg.utime / 1e6)
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      clean.append((t, *dairlib.lcmt_object_state.decode(
          event.data).position[:7]))
    elif ch == 'PRINTER_STATE_SIMULATION':
      if gantry and t - gantry[-1][0] < 0.005:
        continue
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      gantry.append((t, q['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      p = dairlib.lcmt_robot_output.decode(event.data).position
      bend.append((t, np.hypot(p[0], p[1])))
    elif ch == 'CONTACT_RESULTS' and t - last >= SAMPLE_DT:
      last = t
      down, point = downward_force(
          drake.lcmt_contact_results_for_viz.decode(event.data))
      force.append((t, down, *(point if point is not None else [np.nan] * 3)))
  rows = sorted((v['state'][0], v) for v in loops.values()
                if all(k in v for k in ('state', 'plan', 'debug')))
  if not force or not clean or not bend:
    raise click.ClickException(
        f'{log}: needs CONTACT_RESULTS and the compliant sim\'s ground truth')
  # Log time less controller (sim) time, for the stdout lines' times.
  return (rows, np.array(clean), np.array(gantry), np.array(bend),
          np.array(force), np.median(clock))


def episodes(force):
  out, start, prev = [], None, None
  for i in np.flatnonzero(force[:, 1] > SQUEEZE_FORCE):
    if start is None or force[i, 0] - force[prev, 0] > MERGE_GAP:
      if start is not None:
        out.append((start, prev))
      start = i
    prev = i
  if start is not None:
    out.append((start, prev))
  return out


def where_on_cone(point, pose):
  rot, pos = rotation(pose[1:5] / np.linalg.norm(pose[1:5])), pose[5:8]
  p = rot.T @ (point - pos)
  axis_z = rot[2, 0]
  place = ('apex' if p[0] > 0.8 * CONE.height else
           'rim' if p[0] < 0.2 * CONE.height else 'face')
  posture = ('upright' if axis_z >= UPRIGHT_AXIS_Z else
             'lying' if abs(axis_z) < LYING_AXIS_Z else 'tilted')
  return f'{place}/{posture}'


def at(series, t):
  return series[np.clip(np.searchsorted(series[:, 0], t), 0,
                        len(series) - 1)]


def replay_latch(rows, gantry):
  """Per loop: (t, C3, goal, press, latched press, lift)."""
  engaged, floor = False, -np.inf
  out = []
  for t, v in rows:
    _, _, quat, pos = v['state']
    knots = v['plan']
    rot = rotation(quat / np.linalg.norm(quat))
    gap, nz = CONE.gap(knots[0], rot, pos)
    if engaged and gap >= RELEASE_GAP:
      engaged = False
    elif not engaged and gap < 0 and nz >= MIN_NORMAL_Z:
      engaged, floor = True, knots[0, 2]
    z = knots[:, 2]
    latched = np.maximum(z, max(floor, knots[0, 2])) if engaged else z
    if engaged:
      floor = max(floor, knots[0, 2])
    held = at(gantry, t + LATENCY)[1]
    c3, goal = v['debug']
    out.append((t, c3, goal, held - z[:3].min(), held - latched[:3].min(),
                (latched - z).max()))
  return np.array(out)


def score(path, replay):
  log = find_log(path)
  folder = op.dirname(log)
  rows, clean, gantry, bend, force, offset = read_log(log)
  tally = T.score_log(path)
  loss = tally['loss']
  onset = loss['onset'] if loss and loss['real'] else None
  squeezes = []
  for a, b in episodes(force):
    ta, tb = force[a, 0], force[b, 0]
    peak = a + int(np.argmax(force[a:b + 1, 1]))
    in_loops = [v for t, v in rows if ta - SAMPLE_DT <= t <= tb + SAMPLE_DT]
    above = [at(gantry, t)[1] - v['plan'][0, 2]
             for t, v in rows if ta - SAMPLE_DT <= t <= tb + SAMPLE_DT]
    squeezes.append(dict(
        start=ta, dur=tb - ta + SAMPLE_DT, peak=force[peak, 1],
        sustained=bool(above) and 1e3 * max(above) > SUSTAINED_MM,
        c3=np.mean([v['debug'][0] for v in in_loops]) if in_loops else np.nan,
        goal=(int(np.median([v['debug'][1] for v in in_loops]))
              if in_loops else -1),
        where=(where_on_cone(force[peak, 2:5], at(clean, force[peak, 0]))
               if np.isfinite(force[peak, 2]) else '?'),
        bend=1e3 * at(bend, force[peak, 0])[1],
        loss=onset is not None and ta - LOSS_BEFORE <= onset <= tb +
        LOSS_AFTER))
  minutes = (force[-1, 0] - force[0, 0]) / 60.0
  stretches = []
  stdout = op.join(folder, 'sc3_stdout.txt')
  if op.exists(stdout):
    start = None
    for line in open(stdout, errors='replace'):
      m = re.match(r'\[press latch\] t=([0-9.eE+-]+) \S+ plan '
                   r'(engaged|released)', line)
      if not m:
        continue
      tc = float(m.group(1)) + offset
      if m.group(2) == 'engaged':
        start = tc
      elif start is not None:
        moved = np.linalg.norm(at(clean, tc)[5:8] - at(clean, start)[5:8])
        stretches.append(dict(start=start, dur=tc - start,
                              stall=tc - start > STALL_SECONDS and
                              moved < STALL_TRAVEL))
        start = None
  result = dict(log=log, name=op.basename(folder),
                date=op.basename(op.dirname(folder)), arm=arm_of(folder),
                minutes=minutes, squeezes=squeezes, stretches=stretches,
                loss=loss, reason=tally['reason'])
  if replay:
    loops = replay_latch(rows, gantry)
    for s in squeezes:
      w = loops[(loops[:, 0] >= s['start'] - WINDOW_BEFORE) &
                (loops[:, 0] <= max(s['start'] + s['dur'] - WINDOW_END,
                                    s['start']))]
      s['press'] = 1e3 * np.median(w[:, 3]) if len(w) else np.nan
      s['latched'] = 1e3 * np.median(w[:, 4]) if len(w) else np.nan

    def lifts(a, b):
      w = loops[(loops[:, 0] >= a - WINDOW_BEFORE) & (loops[:, 0] <= b) &
                (loops[:, 1] > 0)]
      return 1e3 * w[:, 5]

    result['tips'] = [lifts(p['start'], p['start'] + p['dur'])
                      for p in tally['pushes'] if p['outcome'] == 'tip+']
    slope = []
    for a, b in T.episodes(force[:, 0], np.abs(force[:, 1]) > 0.2, 0.4):
      goals = [v['debug'][1] for t, v in rows if a <= t <= b]
      if not goals or np.median(goals) != T.TOE_GOAL + 1:
        continue
      if at(clean, a - 0.2)[5] - at(clean, b + 1.5)[5] >= 0.005:
        slope.append(lifts(a, b))
    result['slope'] = slope
  return result


def arm_of(folder):
  record = op.join(folder, 'run_record.yaml')
  name = (yaml.safe_load(open(record)) or {}).get('arm') if op.exists(
      record) else None
  options = T.load_copy(folder, 'sampling_c3_params')
  latch = (f'latch nz={options.get("ee_press_latch_min_normal_z", MIN_NORMAL_Z)}'
           if options.get('ee_press_latch') else 'no latch')
  return name or f'w_G={options.get("w_G")} {latch}'


def rate(runs, threshold):
  minutes = sum(r['minutes'] for r in runs)
  n = sum(s['peak'] >= threshold for r in runs for s in r['squeezes'])
  return n / max(minutes, 1e-9)


def print_run(r, show):
  sq = r['squeezes']
  n30 = len(sq)
  n80 = sum(s['peak'] >= PRIMARY_FORCE for s in sq)
  lost = sum(s['loss'] for s in sq)
  st = r['stretches']
  print(f'{r["date"]}/{r["name"]}  {r["arm"]} | {r["reason"]} | '
        f'{r["minutes"]:.1f} min | squeezes >=30 N {n30} '
        f'({n30 / r["minutes"]:.2f}/min), >=80 N {n80} '
        f'({n80 / r["minutes"]:.2f}/min), before a loss {lost} | latch '
        f'stretches {len(st)}, longest '
        f'{max([s["dur"] for s in st], default=0):.1f} s, stalls '
        f'{sum(s["stall"] for s in st)}')
  if show:
    for s in sq:
      extra = (f' | replay press {s["press"]:+5.1f} -> {s["latched"]:+5.1f} mm'
               if 'press' in s else '')
      print(f'    {s["start"]:6.1f} s {s["dur"]:4.1f} s peak {s["peak"]:4.0f} N'
            f' {"sustained" if s["sustained"] else "impact   "} goal '
            f'{s["goal"]} C3 {s["c3"]:.2f} {s["where"]:13s} bend '
            f'{s["bend"]:4.0f} mm{"  LOSS" if s["loss"] else ""}{extra}')


def summarize(runs, label):
  minutes = sum(r['minutes'] for r in runs)
  sq = [s for r in runs for s in r['squeezes']]
  print(f'\n== {label}: {len(runs)} runs, {minutes:.0f} min')
  for threshold in (SQUEEZE_FORCE, PRIMARY_FORCE):
    lo, hi = T.bootstrap(runs, lambda rs: rate(rs, threshold))
    tier = [s for s in sq if s['peak'] >= threshold]
    print(f'   >= {threshold:.0f} N: {rate(runs, threshold):.2f} per min '
          f'[{lo:.2f}, {hi:.2f}]; sustained {sum(s["sustained"] for s in tier)}'
          f' of {len(tier)}; before a loss {sum(s["loss"] for s in tier)}')
  places = {}
  for s in sq:
    if s['peak'] >= PRIMARY_FORCE:
      places[s['where']] = places.get(s['where'], 0) + 1
  print('   >= 80 N by place: ' + ', '.join(
      f'{k} {v}' for k, v in sorted(places.items(), key=lambda kv: -kv[1])))
  st = [s for r in runs for s in r['stretches']]
  if st:
    print(f'   latch stretches {len(st)} ({len(st) / minutes:.2f}/min), '
          f'median {np.median([s["dur"] for s in st]):.1f} s, stalls '
          f'{sum(s["stall"] for s in st)}')
  if runs and 'tips' in runs[0]:
    for threshold in (SQUEEZE_FORCE, PRIMARY_FORCE):
      tier = [s for s in sq if s['peak'] >= threshold and
              np.isfinite(s['press'])]
      if not tier:
        continue
      fmt = ' / '.join(
          f'{np.mean([s[k] > mm for s in tier]):.2f}'
          for k in ('press', 'latched') for mm in (0.5, 2.0))
      print(f'   replay, >= {threshold:.0f} N ({len(tier)}): uncovered at '
            f'0.5 / 2 mm, unlatched then latched: {fmt}')
    for name, key in (('good toe tips', 'tips'), ('goal-3 slope pushes',
                                                  'slope')):
      lifts = [w for r in runs for w in r[key] if len(w)]
      if not lifts:
        continue
      allw = np.concatenate(lifts)
      raised = allw[allw > 1.0]
      print(f'   replay, {name} ({len(lifts)}): loops raised > 1 mm '
            f'{np.mean(allw > 1.0):.2f} (median lift '
            f'{np.median(raised) if len(raised) else 0:.1f} mm), pushes '
            f'touched {np.mean([np.any(w > 1.0) for w in lifts]):.2f}')


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--jobs', default=6, show_default=True)
@click.option('--episodes', 'show', is_flag=True, help='List every squeeze.')
@click.option('--replay_press_latch', is_flag=True,
              help='Replay the press latch on logs recorded without it.')
def main(logs, jobs, show, replay_press_latch):
  with multiprocessing.Pool(min(jobs, len(logs))) as pool:
    runs = pool.starmap(score, [(p, replay_press_latch) for p in logs])
  for r in runs:
    print_run(r, show)
  groups = {}
  for r in runs:
    groups.setdefault(r['arm'], []).append(r)
  for label, rs in groups.items():
    summarize(rs, label)


if __name__ == '__main__':
  main()
