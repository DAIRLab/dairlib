"""Tallies closed-loop cone demo sim runs per goal, per toe push and per arm.

Roadmap A (re-baseline, then settle w_G and the cost budget) needs outcomes
that a handful of runs can resolve.  Whole-run outcomes can't: with 10 runs an
arm, a toe-loss rate of 60% against 20% is detected only about a quarter of the
time.  So besides the run outcomes this scores every push the finger makes on
the cone during the toe phase (goal 2), and rates per goal-2 minute, with
confidence intervals bootstrapped over runs.

Per run (compliant-sim logs; the stop reason comes from run_sim_batch.py's
run_record.yaml when there is one):
  arm        run_sim_batch.py's --arm name (group with --group name), and,
             read from the log folder's yaml copies: w_G, the cost-LCS budget
             entry, whether unload-before-lift is on, the sim's cone model, and
             the ramp's contact model as CONTACT_RESULTS shows it (point pairs
             or hydroelastic surfaces between the cone and ramp_link).
  goals      when each goal became the target (detected_goal_changes).
  loss       for a run stopped tipped / flipped / out: the goal and the time
             the cone entered, for good, the pose the monitor stops on (its
             rules, replayed on the log's own clock: run_record.yaml's stop time
             counts from the logger's start, a few seconds earlier), who was in
             control over the 3 s before
             (C3 share, latched share), the last push on the cone that ended
             within 5 s before, and whether unload-before-lift fired in the 10 s
             before ([repos unload] in sc3_stdout.txt).
  passive    the share of goal-2 C3 loops whose executed EE plan moves under
             10 mm (score_c3_plan_activity.py's measure of C3 buying progress
             with phantom environment forces).
  loop       control-loop period p50 / p90 from C3_ACTUAL arrival times.
  goal 4     closest approach to the final goal and the error at the end.

Per push: an episode of fingertip-or-shaft contact with the cone
(CONTACT_RESULTS, > 0.2 N, gaps under 0.4 s merged) while goal 2 is the target,
kept if the finger bent >= 5 mm or the cone moved >= 5 mm or tilted >= 10 deg
by 1.5 s after it.  Reported: the gantry's height above the apex at the start,
the push direction (degrees from -x, toward +y positive), the peak bend, the
largest bend before the cone first tilted 18 deg from upright, how far the base
slid before that, the C3 share, and the outcome against the goal-2 target,
comparing 0.2 s before with 1.5 s after:
  upright cone (axis z >= 0.9 before)
    tip+    tipped (axis z < 0.6 after) with the axis within 60 deg of goal 2's
    tip-    tipped any other way: sideways, backward, onto the plate
    shove   still upright, 5 mm or more further from the goal
    adv     still upright, 5 mm or more closer
    none    otherwise
  cone already tilted
    better  5 mm closer or 10 deg better aligned, and neither worse
    worse   5 mm further or 10 deg worse aligned, and neither better
    mixed   otherwise

Per arm (or per --group key): toe goal met, mid-slope reached, seated; losses
in goal 2 per goal-2 minute; the Kaplan-Meier median time to the toe goal
(losses and caps censored); upright-push outcome shares and tip+ / bad (tip- +
shove) rates per goal-2 minute with 90% run-bootstrap intervals; passive share;
loop period; and, between the first two groups, Fisher's exact p on toe met and
toe lost.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/tally_cone_runs.py \\
      ~/3d_printer/logs/2026/10_02_26/0000{14..16} \\
      ~/3d_printer/logs/2026/10_02_26/0000{20..28} [--pushes] [--group ramp]
"""

import glob
import multiprocessing
import os.path as op
import re
import sys

import click
import numpy as np
import yaml
from lcm import EventLog
from scipy.stats import fisher_exact

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'external', 'drake+',
                        'lcmtypes'))
sys.path.append(op.dirname(__file__))
import dairlib  # noqa: E402
import drake  # noqa: E402
from score_finger_load_guard import (CONE_HEIGHT, PRINTER_Z_TO_EE_CENTRE,  # noqa: E402,E501
                                     decode_debug, find_log, rotation)

CONTACT_DT = 0.05
CONTACT_FORCE = 0.2
CONTACT_GAP = 0.4
PUSH_BEND = 0.005
PUSH_TRAVEL = 0.005
PUSH_TILT = 10.0
AFTER = 1.5
TIP_AXIS_Z = 0.6
UPRIGHT_AXIS_Z = 0.9
TIP_GOOD_DEG = 60.0
SHOVE = 0.005
BETTER_DEG = 10.0
PASSIVE_MM = 10.0
WORKSPACE_XY = (0.0, 0.35)
LOST = ('tipped', 'flipped', 'out', 'exited')
TOE_GOAL = 2
BAD = ('tip-', 'shove')


def axis(quat_wxyz):
  return rotation(np.asarray(quat_wxyz) / np.linalg.norm(quat_wxyz))[:, 0]


def angle_deg(a, b):
  return np.degrees(np.arccos(np.clip(a @ b, -1.0, 1.0)))


def load_copy(folder, prefix):
  found = sorted(glob.glob(op.join(folder, f'{prefix}_[0-9]*.yaml')))
  return yaml.safe_load(open(found[0])) if found else {}


def arm_of(folder):
  options = load_copy(folder, 'sampling_c3_params')
  progress = load_copy(folder, 'progress_params')
  sim = load_copy(folder, 'sim_params')
  guard = progress.get('jam_guard') or {}
  models = sim.get('object_models') or ['?']
  return dict(
      w_G=options.get('w_G'), w_G_position=options.get('w_G_position'),
      budget=options.get('num_contacts_index_for_cost'),
      unload='repos_unload_load' in guard,
      cone=op.splitext(op.basename(models[0]))[0],
      bed_hydro=bool(sim.get('hydroelastic_bed', False)))


def goals_of(folder):
  goals = load_copy(folder, 'goal_params')
  position = np.array([g[0] for g in goals['fixed_target_position_sequence']])
  quats = goals.get('fixed_target_orientation_sequence') or []
  axes = [axis(np.array(q[0], dtype=float)) for q in quats]
  return position, axes


def has(names, key):
  return any(key in n for n in names)


def read_log(log):
  rows, actual, clean, ee, defl, contacts = [], [], [], [], [], []
  plan_disp = np.nan
  t0, last_contact = None, -np.inf
  for event in EventLog(log, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      rows.append((t, msg.utime / 1e6, msg.is_c3_mode,
                   msg.detected_goal_changes, msg.jam_tripped, plan_disp))
    elif ch == 'C3_ACTUAL':
      actual.append(t)
    elif ch == 'C3_EXECUTION_TRAJECTORY_ACTOR':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      for traj in msg.saved_traj.trajectories:
        if traj.trajectory_name == 'end_effector_position_target':
          p = np.array(traj.datapoints)[:3]
          plan_disp = np.linalg.norm(p[:, -1] - p[:, 0])
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      clean.append((t, *dairlib.lcmt_object_state.decode(
          event.data).position[:7]))
    elif ch == 'PRINTER_STATE_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      ee.append((t, q['x_axis_joint'], q['y_axis_joint'],
                 q['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      p = dairlib.lcmt_robot_output.decode(event.data).position
      defl.append((t, np.hypot(p[0], p[1])))
    elif ch == 'CONTACT_RESULTS' and t - last_contact >= CONTACT_DT:
      last_contact = t
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      ee_cone = 0.0
      ramp_point = ramp_hydro = plate = False
      for c in msg.point_pair_contact_info:
        names = (c.body1_name, c.body2_name)
        if not has(names, 'cone'):
          continue
        if has(names, 'end_effector'):
          ee_cone += np.linalg.norm(c.contact_force)
        ramp_point |= has(names, 'ramp_link')
        plate |= has(names, 'build_plate')
      for c in msg.hydroelastic_contacts:
        names = (c.body1_name, c.body2_name)
        if not has(names, 'cone'):
          continue
        if has(names, 'end_effector'):
          ee_cone += np.linalg.norm(c.force_C_W)
        ramp_hydro |= has(names, 'ramp_link')
        plate |= has(names, 'build_plate')
      contacts.append((t, ee_cone, ramp_point, ramp_hydro, plate))
  if not rows or not clean or not defl or not contacts:
    raise click.ClickException(
        f'{log}: needs SAMPLING_C3_DEBUG and the compliant sim\'s ground truth')
  return (np.array(rows, dtype=float), np.array(actual), np.array(clean),
          np.array(ee), np.array(defl), np.array(contacts, dtype=float))


def interp_rows(series, t):
  idx = np.clip(np.searchsorted(series[:, 0], t), 0, len(series) - 1)
  return series[idx]


def episodes(times, active, gap):
  out, start, last = [], None, None
  for t, a in zip(times, active):
    if a:
      if start is None or t - last > gap:
        if start is not None:
          out.append((start, last))
        start = t
      last = t
  if start is not None:
    out.append((start, last))
  return out


def stdout_unloads(folder, rows):
  path = op.join(folder, 'sc3_stdout.txt')
  if not op.exists(path):
    return []
  offset = np.median(rows[:, 0] - rows[:, 1])
  out = []
  for line in open(path, errors='replace'):
    m = re.match(r'\[repos unload\] t=([0-9.]+)', line)
    if m:
      out.append(float(m.group(1)) + offset)
  return out


def loss_onset(reason, clean, contacts, end):
  """Start of the final stretch in run_sim_batch.py's stop pose, and whether
  the run ended in it.  Before the monitor read hydroelastic contacts it cut
  runs as tipped whose cone still touched the ramp (rim on the plate, apex on
  the ramp): those end outside the pose and are not losses."""
  if reason == 'exited':
    return end, True
  t = contacts[:, 0]
  pose = interp_rows(clean, t)
  axes = np.array([axis(p[1:5]) for p in pose])
  if reason == 'tipped':
    cond = ((axes[:, 2] < 0.5) & (contacts[:, 4] > 0) & (contacts[:, 2] == 0) &
            (contacts[:, 3] == 0))
  elif reason == 'flipped':
    cond = (axes[:, 0] > 0.3) & (axes[:, 2] < -0.7)
  else:
    xy = pose[:, 5:7]
    cond = ((xy < WORKSPACE_XY[0]) | (xy > WORKSPACE_XY[1])).any(axis=1) | (
        pose[:, 7] < -0.02)
  if not cond[-1]:
    return end, False
  clear = np.flatnonzero(~cond)
  return (t[clear[-1] + 1] if len(clear) else t[0]), True


def classify(before, after, goal_pos, goal_axis):
  ax0, ax1 = axis(before[1:5]), axis(after[1:5])
  pos0 = np.linalg.norm(before[5:8] - goal_pos)
  pos1 = np.linalg.norm(after[5:8] - goal_pos)
  if ax0[2] >= UPRIGHT_AXIS_Z:
    if ax1[2] < TIP_AXIS_Z:
      return 'tip+' if angle_deg(ax1, goal_axis) <= TIP_GOOD_DEG else 'tip-'
    if pos1 - pos0 >= SHOVE:
      return 'shove'
    return 'adv' if pos0 - pos1 >= SHOVE else 'none'
  d_ang = angle_deg(ax1, goal_axis) - angle_deg(ax0, goal_axis)
  better = pos0 - pos1 >= SHOVE or d_ang <= -BETTER_DEG
  worse = pos1 - pos0 >= SHOVE or d_ang >= BETTER_DEG
  return 'better' if better and not worse else (
      'worse' if worse and not better else 'mixed')


def score_log(path):
  log = find_log(path)
  folder = op.dirname(log)
  rows, actual, clean, ee, defl, contacts = read_log(log)
  record_path = op.join(folder, 'run_record.yaml')
  record = yaml.safe_load(open(record_path)) if op.exists(record_path) else {}
  goal_pos, goal_axes = goals_of(folder)
  arm = arm_of(folder)
  arm['name'] = record.get('arm')  # run_sim_batch.py --arm
  ramp = contacts[:, 2].sum(), contacts[:, 3].sum()
  # Read from the contacts: a run lost before the ramp (10_02_26/000032) has
  # none to read.
  arm['ramp'] = ('none' if not any(ramp) else
                 'hydro' if ramp[1] > ramp[0] else 'point')

  t, goal = rows[:, 0], rows[:, 3].astype(int)
  end = t[-1]
  goal_times = {g: t[np.argmax(goal >= g)] for g in range(1, goal.max() + 1)}
  t2 = goal_times.get(TOE_GOAL)
  t3 = goal_times.get(TOE_GOAL + 1)
  reason = record.get('reason', '?')

  def goal_at(time):
    return int(goal[min(np.searchsorted(t, time), len(t) - 1)])

  # Pushes on the cone while goal 2 is the target.
  pushes = []
  if t2 is not None:
    toe_end = t3 if t3 is not None else end
    for ts, te in episodes(contacts[:, 0], contacts[:, 1] > CONTACT_FORCE,
                           CONTACT_GAP):
      if not t2 <= ts < toe_end:
        continue
      before = interp_rows(clean, ts - 0.2)
      after = interp_rows(clean, te + AFTER)
      window = clean[(clean[:, 0] >= ts) & (clean[:, 0] <= te + AFTER)]
      bend_w = defl[(defl[:, 0] >= ts) & (defl[:, 0] <= te)]
      peak = bend_w[:, 1].max() if len(bend_w) else 0.0
      travel = np.linalg.norm(window[:, 5:8] - before[5:8], axis=1).max()
      tilt = max(angle_deg(axis(w[1:5]), axis(before[1:5])) for w in window)
      if peak < PUSH_BEND and travel < PUSH_TRAVEL and tilt < PUSH_TILT:
        continue
      ax0 = axis(before[1:5])
      apex_z = before[7] + CONE_HEIGHT * ax0[2]
      ee0, ee1 = interp_rows(ee, ts), interp_rows(ee, te)
      d = ee1[1:3] - ee0[1:3]
      direction = (np.degrees(np.arctan2(d[1], -d[0]))
                   if np.linalg.norm(d) > 1e-3 else np.nan)
      vertical = np.array([angle_deg(axis(w[1:5]), np.array([0, 0, 1.0]))
                           for w in window])
      tilted = np.flatnonzero(vertical >= 18.0)
      t18 = window[tilted[0], 0] if len(tilted) else np.nan
      if np.isfinite(t18):
        pre = defl[(defl[:, 0] >= ts) & (defl[:, 0] <= t18), 1]
        bend18 = pre.max() if len(pre) else 0.0
        slide18 = np.linalg.norm(interp_rows(clean, t18)[5:8] - before[5:8])
      else:
        bend18 = slide18 = np.nan
      in_push = (t >= ts) & (t <= te)
      c3_share = rows[in_push, 2].mean() if in_push.any() else np.nan
      pushes.append(dict(
          start=ts, dur=te - ts, upright=ax0[2] >= UPRIGHT_AXIS_Z,
          above_apex=1e3 * (ee0[3] - apex_z), direction=direction,
          peak=1e3 * peak, bend18=1e3 * bend18, slide18=1e3 * slide18,
          c3=c3_share, outcome=classify(before, after, goal_pos[TOE_GOAL],
                                        goal_axes[TOE_GOAL])))

  # The loss.
  loss = None
  if reason in LOST:
    onset, real = loss_onset(reason, clean, contacts, end)
    window = (t >= onset - 3.0) & (t <= onset)
    last_push = [p for p in pushes if onset - 5.0 <= p['start'] + p['dur']
                 <= onset + 0.5]
    unloads = [u for u in stdout_unloads(folder, rows)
               if onset - 10.0 <= u <= onset]
    loss = dict(reason=reason, real=real, onset=onset, goal=goal_at(onset),
                since_goal=onset - goal_times.get(goal_at(onset), 0.0),
                c3=rows[window, 2].mean() if window.any() else np.nan,
                latched=rows[window, 4].mean() if window.any() else np.nan,
                push=last_push[-1]['outcome'] if last_push else None,
                unload=len(unloads))

  in_toe = (goal == TOE_GOAL) & (rows[:, 2] > 0) & np.isfinite(rows[:, 5])
  passive = (np.mean(1e3 * rows[in_toe, 5] < PASSIVE_MM)
             if in_toe.any() else np.nan)
  period = np.diff(actual) * 1e3
  goal4 = None
  last = len(goal_pos) - 1
  if last in goal_times:
    stretch = clean[clean[:, 0] >= goal_times[last]]
    dist = 1e3 * np.linalg.norm(stretch[:, 5:8] - goal_pos[last], axis=1)
    ang = np.array([angle_deg(axis(r[1:5]), goal_axes[last]) for r in stretch])
    i = int(np.argmin(dist))
    goal4 = dict(min_dist=dist[i], ang_at_min=ang[i], end_dist=dist[-1],
                 end_ang=ang[-1], seconds=end - goal_times[last])
  toe_minutes = ((min(t3, end) if t3 is not None else end) - t2) / 60.0 \
      if t2 is not None else 0.0
  return dict(log=log, name=op.basename(folder), date=op.basename(
      op.dirname(folder)), arm=arm, reason=reason, end=end,
      goal_times=goal_times, furthest=int(goal.max()), loss=loss,
      pushes=pushes, passive=passive, toe_minutes=toe_minutes,
      period=(np.percentile(period, 50), np.percentile(period, 90)),
      goal4=goal4, n_goals=len(goal_pos))


def arm_label(arm, keys):
  parts = []
  for k in keys:
    v = arm.get(k)
    parts.append(f'{k}={v:g}' if isinstance(v, float) else f'{k}={v}')
  return ' '.join(parts)


def kaplan_meier_median(durations, events):
  order = np.argsort(durations)
  d, e = np.asarray(durations)[order], np.asarray(events)[order]
  survival, at_risk = 1.0, len(d)
  for di, ei in zip(d, e):
    if ei:
      survival *= 1 - 1 / at_risk
      if survival <= 0.5:
        return di, True
    at_risk -= 1
  return (d.max() if len(d) else np.nan), False


def bootstrap(runs, stat, n=2000, seed=0):
  rng = np.random.default_rng(seed)
  values = []
  for _ in range(n):
    pick = [runs[i] for i in rng.integers(0, len(runs), len(runs))]
    v = stat(pick)
    if np.isfinite(v):
      values.append(v)
  return np.percentile(values, [5, 95]) if values else (np.nan, np.nan)


def toe_rates(runs):
  minutes = sum(r['toe_minutes'] for r in runs)
  up = [p for r in runs for p in r['pushes'] if p['upright']]
  good = sum(p['outcome'] == 'tip+' for p in up)
  bad = sum(p['outcome'] in BAD for p in up)
  return minutes, len(up), good, bad


def summarize(runs, label):
  n = len(runs)
  toe = sum(r['furthest'] > TOE_GOAL for r in runs)
  mid = sum(r['furthest'] > TOE_GOAL + 1 for r in runs)
  seated = sum(r['reason'] == 'success' for r in runs)
  reached_toe = [r for r in runs if TOE_GOAL in r['goal_times']]
  toe_lost = sum(1 for r in reached_toe if r['loss'] and r['loss']['real']
                 and r['loss']['goal'] == TOE_GOAL)
  cut = sum(1 for r in runs if r['loss'] and not r['loss']['real'])
  durations, events = [], []
  for r in reached_toe:
    start = r['goal_times'][TOE_GOAL]
    met = r['goal_times'].get(TOE_GOAL + 1)
    durations.append((met if met is not None else r['end']) - start)
    events.append(met is not None)
  median, reached = kaplan_meier_median(durations, events)
  minutes, n_up, good, bad = toe_rates(runs)
  print(f'\n== {label}: {n} runs')
  print(f'   toe goal met {toe}/{n}, mid-slope reached {mid}/{n}, seated '
        f'{seated}/{n}; lost in goal 2: {toe_lost} of {len(reached_toe)} '
        f'({toe_lost / max(minutes, 1e-9):.2f} per goal-2 min over '
        f'{minutes:.1f} min)' + (f'; {cut} cut by the monitor on the ramp, '
                                  'counted as censored' if cut else ''))
  print(f'   time to toe goal, Kaplan-Meier median: '
        + (f'{median:.0f} s' if reached else f'> {median:.0f} s'))
  outcomes = {}
  for r in runs:
    for p in r['pushes']:
      if p['upright']:
        outcomes[p['outcome']] = outcomes.get(p['outcome'], 0) + 1
  shares = ', '.join(f'{k} {v}' for k, v in sorted(outcomes.items()))
  lo, hi = bootstrap(runs, lambda rs: toe_rates(rs)[2] / max(
      toe_rates(rs)[0], 1e-9))
  blo, bhi = bootstrap(runs, lambda rs: toe_rates(rs)[3] / max(
      toe_rates(rs)[0], 1e-9))
  slo, shi = bootstrap(runs, lambda rs: toe_rates(rs)[3] / toe_rates(rs)[1]
                       if toe_rates(rs)[1] else np.nan)
  print(f'   upright toe pushes {n_up}: {shares}')
  print(f'   tip+ per goal-2 min {good / max(minutes, 1e-9):.2f} '
        f'[{lo:.2f}, {hi:.2f}]; bad (tip- + shove) per min '
        f'{bad / max(minutes, 1e-9):.2f} [{blo:.2f}, {bhi:.2f}]; bad share '
        f'{bad / max(n_up, 1):.2f} [{slo:.2f}, {shi:.2f}]')
  passive = [r['passive'] for r in runs if np.isfinite(r['passive'])]
  print(f'   goal-2 passive C3 plans, median over runs '
        f'{np.median(passive) if passive else np.nan:.2f}; loop p50 / p90 '
        f'{np.median([r["period"][0] for r in runs]):.0f} / '
        f'{np.median([r["period"][1] for r in runs]):.0f} ms')
  return dict(n=n, toe=toe, reached_toe=len(reached_toe), toe_lost=toe_lost)


def print_run(r, show_pushes):
  arm = r['arm']
  goals = ' '.join(f'{g}@{tt:.0f}' for g, tt in sorted(r['goal_times'].items()))
  print(f'{r["date"]}/{r["name"]}  wG {arm["w_G"]} budget {arm["budget"]} '
        f'unload {"on" if arm["unload"] else "off"} ramp {arm["ramp"]} '
        f'cone {arm["cone"]}  | goals {goals} | {r["reason"]} at '
        f'{r["end"]:.0f} s | loop {r["period"][0]:.0f}/{r["period"][1]:.0f} '
        f'ms | passive {r["passive"]:.2f}')
  if r['loss']:
    l = r['loss']
    if not l['real']:
      print(f'    NOT A LOSS: stopped {l["reason"]} with the cone still on the '
            'ramp (the monitor read point-pair contacts only)')
    print(f'    loss: goal {l["goal"]} ({l["since_goal"]:.0f} s in), onset '
          f'{l["onset"]:.1f} s, C3 {l["c3"]:.2f}, latched {l["latched"]:.2f},'
          f' last push {l["push"]}, unload-before-lift in last 10 s '
          f'{l["unload"]}')
  if r['goal4']:
    g = r['goal4']
    print(f'    final goal: {g["seconds"]:.0f} s; closest {g["min_dist"]:.0f} '
          f'mm (axis {g["ang_at_min"]:.0f} deg); at the end {g["end_dist"]:.0f}'
          f' mm (axis {g["end_ang"]:.0f} deg)')
  if show_pushes:
    for p in r['pushes']:
      print(f'    push {p["start"]:6.1f} s {p["dur"]:4.1f} s '
            f'{"upright" if p["upright"] else "tilted "} above apex '
            f'{p["above_apex"]:+5.0f} mm dir {p["direction"]:+5.0f} deg peak '
            f'{p["peak"]:4.1f} mm bend<18deg {p["bend18"]:5.1f} slide<18deg '
            f'{p["slide18"]:5.1f} mm C3 {p["c3"]:.2f}  {p["outcome"]}')


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--jobs', default=6, show_default=True)
@click.option('--pushes', is_flag=True, help='List every goal-2 push.')
@click.option('--group', default='w_G,budget,unload,ramp,cone,bed_hydro',
              show_default=True,
              help='Arm keys that define a group, comma separated.')
def main(logs, jobs, pushes, group):
  with multiprocessing.Pool(min(jobs, len(logs))) as pool:
    runs = pool.map(score_log, logs)
  for r in runs:
    print_run(r, pushes)
  keys = group.split(',')
  groups = {}
  for r in runs:
    groups.setdefault(arm_label(r['arm'], keys), []).append(r)
  summaries = [(label, summarize(rs, label)) for label, rs in groups.items()]
  if len(summaries) >= 2:
    (la, a), (lb, b) = summaries[:2]
    met = fisher_exact([[a['toe'], a['n'] - a['toe']],
                        [b['toe'], b['n'] - b['toe']]])[1]
    lost = fisher_exact([[a['toe_lost'], a['reached_toe'] - a['toe_lost']],
                         [b['toe_lost'], b['reached_toe'] - b['toe_lost']]])[1]
    print(f'\nFisher exact, first two groups: toe goal met p = {met:.3f}, '
          f'lost in goal 2 p = {lost:.3f}')


if __name__ == '__main__':
  main()
