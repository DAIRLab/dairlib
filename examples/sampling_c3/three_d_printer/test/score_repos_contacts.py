"""Scores EE-cone contacts made while repositioning, in compliant-sim cone logs.

Every repositioning plan starts at x0.  With use_predicted_x0_repos, x0 is
the EE position predicted from the previous plan.  On the loop that switches
from C3, that previous plan is a C3 plan, whose knots run into the cone to push
it.  So the first repositioning plan could start 10-20 mm ahead of the gantry,
inside the cone, and the printer finished the push on its way to knot 0.  A
lift-first plan keeps x0's xy, and each loop's plan is rebuilt from the
previous one's prediction, so the gantry kept driving to that point.  In the
2026-09-30 compliant sim M3 (09_30_26/000013, 84.9 s) this stood a freshly
tipped cone back up at the goal change.  Since then
reset_predicted_x0_on_switch_to_repos starts that first plan at the reported
EE.

Per log this reports:
  contacts:  unlatched repositioning-mode EE-cone contact episodes
             (CONTACT_RESULTS force >= 0.5 N, merged across gaps < 0.3 s);
             the significant ones (>= 2 N, or >= 5 mm of cone travel or finger
             bend); and the harmful ones (bend >= 10 mm, or the cone axis's
             world z changing >= 0.2).  It also counts the significant ones
             that moved the cone >= 15 mm (the early goal-0 ones are mostly
             the cone still sliding from a C3 push), and the ones that got
             worse while repositioning: the cone moved >= 15 mm, its axis
             turned >= 0.2, or the bend grew >= 5 mm past its value at the
             episode's onset.  Many episodes are a C3 push still under way at
             the switch, so being harmful doesn't mean repositioning did it.
  classes:   each significant episode, from the unlatched repositioning loops
             in the 0.6 s before it (the printer's command latency):
               handover  knot 0 inside the cone estimate and >= 4 mm from the
                         reported EE, within 0.7 s of a C3->repositioning
                         switch
               later     the same, or the reported EE -> knot 0 segment
                         cutting the estimate, later in the stretch
               path      knot 0 clear, but the plan's path cuts the estimate
               pose      everything clear of the estimate, but it cuts the
                         true (clean) pose
               other     none of these (bent finger, ramp, cone sliding in)
  switches:  every C3->repositioning switch: the distance from the reported EE
             to the published plan's knot 0 (0 with the reset on), and how
             often knot 0 is inside the cone estimate.  For a straight short
             hop, it also checks whether the hop from the reported EE to the
             plan's end still cuts the estimate.
  lifted:    short hops (target under 18 mm away in xy) whose repositioning
             plan lifts first instead of going straight: StraightHopIsClear
             turning the hop away.  Before it, no short hop lifted.  The
             fixed-geometry reroute can also lift one beside the ramp.
  resets:    the '[repos start]' lines in sc3_stdout.txt.

The cone geometry and pose helpers come from score_finger_load_guard.  The
reported EE is C3_ACTUAL's head (the state before ResolvePredictedEEState),
and knot 0 is TRACKING_TRAJECTORY_ACTOR's first knot.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_repos_contacts.py \\
      ~/3d_printer/logs/2026/10_02_26/00000{2..7}
"""

import collections
import multiprocessing
import os.path as op
import sys

import click
import numpy as np
from lcm import EventLog

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'external', 'drake+',
                        'lcmtypes'))
sys.path.append(op.dirname(__file__))
import dairlib  # noqa: E402
import drake  # noqa: E402
from score_finger_load_guard import (EE_RADIUS, axis_z, cone_query,  # noqa: E402
                                     decode_debug, rotation)
from score_ramp_jams import ee_plan, find_log  # noqa: E402

CONTACT_FORCE = 0.5        # N
EPISODE_GAP = 0.3          # s
LATENCY_WINDOW = 0.6       # s of plans before an onset that drove the gantry
HANDOVER_WINDOW = 0.7      # s after a switch
MIN_OFFSET = 0.004         # m, knot 0 to the reported EE
SHORT_HOP = 0.018          # m of xy, the short-hop branch below cruise height
CURRENT_REPOS_TARGET_INDEX = 1
CLASSES = ('handover', 'later', 'path', 'pose', 'other')


def gap(p, quat, pos):
  """EE-sphere surface to cone surface [m]; negative when they overlap."""
  return cone_query(rotation(quat).T @ (p - pos))[0] - EE_RADIUS


def segment_gap(a, b, quat, pos, samples=10):
  return min(gap(a + s * (b - a), quat, pos)
             for s in np.linspace(0.0, 1.0, samples + 1))


def path_gap(knots, quat, pos, max_knots=8):
  """Smallest gap along the plan's first max_knots segments."""
  g = gap(knots[0], quat, pos)
  for k in range(min(max_knots, len(knots) - 1)):
    g = min(g, segment_gap(knots[k], knots[k + 1], quat, pos, 6))
  return g


def hop_cuts(start, end, quat, pos, samples=10):
  """Whether the straight hop overlaps the cone, past what its start does:
  StraightHopIsClear's rule at zero clearance.  A start inside may only move
  out; once out the hop must stay out."""
  previous = gap(start, quat, pos)
  escaping = previous < 0
  for s in np.linspace(0.0, 1.0, samples + 1)[1:]:
    g = gap(start + s * (end - start), quat, pos)
    if g >= 0:
      escaping = False
    elif not escaping or g < previous - 1e-4:
      return True
    previous = g
  return False


def plan_kind(knots):
  if np.linalg.norm(knots - knots[0], axis=1).max() < 2e-4:
    return 'still'
  first = knots[1] - knots[0]
  if np.linalg.norm(first[:2]) < 2e-4 and first[2] > 1e-4:
    return 'lift'
  span = knots[-1] - knots[0]
  unit = span / (np.linalg.norm(span) + 1e-12)
  rel = knots - knots[0]
  off_line = np.linalg.norm(rel - np.outer(rel @ unit, unit), axis=1).max()
  if off_line < 5e-4:
    return 'short' if np.linalg.norm(span[:2]) < SHORT_HOP else 'direct'
  return 'other'


def read_log(path):
  loops, contacts, bend, clean = {}, [], [], []
  t0 = None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      loops.setdefault(msg.utime, {})['debug'] = (
          t, bool(msg.is_c3_mode), bool(msg.jam_tripped))
    elif ch == 'C3_ACTUAL':
      msg = dairlib.lcmt_c3_state.decode(event.data)
      state = np.array(msg.state)
      loops.setdefault(msg.utime, {})['state'] = (state[:3], state[3:7],
                                                  state[7:10])
    elif ch == 'SAMPLE_LOCATIONS':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      saved = msg.saved_traj
      block = saved.trajectories[saved.trajectory_names.index(
          'sample_locations')]
      locations = np.array(block.datapoints)
      if locations.shape[1] > CURRENT_REPOS_TARGET_INDEX:
        loops.setdefault(msg.utime, {})['target'] = \
            locations[:3, CURRENT_REPOS_TARGET_INDEX]
    elif ch == 'TRACKING_TRAJECTORY_ACTOR':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      knots = ee_plan(msg)
      if knots is not None:
        loops.setdefault(msg.utime, {})['plan'] = knots
    elif ch == 'CONTACT_RESULTS':
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      force = 0.0
      for pair in msg.point_pair_contact_info:
        bodies = pair.body1_name + '|' + pair.body2_name
        if 'end_effector' in bodies and 'cone' in bodies:
          force = max(force, np.linalg.norm(pair.contact_force))
      contacts.append((t, force))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      bend.append((t, np.hypot(*msg.position[:2])))
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      msg = dairlib.lcmt_object_state.decode(event.data)
      clean.append((t, *msg.position[:7]))
  if not contacts or not bend:
    raise click.ClickException(
        f'{path} has no CONTACT_RESULTS or FINGER_DEFLECTION_SIMULATION: '
        'this scores compliant-finger sim logs only')
  rows = [v for _, v in sorted(loops.items())
          if all(k in v for k in ('debug', 'state', 'plan'))]
  return dict(
      t=np.array([r['debug'][0] for r in rows]),
      c3=np.array([r['debug'][1] for r in rows]),
      jam=np.array([r['debug'][2] for r in rows]),
      state=[r['state'] for r in rows], plan=[r['plan'] for r in rows],
      target=[r.get('target') for r in rows],
      contacts=np.array(contacts), bend=np.array(bend), clean=np.array(clean))


def count_resets(path):
  stdout = op.join(op.dirname(path), 'sc3_stdout.txt')
  if not op.isfile(stdout):
    return None
  with open(stdout, errors='replace') as f:
    return sum(1 for line in f if line.startswith('[repos start]'))


def episodes_of(log):
  """[onset, end, peak force] of unlatched repositioning contacts."""
  t, c3, jam = log['t'], log['c3'], log['jam']
  contacts = log['contacts']
  loop = np.clip(np.searchsorted(t, contacts[:, 0], side='right') - 1, 0,
                 len(t) - 1)
  active = (contacts[:, 1] > CONTACT_FORCE) & ~c3[loop] & ~jam[loop]
  out = []
  for (tc, force), on in zip(contacts, active):
    if not on:
      continue
    if out and tc - out[-1][1] < EPISODE_GAP:
      out[-1][1] = tc
      out[-1][2] = max(out[-1][2], force)
    else:
      out.append([tc, tc, force])
  return out


def score_log(args):
  arg, = args
  path = find_log(arg)
  log = read_log(path)
  t, c3, jam = log['t'], log['c3'], log['jam']
  clean, bend = log['clean'], log['bend']

  def clean_at(tq):
    return clean[min(np.searchsorted(clean[:, 0], tq), len(clean) - 1), 1:]

  switch = np.flatnonzero(c3[:-1] & ~c3[1:]) + 1
  # Each loop's most recent C3->repositioning switch (or -inf).
  last_switch = np.full(len(t), -np.inf)
  for j in switch:
    last_switch[j:] = t[j]

  classes = collections.Counter()
  significant = harmful = moved = worse = 0
  notes = []
  for onset, end, peak in episodes_of(log):
    sel = (bend[:, 0] >= onset - 0.1) & (bend[:, 0] <= end + 0.3)
    peak_bend = bend[sel, 1].max() if sel.any() else 0.0
    onset_bend = bend[min(np.searchsorted(bend[:, 0], onset),
                          len(bend) - 1), 1]
    before, after = clean_at(onset - 0.1), clean_at(end + 0.5)
    travel = np.linalg.norm(after[4:7] - before[4:7])
    turn = axis_z(after[:4]) - axis_z(before[:4])
    if not (peak >= 2.0 or travel >= 0.005 or peak_bend >= 0.005):
      continue
    significant += 1
    is_harmful = peak_bend >= 0.010 or abs(turn) >= 0.2
    harmful += is_harmful
    moved += travel >= 0.015
    worse += (travel >= 0.015 or abs(turn) >= 0.2
              or peak_bend - onset_bend >= 0.005)
    window = [k for k in range(len(t))
              if onset - LATENCY_WINDOW <= t[k] <= onset
              and not c3[k] and not jam[k]]
    rows = []
    for k in window:
      ee, quat, pos = log['state'][k]
      knots = log['plan'][k]
      true = clean_at(t[k])
      rows.append(dict(
          off=np.linalg.norm(knots[0] - ee), knot0=gap(knots[0], quat, pos),
          catch=segment_gap(ee, knots[0], quat, pos),
          path=path_gap(knots, quat, pos),
          clean=min(segment_gap(ee, knots[0], true[:4], true[4:7]),
                    path_gap(knots, true[:4], true[4:7])),
          since=t[k] - last_switch[k]))
    inside = [r for r in rows if r['knot0'] < 0 and r['off'] >= MIN_OFFSET]
    if inside and min(r['since'] for r in inside) < HANDOVER_WINDOW:
      cls = 'handover'
    elif inside or any(r['catch'] < 0 and r['off'] >= MIN_OFFSET
                       for r in rows):
      cls = 'later'
    elif any(r['knot0'] >= 0 and r['path'] < 0 for r in rows):
      cls = 'path'
    elif rows and all(min(r['catch'], r['path']) >= 0 for r in rows) and any(
        r['clean'] < 0 for r in rows):
      cls = 'pose'
    else:
      cls = 'other'
    classes[cls] += 1
    if is_harmful:
      notes.append(f'    harmful at {onset:.1f} s: {cls}, {peak:.0f} N, bend '
                   f'{1e3 * peak_bend:.0f} mm, cone moved {1e3 * travel:.0f}'
                   f' mm, axis z {turn:+.2f}')

  # The first repositioning plan after each switch.
  offsets, inside, short_cuts, shorts = [], [], 0, 0
  for j in switch:
    if jam[j]:
      continue
    ee, quat, pos = log['state'][j]
    knots = log['plan'][j]
    offsets.append(np.linalg.norm(knots[0] - ee))
    inside.append(gap(knots[0], quat, pos) < 0)
    if plan_kind(knots) == 'short':
      shorts += 1
      short_cuts += hop_cuts(ee, knots[-1], quat, pos)
  offsets, inside = np.array(offsets), np.array(inside, dtype=bool)
  inside_at_switch = int(inside.sum())

  # Short hops that lift first instead of going straight: what
  # StraightHopIsClear turns away (before it, every short hop went straight).
  # The fixed-geometry reroute can also lift a short hop, near the ramp.
  lifted = np.zeros(len(t), dtype=bool)
  for k in range(len(t)):
    target = log['target'][k]
    if c3[k] or jam[k] or target is None:
      continue
    knots = log['plan'][k]
    hop = target - knots[0]
    unit = hop / (np.linalg.norm(hop) + 1e-12)
    rel = knots - knots[0]
    off_hop = np.linalg.norm(rel - np.outer(rel @ unit, unit), axis=1).max()
    lifted[k] = (np.linalg.norm(hop[:2]) < SHORT_HOP and off_hop > 5e-4
                 and plan_kind(knots) == 'lift')
  # A turned-away hop keeps lifting for several loops; a single loop is
  # mostly SAMPLE_LOCATIONS' target lagging a retarget by one loop.
  edges = np.diff(np.r_[0, lifted.astype(int), 0])
  lengths = np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)
  lifted_stretches = int(np.sum(lengths >= 2))

  minutes = (t[-1] - t[0]) / 60
  resets = count_resets(path)
  name = '/'.join(path.split('/')[-3:-1])
  out = [f'{name}: {60 * minutes:.0f} s, {significant} significant '
         f'repositioning contacts ({significant / minutes:.2f}/min), '
         f'{harmful} harmful, {moved} moved the cone >= 15 mm, {worse} got '
         f'worse while repositioning',
         '  classes: ' + ', '.join(f'{c} {classes[c]}' for c in CLASSES),
         f'  {len(offsets)} unlatched switches, {inside_at_switch} with knot 0 '
         f'inside the cone: knot 0 to reported EE {percentiles(offsets)}; '
         f'straight short hops {shorts}, from the reported EE cutting the '
         f'cone {short_cuts}'
         + ('' if resets is None else f'; [repos start] lines {resets}'),
         f'  short hops lifted first: {lifted_stretches} times (2+ loops)']
  out += notes
  return '\n'.join(out), dict(
      minutes=minutes, significant=significant, harmful=harmful, moved=moved,
      worse=worse,
      classes=classes, offsets=offsets, inside=inside, shorts=shorts,
      short_cuts=short_cuts, lifted=lifted_stretches)


def percentiles(offsets):
  if len(offsets) == 0:
    return 'none'
  return (f'p50 {1e3 * np.median(offsets):.1f} / p90 '
          f'{1e3 * np.percentile(offsets, 90):.1f} mm')


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--jobs', default=6, show_default=True)
def main(logs, jobs):
  with multiprocessing.Pool(min(jobs, len(logs))) as pool:
    results = pool.map(score_log, [(log,) for log in logs])
  for text, _ in results:
    print(text)
  totals = [r for _, r in results]
  minutes = sum(r['minutes'] for r in totals)
  classes = sum((r['classes'] for r in totals), collections.Counter())
  offsets = np.concatenate([r['offsets'] for r in totals])
  inside = np.concatenate([r['inside'] for r in totals])
  significant = sum(r['significant'] for r in totals)
  harmful = sum(r['harmful'] for r in totals)
  print(f'\nAll {len(logs)} logs, {minutes:.0f} min: {significant} significant '
        f'repositioning contacts ({significant / minutes:.2f}/min), {harmful}'
        f' harmful ({harmful / minutes:.2f}/min), '
        f'{sum(r["moved"] for r in totals)} moved the cone >= 15 mm, '
        f'{sum(r["worse"] for r in totals)} got worse while repositioning '
        f'({sum(r["worse"] for r in totals) / minutes:.2f}/min)')
  print('  classes: ' + ', '.join(
      f'{c} {classes[c]} ({classes[c] / minutes:.2f}/min)' for c in CLASSES))
  print(f'  {len(offsets)} unlatched switches; knot 0 to reported EE '
        f'{percentiles(offsets)} ({inside.sum()} with knot 0 inside the cone: '
        f'{percentiles(offsets[inside])}; clear of it: '
        f'{percentiles(offsets[~inside])})')
  print(f'  straight short hops at a switch {sum(r["shorts"] for r in totals)}'
        f', from the reported EE cutting the cone '
        f'{sum(r["short_cuts"] for r in totals)}; short hops lifted first '
        f'{sum(r["lifted"] for r in totals)} times')


if __name__ == '__main__':
  main()
