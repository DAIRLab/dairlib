"""Scores the jam guard's unload legs in compliant-finger sim logs, and replays
issue #14's check: lift in place instead of driving an unload leg that would
run into the object.

After a jam trip the latched plan heads horizontally, at the current height,
for the frozen entry point (jam_entry_point_ in sampling_based_c3_controller.cc),
and ShouldUnloadBeforeLifting's [repos unload] leg does the same without the
latch.  When the finger has already slipped free, or was never loaded, that
straight leg can run the tip back into the cone.  The check replayed here fires
on a loop when the reported EE reads at least --start_gap outside the estimated
cone and the straight leg from it to the entry point dips at least
--entry_depth into the estimate.  A caught finger's leg starts inside the
estimate; one that starts outside and enters is taken to be free -- which a
pass-through or a lagging estimate can fake, hence the ground-truth scoring.

The entry point is not logged.  It is rebuilt from the published plan
(TRACKING_TRAJECTORY_ACTOR): the leg's first knots run from knot 0 towards it,
and jam_unload_distance is the reported EE's horizontal distance to it.  Each
loop's plan is logged just before its SAMPLING_C3_DEBUG message and its
C3_ACTUAL (reported EE plus the estimated cone) just after.  Gaps use the
controller's hexagonal cone mesh (score_vertical_squeeze.HexCone).

Ground truth (simulation only): FINGER_DEFLECTION_SIMULATION, CONTACT_RESULTS
and OBJECT_STATE_SIMULATION_CLEAN.  Per trip, with score_jam_trips.py's
definitions:
  cls      NOCONT no finger contact, SLIP loaded then slipped before the
           turn-back, CAUGHT loaded through the turn-back, CONT the rest
  harm     a harmful leg back
  fire     first loop the check fires [s after the trip], '-' if none; '*'
           marks one in time, at least MARGIN before the leg-back contact
  bend     true bend at the fire, then its minimum over the next LATENCY s,
           when a lift would land [mm]
  caught   the finger stays bent >= CAUGHT_MM over that span, so the lift
           lands on a loaded finger.  What holds it: side (finger-cone force
           mostly horizontal), top (mostly vertical: the tip pinned on a face
           it presses down on) or ramp (no cone contact).
The [repos unload] legs come from sc3_stdout.txt, grouped into episodes; each
is scored for leg contact and harm the same way, and for the check.

--reference LOG@T replays the check at a load-tier trip at T [s] on a log
without one, such as a pass-through recorded before the tier existed or a
hardware launch: the entry point is the rebuilt load tier's anchor at T
(score_finger_load_guard.FingerLoadEstimator) and the check runs over the
LATENCY s after, while the logged motion still matches what the controller
would have done.

On roadmap B's 24 goal-3-start runs (10_06_26/000017-49, as in
score_jam_trips.py), the default check acts on 12 of the 19 harmful legs back,
10 of them in time. It would also lift 4 fingers still caught when the lift
lands, all pinned on the lying cone's top.  --reference
09_30_26/000007@117.96 FIRES at 118.25 s, with the finger bent 27 mm and
lifting the cone: that is the pass-through that launched it.  So the check
was not shipped (issue #14).  Of the 64 [repos unload] episodes, 9 were
harmful, mostly C3's push still in flight at the handover.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_unload_legs.py \\
      ~/3d_printer/logs/2026/10_06_26/0000{17,19,21,23,25,27,29,31} \\
      [--grid] [--trips] [--jobs 8] \\
      [--reference ~/3d_printer/logs/2026/09_30_26/000007@117.96]
"""

import multiprocessing
import os.path as op
import re
import sys

import click
import numpy as np
from lcm import EventLog

sys.path.append(op.dirname(__file__))
import score_jam_trips as J  # noqa: E402
from score_finger_load_guard import (EE_RADIUS,  # noqa: E402
                                     FingerLoadEstimator, cone_query,
                                     decode_debug, find_log, read_log as
                                     read_replay_log, rotation)
from score_ramp_jams import ee_plan  # noqa: E402
from score_vertical_squeeze import CONE  # noqa: E402
import dairlib  # noqa: E402
import drake  # noqa: E402

LATENCY = 0.35        # the printer's command latency in the sim [s]
MARGIN = 0.15         # a fire must land this long before the contact [s]
CAUGHT_MM = 5.0       # a bend held over the latency that counts as caught
TOP_SHARE = 0.5       # vertical share of the finger-cone force for 'top'
UNLOAD_RELEASE = 0.004
LEG_STEP = 0.002      # leg sampling [m]
EPISODE_GAP = 0.3     # [repos unload] lines further apart start a new one [s]
GRID = [(g, m) for g in (-0.002, 0.0, 0.002, 0.005)
        for m in (0.003, 0.005, 0.008)]


def finger_cone_force(data):
  """Largest finger-cone point-pair force [N] and its vertical share."""
  msg = drake.lcmt_contact_results_for_viz.decode(data)
  best = np.zeros(3)
  for c in msg.point_pair_contact_info:
    names = c.body1_name + c.body2_name
    if 'end_effector' in names and 'cone' in names:
      f = np.array(c.contact_force)
      if np.linalg.norm(f) > np.linalg.norm(best):
        best = f
  norm = np.linalg.norm(best)
  return norm, abs(best[2]) / norm if norm > 0 else 0.0


def read_log(path):
  """score_jam_trips.read_log's series, plus per-loop snapshots (plan, the
  C3_ACTUAL that follows the debug message, unload distance) and the force's
  vertical share."""
  t0 = None
  debug, ee, defl, force, cone, loops = [], [], [], [], [], []
  plan = None
  for event in EventLog(find_log(path), 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      d = decode_debug(event.data)
      debug.append((t, d.jam_tripped, getattr(d, 'jam_tripped_by_load', 0),
                    getattr(d, 'jam_deep_armed', 0), d.jam_unload_distance,
                    getattr(d, 'detected_goal_changes', 0)))
      loops.append(dict(t=t, ctrl_t=d.utime / 1e6, tripped=d.jam_tripped,
                        unl=d.jam_unload_distance,
                        pushing=getattr(d, 'jam_retreat_pushing', False),
                        plan=plan, state=None))
    elif ch == 'C3_ACTUAL':
      state = np.array(dairlib.lcmt_c3_state.decode(event.data).state)
      ee.append((t, *state[:3]))
      if loops and loops[-1]['state'] is None:
        loops[-1]['state'] = state
    elif ch == 'TRACKING_TRAJECTORY_ACTOR':
      knots = ee_plan(dairlib.lcmt_timestamped_saved_traj.decode(event.data))
      if knots is not None:
        plan = knots
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      position = dairlib.lcmt_robot_output.decode(event.data).position
      defl.append((t, np.hypot(*position[:2])))
    elif ch == 'CONTACT_RESULTS':
      force.append((t, *finger_cone_force(event.data)))
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      q = np.array(dairlib.lcmt_object_state.decode(event.data).position[:7])
      cone.append((t, *q[4:7], *J.axis_world(q[:4] / np.linalg.norm(q[:4]))))
  series = [np.array(a) for a in (debug, ee, defl, force, cone)]
  return series, loops


def leg_gaps(ee, entry_xy, quat, position):
  """The reported EE's gap to the estimated cone, and the deepest gap along
  the horizontal leg from it to the entry point [m]."""
  rot = rotation(quat / np.linalg.norm(quat))
  start = CONE.gap(ee, rot, position)[0]
  length = np.linalg.norm(entry_xy - ee[:2])
  deepest = start
  for f in np.linspace(0, 1, max(2, int(np.ceil(length / LEG_STEP)) + 1)):
    point = np.r_[ee[:2] + f * (entry_xy - ee[:2]), ee[2]]
    deepest = min(deepest, CONE.gap(point, rot, position)[0])
  return start, deepest


def entry_from_plan(loop):
  """The entry point's xy, rebuilt from the loop's plan and unload distance;
  None when the plan's first segment does not head anywhere."""
  plan, state, unl = loop['plan'], loop['state'], loop['unl']
  if plan is None or state is None or len(plan) < 2 or not np.isfinite(unl):
    return None
  heading = (plan[1] - plan[0])[:2]
  if np.linalg.norm(heading) < 1e-6:
    return None
  u = heading / np.linalg.norm(heading)
  a = plan[0][:2] - state[:2]
  disc = (a @ u) ** 2 - (a @ a - unl ** 2)
  if disc < 0:
    return None
  return plan[0][:2] + (-(a @ u) + np.sqrt(disc)) * u


def check_loop(loop):
  entry = entry_from_plan(loop)
  if entry is None:
    return None
  state = loop['state']
  return leg_gaps(state[:3], entry, state[3:7], state[7:10])


def fires(gaps, start_gap, entry_depth):
  return (gaps is not None and gaps[0] >= start_gap and
          gaps[1] <= -entry_depth)


def ground_truth(t, defl, force):
  """Bend at t, its minimum over the latency, and what holds the finger."""
  bend = J.at(defl, t)[0]
  span = J.window(defl, t, t + LATENCY)
  held = span[:, 1].min(initial=bend)
  touching = J.window(force, t, t + LATENCY)
  touching = touching[touching[:, 1] >= J.CONTACT_N]
  if not len(touching):
    what = 'ramp'
  else:
    share = (touching[:, 1] * touching[:, 2]).sum() / touching[:, 1].sum()
    what = 'top' if share >= TOP_SHARE else 'side'
  return 1e3 * bend, 1e3 * held, what


def classify(r):
  if r['d_trip'] < 1e3 * J.LOADED and r['F_out'] < J.CONTACT_N:
    return 'NOCONT'
  if r['slip'] is not None:
    return 'SLIP'
  return 'CAUGHT' if r['d_trip'] >= 1e3 * J.LOADED else 'CONT'


def stdout_episodes(path):
  """Controller times of the [repos unload] lines, grouped into episodes."""
  log_dir = path if op.isdir(path) else op.dirname(path)
  name = op.join(log_dir, 'sc3_stdout.txt')
  if not op.isfile(name):
    return []
  times = [float(m.group(1)) for m in
           re.finditer(r'\[repos unload\] t=([-\d.eE+]+)', open(name).read())]
  episodes = []
  for t in times:
    if episodes and t - episodes[-1][1] <= EPISODE_GAP:
      episodes[-1][1] = t
    else:
      episodes.append([t, t])
  return episodes


def score_log(path):
  series, loops = read_log(path)
  debug, ee, defl, force, cone = series
  if not len(defl) or not len(cone):
    return dict(log=path, trips=[], repos=[], sim=False)
  log_t = np.array([l['t'] for l in loops])
  ctrl_t = np.array([l['ctrl_t'] for l in loops])
  trips = []
  for r in J.score_trips(path, debug, ee, defl, force, cone):
    i = int(np.searchsorted(log_t, r['t'] - 1e-6))
    # The leg runs until the finger counts as unloaded or the push guard
    # lifts; past that the plan follows the escape, not the entry point.
    checked = []
    for loop in loops[i:]:
      if not loop['tripped'] or loop['pushing'] or not (
          loop['unl'] >= UNLOAD_RELEASE):
        break
      checked.append((loop['t'], check_loop(loop)))
    contact = J.window(force, r['t_turn'], r['t'] + r['dur'] + 0.3)
    contact = contact[contact[:, 1] >= J.CONTACT_N]
    trips.append(dict(r, cls=classify(r),
                      harm=bool(r['F_back'] >= J.CONTACT_N and
                                J.is_harm(*r['back'])),
                      t_hit=contact[0, 0] if len(contact) else None,
                      checked=checked))
  repos = []
  for c0, c1 in stdout_episodes(path):
    i0 = int(np.searchsorted(ctrl_t, c0 - 1e-6))
    i1 = int(np.searchsorted(ctrl_t, c1 - 1e-6))
    if i0 >= len(loops):
      continue
    i1 = min(i1, len(loops) - 1)
    t0, t1 = log_t[i0], log_t[i1]
    peak = J.window(force, t0, t1 + 0.5)[:, 1].max(initial=0)
    moved = J.cone_change(J.at(cone, t0), J.at(cone, t1 + 0.5))
    repos.append(dict(t=t0, dur=t1 - t0, bend=1e3 * J.at(defl, t0)[0],
                      peak=peak,
                      harm=bool(peak >= J.CONTACT_N and J.is_harm(*moved)),
                      checked=[(l['t'], check_loop(l))
                               for l in loops[i0:i1 + 1]]))
  return dict(log=op.basename(op.normpath(path)), trips=trips, repos=repos,
              sim=True, defl=defl, force=force)


def first_fire(checked, start_gap, entry_depth):
  return next((t for t, gaps in checked
               if fires(gaps, start_gap, entry_depth)), None)


def tally(results, start_gap, entry_depth):
  """Per-setting counts over every trip and [repos unload] episode."""
  out = dict(fired=0, harm=0, harm_in_time=0, caught=[], repos_fired=0,
             repos_caught=[])
  for res in results:
    for tr in res['trips']:
      t = first_fire(tr['checked'], start_gap, entry_depth)
      if t is None:
        continue
      out['fired'] += 1
      if tr['harm']:
        out['harm'] += 1
        out['harm_in_time'] += (tr['t_hit'] is None or
                                t <= tr['t_hit'] - MARGIN)
      bend, held, what = ground_truth(t, res['defl'], res['force'])
      if held >= CAUGHT_MM:
        out['caught'].append(
            f'{res["log"]}@{tr["t"]:.1f}+{t - tr["t"]:.2f} {what} '
            f'{bend:.0f}/{held:.0f}mm')
    for ep in res['repos']:
      t = first_fire(ep['checked'], start_gap, entry_depth)
      if t is None:
        continue
      out['repos_fired'] += 1
      bend, held, what = ground_truth(t, res['defl'], res['force'])
      if held >= CAUGHT_MM:
        out['repos_caught'].append(f'{res["log"]}@{t:.1f} {what} '
                                   f'{bend:.0f}/{held:.0f}mm')
  return out


def print_trips(results, start_gap, entry_depth):
  for res in results:
    for tr in res['trips']:
      t = first_fire(tr['checked'], start_gap, entry_depth)
      if t is None:
        fire = '    -  '
        truth = ''
      else:
        late = tr['t_hit'] is not None and t > tr['t_hit'] - MARGIN
        fire = f'{t - tr["t"]:5.2f}{" " if late else "*"} '
        bend, held, what = ground_truth(t, res['defl'], res['force'])
        truth = (f'bend {bend:4.1f}/{held:4.1f}'
                 + (f' CAUGHT {what}' if held >= CAUGHT_MM else ''))
      print(f'{res["log"]} {tr["t"]:7.2f} {tr["cls"]:6s} '
            f'{"HARM" if tr["harm"] else "    "} fire {fire} {truth}')


def reference(spec, start_gap, entry_depth, loop_offset=0.09, clear_gap=0.005):
  """The check at a hypothetical load-tier trip at T on LOG."""
  folder, when = spec.rsplit('@', 1)
  when = float(when)
  loops, ee, obj, clean, defl = read_replay_log(find_log(op.expanduser(folder)))
  t = loops[:, 0] - loop_offset
  latched = loops[:, 1] > 0
  ee_at = np.stack([np.interp(t, ee[:, 0], ee[:, k]) for k in (1, 2, 3)], 1)
  latest = np.clip(np.searchsorted(obj[:, 0], t, side='right') - 1, 0,
                   len(obj) - 1)
  i_trip = int(np.searchsorted(loops[:, 0], when))
  estimator = FingerLoadEstimator(clear_gap)
  entry = None
  rows = []
  for i in range(len(t)):
    o = obj[latest[i]]
    rot, pos = rotation(o[1:5]), o[5:8]
    distance, normal_o = cone_query(rot.T @ (ee_at[i] - pos))
    estimator.update(ee_at[i], distance - EE_RADIUS, rot @ normal_o, rot, pos,
                     latched[i] or (entry is not None))
    if i == i_trip:
      if estimator.anchor_o is None:
        return f'{spec}: no anchor at the trip'
      entry = (rot @ estimator.anchor_o + pos)[:2]
    if entry is not None:
      if loops[i, 0] > when + LATENCY:
        break
      start, deepest = leg_gaps(ee_at[i], entry, o[1:5], pos)
      bend = (1e3 * np.interp(loops[i, 0], defl[:, 0],
                              np.hypot(defl[:, 1], defl[:, 2]))
              if defl is not None else np.nan)
      rows.append((loops[i, 0], start, deepest, bend))
  fired = [r for r in rows if r[1] >= start_gap and r[2] <= -entry_depth]
  detail = ', '.join(f'{r[0]:.2f}: start {1e3 * r[1]:+.1f} leg '
                     f'{1e3 * r[2]:+.1f} bend {r[3]:.0f}' for r in rows)
  return (f'{spec}: {"FIRES at " + f"{fired[0][0]:.2f}" if fired else "no fire"}'
          f'  [{detail}]')


@click.command()
@click.argument('logs', nargs=-1)
@click.option('--start_gap', default=0.0, show_default=True,
              help='The reported EE must read at least this far outside the '
                   'estimate [m].')
@click.option('--entry_depth', default=0.005, show_default=True,
              help='...and the leg must dip at least this far into it [m].')
@click.option('--grid', is_flag=True, help='Tabulate a grid of both settings.')
@click.option('--trips', is_flag=True, help='One row per trip.')
@click.option('--reference', 'references', multiple=True,
              help='LOG@T: replay the check at a load-tier trip at T [s].')
@click.option('--jobs', default=8, show_default=True)
def main(logs, start_gap, entry_depth, grid, trips, references, jobs):
  for spec in references:
    print(reference(spec, start_gap, entry_depth))
  if not logs:
    return
  with multiprocessing.Pool(jobs) as pool:
    results = [r for r in pool.map(score_log, [op.expanduser(p) for p in logs])
               if r['sim']]
  if trips:
    print_trips(results, start_gap, entry_depth)
  all_trips = [tr for r in results for tr in r['trips']]
  episodes = [ep for r in results for ep in r['repos']]
  harmful = [tr for tr in all_trips if tr['harm']]
  print(f'\n{len(all_trips)} trips in {len(results)} logs, {len(harmful)} '
        f'harmful legs back ('
        + ', '.join(f'{c} {sum(tr["cls"] == c for tr in harmful)}'
                    for c in ('NOCONT', 'SLIP', 'CAUGHT', 'CONT')) + ').')
  print(f'{len(episodes)} [repos unload] episodes: '
        f'{sum(ep["peak"] >= J.CONTACT_N for ep in episodes)} with >= '
        f'{J.CONTACT_N:.0f} N of finger-cone force, '
        f'{sum(ep["harm"] for ep in episodes)} harmful'
        + (': ' + ', '.join(f'{r["log"]}@{ep["t"]:.1f}' for r in results
                            for ep in r['repos'] if ep['harm'])
           if any(ep['harm'] for ep in episodes) else '') + '.')
  settings = GRID if grid else [(start_gap, entry_depth)]
  print('\nstart  depth | trips fired  harmful (in time)  lifts on caught | '
        'repos fired  caught')
  for g, m in settings:
    s = tally(results, g, m)
    print(f'{1e3 * g:+5.1f} {1e3 * m:5.1f}  | {s["fired"]:11d}  '
          f'{s["harm"]:3d}/{len(harmful)} ({s["harm_in_time"]:2d})       '
          f'{len(s["caught"]):3d}        | {s["repos_fired"]:11d}  '
          f'{len(s["repos_caught"]):3d}')
    for line in s['caught'] + s['repos_caught']:
      print(f'      caught: {line}')


if __name__ == '__main__':
  main()
