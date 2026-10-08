"""Scores what happens to the cone after each jam trip in compliant-finger sim
logs.

Simulation only: it reads FINGER_DEFLECTION_SIMULATION, CONTACT_RESULTS and
OBJECT_STATE_SIMULATION_CLEAN, which hardware has no equivalent of.

Each trip runs from a rising edge of jam_tripped on SAMPLING_C3_DEBUG to its
release.  The gantry keeps going along its pushing direction for a while after
the trip (printer command lag), then turns back on the unload leg toward the
frozen entry point.  The turn-back is where the reported EE (C3_ACTUAL) is
furthest along its approach over the 0.3 s before the trip.  One row per trip:
  goal     goal index (SAMPLING_C3_DEBUG's detected_goal_changes plus
           --first_goal)
  tier     D deep tier armed while latched, L load tier at the trip, G other
           (gap, travel or force)
  d_trip   true finger deflection at the trip [mm]
  over     how far the EE kept going along its approach after the trip [mm]
  slip     when the finger unloaded (deflection < 2 mm) before the turn-back
           [s after the trip]; '-' if it didn't or was never loaded (< 2 mm)
  unl      peak jam_unload_distance while latched [mm]
  F_out    peak finger-cone force from the trip to the turn-back [N]
  F_back   peak finger-cone force from the turn-back to 0.3 s after the
           release [N]
  dcone    the cone's xy displacement [mm] and axis change (norm of the
           difference of unit axes) from the trip to 0.5 s after the release,
           then the same split at the turn-back: out (trip to turn-back) and
           back (turn-back to 0.5 s after the release)
  HARM     dcone >= 10 mm or axis change >= 0.3, with the phase(s) where the
           same bar was crossed

The totals at the end count:
  no-contact trips   d_trip < 2 mm and F_out < 3 N
  loaded slips       the finger unloaded before the turn-back
  leg-back contact   F_back >= 3 N
  harmful legs back  leg-back contact and the back phase crossed the HARM bar

On roadmap B's 24 goal-3-start runs (2026-10-06, see Usage) there were 89
trips: 33 no-contact, 38 with leg-back contact, 19 harmful legs back (7 of them
after a no-contact trip).

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_jam_trips.py \\
      --first_goal 3 \\
      ~/3d_printer/logs/2026/10_06_26/0000{17,19,21,23,25,27,29,31}
diagnose_jam_trip.py prints the loop-by-loop timeline around one trip.
"""

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
from run_sim_batch import axis_world  # noqa: E402
from score_finger_load_guard import decode_debug, find_log  # noqa: E402

LOADED = 0.002          # deflection that counts as a loaded finger [m]
CONTACT_N = 3.0         # finger-cone force that counts as contact [N]
HARM_XY = 10.0          # cone displacement that counts as harm [mm]
HARM_AXIS = 0.3         # cone axis change that counts as harm [-]


def tip_cone_force(data):
  msg = drake.lcmt_contact_results_for_viz.decode(data)
  force = 0.0
  for c in msg.point_pair_contact_info:
    names = c.body1_name + c.body2_name
    if 'end_effector' in names and 'cone' in names:
      force = max(force, np.linalg.norm(c.contact_force))
  return force


def read_log(path):
  t0 = None
  debug, ee, defl, force, cone = [], [], [], [], []
  for event in EventLog(find_log(path), 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      d = decode_debug(event.data)
      debug.append((t, d.jam_tripped, d.jam_tripped_by_load, d.jam_deep_armed,
                    d.jam_unload_distance, d.detected_goal_changes))
    elif ch == 'C3_ACTUAL':
      ee.append((t, *dairlib.lcmt_c3_state.decode(event.data).state[:3]))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      position = dairlib.lcmt_robot_output.decode(event.data).position
      defl.append((t, np.hypot(*position[:2])))
    elif ch == 'CONTACT_RESULTS':
      force.append((t, tip_cone_force(event.data)))
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      q = np.array(dairlib.lcmt_object_state.decode(event.data).position[:7])
      cone.append((t, *q[4:7], *axis_world(q[:4] / np.linalg.norm(q[:4]))))
  return [np.array(a) for a in (debug, ee, defl, force, cone)]


def at(series, t):
  return series[min(np.searchsorted(series[:, 0], t), len(series) - 1), 1:]


def window(series, t1, t2):
  return series[(series[:, 0] >= t1) & (series[:, 0] <= t2)]


def cone_change(a, b):
  return 1e3 * np.linalg.norm(b[:2] - a[:2]), np.linalg.norm(b[3:] - a[3:])


def is_harm(xy, axis):
  return xy >= HARM_XY or axis >= HARM_AXIS


def score_log(path):
  return score_trips(path, *read_log(path))


def score_trips(path, debug, ee, defl, force, cone):
  """One row per trip, from series in read_log's layout."""
  tripped = debug[:, 1] > 0.5
  rows = []
  for i in np.flatnonzero(tripped[1:] & ~tripped[:-1]) + 1:
    t_on = debug[i, 0]
    j = next((k for k in range(i, len(debug)) if not tripped[k]),
             len(debug) - 1)
    t_off = debug[j, 0]
    latched = debug[i:j + 1]
    tier = ('D' if latched[:, 3].max() > 0.5
            else 'L' if debug[i, 2] > 0.5 else 'G')
    approach = (at(ee, t_on) - at(ee, t_on - 0.3))[:2]
    approach /= np.linalg.norm(approach) + 1e-9
    leg = window(ee, t_on, t_off)
    if not len(leg):
      continue
    along = (leg[:, 1:3] - at(ee, t_on)[:2]) @ approach
    t_turn = leg[np.argmax(along), 0]
    d_trip = at(defl, t_on)[0]
    out_defl = window(defl, t_on, t_turn)
    unloaded = np.flatnonzero(out_defl[:, 1] < LOADED)
    slip = (out_defl[unloaded[0], 0] - t_on
            if len(unloaded) and d_trip > LOADED else None)
    c_on, c_turn = at(cone, t_on), at(cone, t_turn)
    c_end = at(cone, t_off + 0.5)
    rows.append(dict(
        log=op.basename(op.normpath(path)), goal=int(debug[i, 5]), t=t_on,
        dur=t_off - t_on, t_turn=t_turn, tier=tier, d_trip=1e3 * d_trip,
        over=1e3 * along.max(), slip=slip, unl=1e3 * latched[:, 4].max(),
        F_out=window(force, t_on, t_turn)[:, 1].max(initial=0),
        F_back=window(force, t_turn, t_off + 0.3)[:, 1].max(initial=0),
        total=cone_change(c_on, c_end), out=cone_change(c_on, c_turn),
        back=cone_change(c_turn, c_end)))
  return rows


def format_row(r, first_goal):
  slip = f'{r["slip"]:4.2f}' if r['slip'] is not None else '   -'
  phase = '+'.join(name for name in ('out', 'back') if is_harm(*r[name]))
  harm = f'  HARM {phase}' if is_harm(*r['total']) else ''
  return (f'{r["log"]} g{r["goal"] + first_goal} {r["t"]:7.2f} '
          f'{r["dur"]:4.1f}s {r["tier"]} d_trip {r["d_trip"]:5.1f} '
          f'over {r["over"]:5.1f} '
          f'slip {slip} unl {r["unl"]:5.1f} F_out {r["F_out"]:5.1f} '
          f'F_back {r["F_back"]:5.1f} dcone {r["total"][0]:5.1f} '
          f'{r["total"][1]:4.2f} out {r["out"][0]:5.1f} {r["out"][1]:4.2f} '
          f'back {r["back"][0]:5.1f} {r["back"][1]:4.2f}{harm}')


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--first_goal', default=0, show_default=True,
              help='Goal index of the run\'s first goal, for runs started at '
                   'a later goal (3 for the goal-3-start runs).')
@click.option('--jobs', default=8, show_default=True,
              help='Logs read in parallel.')
def main(logs, first_goal, jobs):
  with multiprocessing.Pool(jobs) as pool:
    per_log = pool.map(score_log, [op.expanduser(p) for p in logs])
  rows = [r for log_rows in per_log for r in log_rows]
  for r in rows:
    print(format_row(r, first_goal))
  if not rows:
    print('No trips.')
    return
  no_contact = [r for r in rows
                if r['d_trip'] < 1e3 * LOADED and r['F_out'] < CONTACT_N]
  back_contact = [r for r in rows if r['F_back'] >= CONTACT_N]
  harmful = [r for r in back_contact if is_harm(*r['back'])]
  print(f'\n{len(rows)} trips in {len(logs)} logs, overshoot p50 '
        f'{np.median([r["over"] for r in rows]):.0f} mm.')
  print(f'  no-contact trips:   {len(no_contact)}')
  print(f'  loaded slips:       {sum(r["slip"] is not None for r in rows)}')
  print(f'  leg-back contact:   {len(back_contact)}')
  print(f'  harmful legs back:  {len(harmful)} '
        f'({sum(r in no_contact for r in harmful)} after a no-contact trip): '
        + ', '.join(f'{r["log"]}@{r["t"]:.1f}' for r in harmful))


if __name__ == '__main__':
  main()
