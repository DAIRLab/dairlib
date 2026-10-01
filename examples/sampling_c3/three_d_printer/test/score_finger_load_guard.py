"""Scores the jam guard's load tier offline over cone demo logs.

The load tier trips when the compliant finger's estimated load -- how far the
printer's reported EE has run past the tangent plane where it last touched the
cone from outside, carried along with the cone's pose estimate -- reaches
load_trip (FingerLoadEstimator in examples/sampling_c3/jamming_metrics.h).
This rebuilds that estimate per control loop from PRINTER_STATE and
OBJECT_STATE against an analytic cone, so it can score logs recorded before the
tier existed; logs that carry jam_finger_load are also scored on the
controller's own value, and the two are compared.

Simulation logs are scored against the compliant finger's ground truth,
FINGER_DEFLECTION_SIMULATION and OBJECT_STATE_SIMULATION_CLEAN:
  - releases:  the finger bends >= --peak_mm and then lets go, and the cone
               moves >= 20 mm or its axis z changes by >= 0.5 within 1 s.
               A release is "missed" by the logged guard when jam_tripped did
               not rise in the 1.5 s before it, and "caught" by the load tier
               when the tier fires between 0.5 s before the bend began and its
               peak.
  - no-load:   load-tier trips with the true bend under 8 mm from 0.3 s
               before to 1 s after.  "loaded" trips are the rest.
  - tips:      the cone's axis z falls from > 0.9 to < 0.4 within 3 s and
               stays under 0.6 for 3 s; reported with whether the load tier
               fired in the 3 s before (a tip by a release is a lucky one).
Hardware logs only get the list of load-tier trip times, to check against the
known launch events.

The rebuild samples the reported EE and the object estimate --loop_offset
before each SAMPLING_C3_DEBUG message, which is roughly when the controller
read them; at 0.09 s it matched the logged reported-EE gap to 0.5 mm (median)
on 09_30_26/000014.  The analytic cone is a true cone, slightly larger than the
controller's collision mesh.

On the 2026-09-29/30 compliant-sim logs (09_29_26/0000{11,12} and every
09_30_26 run), before the tier existed:  25 releases, 22 missed by the logged
guard; at 20 mm the tier catches 15 (median true bend 22 mm at the trip, 0.37 s
ahead of the release at p10) with 18 no-load trips, and fires before 11 of 16
tips; at 15 mm it catches 20 with 60 no-load trips.  On the hardware logs
09_23_26_pbody/00000{0,1,2} it fires at 150.4 s and 111.7 s, 0.55 s and 0.4 s
ahead of the deep tier at the hw0 and hw2 launches (hw1 is the deep tier's
alone), plus 4 other trips; 6 more across 09_17_26/00000{0,1}.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_finger_load_guard.py \\
      ~/3d_printer/logs/2026/09_30_26/0000{00..14} \\
      ~/3d_printer/logs/2026/09_29_26/0000{11,12}
"""

import glob
import os.path as op
import sys

import click
import numpy as np
from lcm import EventLog
from scipy.spatial.transform import Rotation as R

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib  # noqa: E402
from archive import dairlib as archive_dairlib  # noqa: E402

# Same cone and EE as score_deep_jam_guard.py: symmetry axis along body +x
# from the base at the origin, and the printer controller's 10 mm EE sphere.
CONE_HEIGHT = 0.0494
CONE_BASE_RADIUS = 0.0254
EE_RADIUS = 0.010
PRINTER_Z_TO_EE_CENTRE = 0.110864
# FingerLoadEstimator::kMinHorizontalNormal.
MIN_HORIZONTAL_NORMAL = 0.3


def find_log(path):
  """Accepts a log file or the folder holding one."""
  if op.isfile(path):
    return path
  found = sorted(glob.glob(op.join(path, '*log-*')))
  if not found:
    raise click.ClickException(f'no log in {path}')
  return found[0]


def decode_debug(data):
  """Newest layout first; only jam_tripped is needed from the older ones,
  which every generation carries (see process_lcm_logs.py)."""
  for lcmt in (dairlib.lcmt_sampling_c3_debug,
               archive_dairlib.lcmt_sampling_c3_debug_v8,
               archive_dairlib.lcmt_sampling_c3_debug_v7,
               archive_dairlib.lcmt_sampling_c3_debug_v6,
               archive_dairlib.lcmt_sampling_c3_debug_v5,
               archive_dairlib.lcmt_sampling_c3_debug_v4,
               archive_dairlib.lcmt_sampling_c3_debug_v3,
               archive_dairlib.lcmt_sampling_c3_debug_v2,
               archive_dairlib.lcmt_sampling_c3_debug):
    try:
      return lcmt.decode(data)
    except ValueError:
      pass
  raise ValueError('SAMPLING_C3_DEBUG in no known layout')


def cone_query(p_body):
  """Signed distance from a point to the solid cone [m], and the outward unit
  gradient, both in the cone's body frame."""
  a = p_body[0]
  radial = p_body[1:]
  r = np.linalg.norm(radial)
  u = radial / r if r > 1e-12 else np.array([1.0, 0.0])
  p = np.array([a, r])

  def nearest_on(s0, s1):
    t = np.clip((p - s0) @ (s1 - s0) / ((s1 - s0) @ (s1 - s0)), 0.0, 1.0)
    c = s0 + t * (s1 - s0)
    return np.linalg.norm(p - c), c

  base = nearest_on(np.array([0.0, 0.0]), np.array([0.0, CONE_BASE_RADIUS]))
  side = nearest_on(np.array([0.0, CONE_BASE_RADIUS]),
                    np.array([CONE_HEIGHT, 0.0]))
  d, c = base if base[0] < side[0] else side
  inside = a > 0 and a / CONE_HEIGHT + r / CONE_BASE_RADIUS < 1
  if inside:
    n = (np.array([-1.0, 0.0]) if base[0] < side[0] else
         np.array([CONE_BASE_RADIUS, CONE_HEIGHT]) /
         np.hypot(CONE_BASE_RADIUS, CONE_HEIGHT))
    return -d, np.array([n[0], n[1] * u[0], n[1] * u[1]])
  n = (p - c) / d if d > 1e-12 else np.array([0.0, 1.0])
  return d, np.array([n[0], n[1] * u[0], n[1] * u[1]])


class FingerLoadEstimator:
  """Port of FingerLoadEstimator (jamming_metrics.h); keep the two in step."""

  def __init__(self, clear_gap):
    self.clear_gap = clear_gap
    self.anchor_o = None
    self.normal_o = None
    self.engaged = False
    self.was_latched = False

  def update(self, ee, gap, normal, rot, pos, latched):
    if self.was_latched and not latched:
      self.anchor_o, self.engaged = None, False
    self.was_latched = latched
    if np.isfinite(gap) and gap < 0:
      self.engaged = True
    load = 0.0
    if self.anchor_o is not None:
      normal_w = rot @ self.normal_o
      horizontal = np.linalg.norm(normal_w[:2])
      if horizontal >= MIN_HORIZONTAL_NORMAL:
        anchor_w = rot @ self.anchor_o + pos
        load = -(ee - anchor_w)[:2] @ (normal_w[:2] / horizontal)
    if (not latched and np.isfinite(gap) and gap >= 0 and
        np.linalg.norm(normal) > 1e-9):
      on_anchor_side = (self.anchor_o is not None and
                        normal @ (rot @ self.normal_o) > 0)
      if (self.anchor_o is None or not self.engaged or on_anchor_side or
          gap >= self.clear_gap):
        self.anchor_o = rot.T @ (ee - pos)
        self.normal_o = rot.T @ (normal / np.linalg.norm(normal))
        self.engaged = False
        load = 0.0
    return np.nan if self.anchor_o is None else max(load, 0.0)


def read_log(path):
  """Per-loop debug rows plus the EE, object and (sim) ground-truth series."""
  loops, ee, obj, clean, defl = [], [], [], [], []
  t0 = None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      loops.append((t, msg.jam_tripped,
                    getattr(msg, 'jam_tripped_by_load', False),
                    getattr(msg, 'jam_finger_load', np.nan)))
    elif ch in ('PRINTER_STATE', 'PRINTER_STATE_SIMULATION'):
      if ee and t - ee[-1][0] < 0.005:
        continue  # 1 kHz in some sims; the control loop is ~15 Hz
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      ee.append((t, q['x_axis_joint'], q['y_axis_joint'],
                 q['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE))
    elif ch in ('OBJECT_STATE', 'OBJECT_STATE_SIMULATION'):
      obj.append((t, *dairlib.lcmt_object_state.decode(event.data).position[:7]))
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      clean.append(
          (t, *dairlib.lcmt_object_state.decode(event.data).position[:7]))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      defl.append((t, *msg.position[:2]))
  if not loops:
    raise click.ClickException(f'{path} has no SAMPLING_C3_DEBUG')
  return (np.array(loops, dtype=float), np.array(ee), np.array(obj),
          np.array(clean) if clean else None, np.array(defl) if defl else None)


def rotation(quat_wxyz):
  return R.from_quat(np.r_[quat_wxyz[1:4], quat_wxyz[0]]).as_matrix()


def axis_z(quat_wxyz):
  """World z of the cone's symmetry axis (body +x)."""
  return rotation(quat_wxyz)[2, 0]


def rebuild_load(loops, ee, obj, clear_gap, loop_offset):
  t = loops[:, 0] - loop_offset
  latched = loops[:, 1] > 0
  ee_at = np.stack([np.interp(t, ee[:, 0], ee[:, i]) for i in (1, 2, 3)], 1)
  latest = np.clip(np.searchsorted(obj[:, 0], t, side='right') - 1, 0,
                   len(obj) - 1)
  estimator = FingerLoadEstimator(clear_gap)
  load = np.full(len(t), np.nan)
  for i in range(len(t)):
    o = obj[latest[i]]
    rot, pos = rotation(o[1:5]), o[5:8]
    distance, normal_o = cone_query(rot.T @ (ee_at[i] - pos))
    load[i] = estimator.update(ee_at[i], distance - EE_RADIUS, rot @ normal_o,
                               rot, pos, latched[i])
  return load


def first_crossings(t, armed, hold):
  """Loop indices where `armed` has held for `hold` s, once per episode."""
  out, since, fired = [], None, False
  for i in range(len(t)):
    if armed[i]:
      since = t[i] if since is None else since
      if not fired and t[i] - since >= hold - 1e-9:
        out.append(i)
        fired = True
    else:
      since, fired = None, False
  return out


def releases(defl, clean, peak_min):
  """Finger releases that moved the cone, from the sim's ground truth."""
  t = defl[:, 0]
  bend = np.hypot(defl[:, 1], defl[:, 2])
  out, i = [], 0
  while i < len(t):
    if bend[i] < peak_min:
      i += 1
      continue
    j = i
    while (j + 1 < len(t) and bend[j + 1] >= 0.3 * bend[i:j + 1].max() and
           t[j + 1] - t[i] < 30):
      j += 1
    peak = i + int(np.argmax(bend[i:j + 1]))
    released = min(j + 1, len(t) - 1)
    onset = i
    while onset > 0 and bend[onset] >= 0.003:
      onset -= 1
    span = np.array([t[released] - 0.2, t[released] + 1.0])
    pose = np.stack([np.interp(span, clean[:, 0], clean[:, k])
                     for k in range(1, 8)], 1)
    moved = np.linalg.norm(pose[1, 4:7] - pose[0, 4:7])
    tilted = abs(axis_z(pose[1, :4]) - axis_z(pose[0, :4]))
    if moved >= 0.020 or tilted >= 0.5:
      out.append(dict(onset=t[onset], peak_t=t[peak], peak=bend[peak],
                      released=t[released]))
    i = released + 1
  return out


def tips(clean):
  t = clean[::20, 0]
  z = np.array([axis_z(q) for q in clean[::20, 1:5]])
  out, i = [], 0
  while i < len(t):
    if z[i] > 0.9:
      end = np.searchsorted(t, t[i] + 3.0)
      down = [k for k in range(i, min(end, len(t))) if z[k] < 0.4]
      if down:
        k = down[0]
        settle = np.searchsorted(t, t[k] + 3.0)
        if settle < len(t) and z[k:settle].max() < 0.6:
          out.append(t[k])
          i = settle
          continue
    i += 1
  return out


def score(path, load_trip, hold, clear_gap, loop_offset, peak_min):
  loops, ee, obj, clean, defl = read_log(path)
  t = loops[:, 0]
  latched = loops[:, 1] > 0
  rebuilt = rebuild_load(loops, ee, obj, clear_gap, loop_offset)
  logged = loops[:, 3]
  has_logged = np.isfinite(logged).any()

  # A live tier's own trips are the rising edges it set; otherwise, what the
  # rebuilt estimate would have set on loops the logged guard left unlatched.
  if has_logged:
    rising = np.flatnonzero(latched[1:] & ~latched[:-1]) + 1
    fired = [i for i in rising if loops[i, 2] > 0]
    source = 'logged'
  else:
    fired = first_crossings(t, (rebuilt >= load_trip) & ~latched, hold)
    source = 'rebuilt'
  result = dict(source=source, fired=fired, t=t)
  if has_logged:
    both = np.isfinite(logged) & np.isfinite(rebuilt)
    result['agreement_mm'] = 1e3 * np.abs(logged[both] - rebuilt[both])
  if defl is None or clean is None:
    return result  # hardware: no ground truth

  bend_t = defl[:, 0]
  bend = np.hypot(defl[:, 1], defl[:, 2])
  true_at = np.interp(t, bend_t, bend)
  rising = np.flatnonzero(latched[1:] & ~latched[:-1]) + 1
  events, used = [], set()
  for e in releases(defl, clean, peak_min):
    guarded = any(e['released'] - 1.5 <= t[i] <= e['released'] for i in rising)
    hit = [i for i in fired if e['onset'] - 0.5 <= t[i] <= e['peak_t']]
    if hit:
      used.add(hit[0])
    events.append(dict(e, guarded=guarded, caught=hit[0] if hit else None))
  no_load, loaded = [], []
  for i in fired:
    if i in used:
      continue
    window = (bend_t >= t[i] - 0.3) & (bend_t <= t[i] + 1.0)
    (no_load if bend[window].max() < 0.008 else loaded).append(i)
  tip_rows = []
  for tip in tips(clean):
    before = [i for i in fired if tip - 3.0 <= t[i] <= tip]
    tip_rows.append((tip, bool(before)))
  result.update(events=events, no_load=no_load, loaded=loaded, tips=tip_rows,
                true_at=true_at)
  return result


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--load_trip', default=0.020, show_default=True,
              help='Load tier trip [m] for the rebuilt estimate.')
@click.option('--hold', default=0.0, show_default=True,
              help='Load tier dwell [s] for the rebuilt estimate.')
@click.option('--clear_gap', default=0.005, show_default=True,
              help='Re-anchor at or above this gap [m] on either side.')
@click.option('--loop_offset', default=0.09, show_default=True,
              help='How long before each debug message the controller read '
                   'the reported EE and object pose [s].')
@click.option('--peak_mm', default=15.0, show_default=True,
              help='Smallest finger bend counted as a release [mm].')
def main(logs, load_trip, hold, clear_gap, loop_offset, peak_mm):
  totals = dict(releases=0, missed=0, caught=0, no_load=0, loaded=0, tips=0,
                tips_fired=0)
  bends, leads = [], []
  for folder in logs:
    path = find_log(folder)
    r = score(path, load_trip, hold, clear_gap, loop_offset, peak_mm / 1e3)
    name = op.basename(path)
    t = r['t']
    print(f'== {name}  ({r["source"]} load tier, {len(r["fired"])} trips)')
    if 'agreement_mm' in r and len(r['agreement_mm']):
      a = r['agreement_mm']
      print(f'   rebuilt vs logged load: |diff| p50 {np.median(a):.1f} mm, '
            f'p90 {np.percentile(a, 90):.1f} mm')
    if 'events' not in r:
      print('   load-tier trips at ' +
            ', '.join(f'{t[i]:.1f}' for i in r['fired']) + ' s')
      continue
    for e in r['events']:
      i = e['caught']
      what = (f'caught at {t[i]:.2f} s, true bend {1e3 * r["true_at"][i]:.0f}'
              f' mm, {e["released"] - t[i]:.2f} s before release'
              if i is not None else 'NOT caught')
      print(f'   release {e["released"]:7.2f} s  peak {1e3 * e["peak"]:3.0f} mm'
            f'  logged guard {"tripped" if e["guarded"] else "missed "}  '
            f'load tier {what}')
      totals['releases'] += 1
      totals['missed'] += not e['guarded']
      if i is not None:
        totals['caught'] += 1
        bends.append(1e3 * r['true_at'][i])
        leads.append(e['released'] - t[i])
    if r['no_load']:
      print('   no-load trips at ' +
            ', '.join(f'{t[i]:.1f}' for i in r['no_load']) + ' s')
    totals['no_load'] += len(r['no_load'])
    totals['loaded'] += len(r['loaded'])
    totals['tips'] += len(r['tips'])
    totals['tips_fired'] += sum(f for _, f in r['tips'])
    for tip, f in r['tips']:
      print(f'   tip at {tip:.1f} s{", load tier fired in the 3 s before" if f else ""}')
  if totals['releases']:
    print(f'\nTOTAL: {totals["releases"]} releases, {totals["missed"]} missed '
          f'by the logged guard; load tier caught {totals["caught"]} '
          f'(true bend at trip p50 {np.median(bends) if bends else np.nan:.0f}'
          f' mm, lead p10 {np.percentile(leads, 10) if leads else np.nan:.2f}'
          f' s); {totals["no_load"]} no-load trips, {totals["loaded"]} other '
          f'loaded trips; fired before {totals["tips_fired"]} of '
          f'{totals["tips"]} tips.')


if __name__ == '__main__':
  main()
