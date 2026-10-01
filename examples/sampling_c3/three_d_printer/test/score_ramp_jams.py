"""Scores the compliant finger catching on the ramp in cone demo sim logs.

C3 does not model the EE against the ramp (num_contacts_index's EE-ground
group, which holds the EE-ramp pairs, is 0), so its raw plans put the EE
inside the ramp's walls, and only the published plan's knots were projected
off the ramp.  Neighbouring knots inside a wall landed on its opposite faces,
the printer drove straight through the wall between them, and the sim's
compliant finger caught on it and bent, then snapped free.  This scores how
often that happens and why, from the sim's ground truth:

  episodes:  EE-ramp contact (CONTACT_RESULTS, end_effector_tip/_peg against
             ramp_link, merged across gaps < 0.3 s) that bent the finger
             (FINGER_DEFLECTION_SIMULATION) >= 10 mm, each with its cause:
               knot_inside    a published knot within 2 mm of the ramp's
                              surface or inside it (EE surface gap < -2 mm)
               segment_cut    every knot clear, but the straight path between
                              two of them, or from the gantry to the plan,
                              runs > 4 mm into the ramp
               gantry_only    the plan's path is clear and the gantry still
                              went > 4 mm in (command lag)
               gantry_clear   the gantry never went in (the cone pushing the
                              finger, a hook from an earlier episode)
             The plans looked at are the ones published on
             TRACKING_TRAJECTORY_ACTOR in the 1.2 s before the episode began.
  launches:  the cone moving >= 50 mm within 1 s of a ramp episode ending.
  path cuts: the share of published plans near the ramp (a knot within 40 mm)
             whose path runs into it, by controller mode.
  holds:     the controller's "[fixed geometry path]" lines, when the run's
             SC3 output was saved next to the log as sc3_stdout.txt.

--replay_path_check MARGINS replays ClearEEPlanOfFixedGeometries
(examples/sampling_c3/reposition.h) on the raw C3 plans
(C3_TRAJECTORY_ACTOR_CURR_PLAN with knot 0 pinned to C3_ACTUAL's EE) of C3 loops
near the ramp, at each knot margin, and reports how often the plan would have
been held short of the ramp.  The replay has no closed loop, so it counts the
logged plans, not the ones the controller would have made instead.

--replay_ramp_tier replays a FingerLoadEstimator (the port in
score_finger_load_guard.py) against the ramp: identity pose, the reported EE's
distance and gradient to the ramp, a trip at --ramp_load_trip on unlatched
loops.  An episode >= 20 mm is caught when the tier fires between 0.5 s before
the bend began and its peak; a trip is "no bend" when the true bend stays under
8 mm from 0.3 s before to 1 s after it.

The ramp is the union of new_ramp.urdf's 14 collision meshes, each taken as its
convex hull (the URDF declares them convex), welded where
sampling_c3_utils.h puts it.  Distances are to the EE sphere's centre; the
printer controller's EE sphere is 10 mm.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_ramp_jams.py \\
      ~/3d_printer/logs/2026/10_01_26/0000{12..17} --episodes
"""

import glob
import multiprocessing
import os.path as op
import re
import sys
from collections import Counter

import click
import numpy as np
from lcm import EventLog
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'external', 'drake+',
                        'lcmtypes'))
sys.path.append(op.dirname(__file__))
import dairlib  # noqa: E402
import drake  # noqa: E402
from score_finger_load_guard import FingerLoadEstimator, decode_debug  # noqa: E402,E501

RAMP_DIR = op.join(DAIRLIB_DIR, 'examples', 'sampling_c3', 'urdf',
                   'three_d_printer', 'ramp')
# k3dPrinterRampAttachmentFrame / ...RotationMatrix (sampling_c3_utils.h).
RAMP_WELD_XYZ = np.array([0.0437, 0.0400, 0.0])
RAMP_WELD_YAW = 3.14159
EE_RADIUS = 0.010
PRINTER_Z_TO_EE_CENTRE = 0.110864
# The cone demo's workspace (cone/parameters/sampling_c3plus_options.yaml).
WORKSPACE_LO = np.array([0.0, 0.0, 0.015])
WORKSPACE_HI = np.array([0.35, 0.35, 0.248])
WORKSPACE_MARGIN = 0.002
NEAR_RAMP = 0.040


class Ramp:
  """Signed distance from points to the ramp, with the outward gradient."""

  def __init__(self):
    urdf = open(op.join(RAMP_DIR, 'new_ramp.urdf')).read()
    pieces = re.findall(
        r'<collision name="(\w+)">\s*<origin xyz="([^"]+)" rpy="([^"]+)"/>'
        r'\s*<geometry>\s*<mesh filename="([^"]+)" scale="([^"]+)"', urdf)
    weld = R.from_euler('z', RAMP_WELD_YAW).as_matrix()
    self.names, self.hulls = [], []
    for name, xyz, rpy, mesh, scale in pieces:
      v = np.array([[float(a) for a in line.split()[1:4]]
                    for line in open(op.join(RAMP_DIR, mesh))
                    if line.startswith('v ')])
      v = v * float(scale.split()[0])
      v = v @ R.from_euler('xyz', [float(a) for a in rpy.split()]).as_matrix().T
      v = (v + np.array([float(a) for a in xyz.split()])) @ weld.T
      v = v + RAMP_WELD_XYZ
      hull = ConvexHull(v)
      self.names.append(name.replace('ramp_part_', 'p'))
      self.hulls.append((v[hull.simplices], hull.equations))

  def query(self, points):
    """(distance [n], outward unit gradient [n, 3], piece index [n])."""
    points = np.atleast_2d(points)
    best = np.full(len(points), np.inf)
    grad = np.zeros((len(points), 3))
    piece = np.zeros(len(points), dtype=int)
    for j, (triangles, planes) in enumerate(self.hulls):
      d, g = _hull_query(points, triangles, planes)
      better = d < best
      best[better], grad[better], piece[better] = d[better], g[better], j
    return best, grad, piece

  def distance(self, points):
    return self.query(points)[0]


def _closest_on_triangles(p, tri):
  """Closest points on each triangle (m, 3, 3) to each point (n, 3)."""
  a, b, c = tri[None, :, 0], tri[None, :, 1], tri[None, :, 2]
  p = p[:, None, :]
  ab, ac, ap = b - a, c - a, p - a
  d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
  bp, cp = p - b, p - c
  d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
  d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
  va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2

  def safe(x):
    return np.where(np.abs(x) < 1e-30, 1e-30, x)

  denom = safe(va + vb + vc)
  out = a + ab * (vb / denom)[..., None] + ac * (vc / denom)[..., None]
  done = np.zeros(d1.shape, dtype=bool)
  for mask, value in (
      ((d1 <= 0) & (d2 <= 0), a),
      ((d3 >= 0) & (d4 <= d3), b),
      ((d6 >= 0) & (d5 <= d6), c),
      ((vc <= 0) & (d1 >= 0) & (d3 <= 0),
       a + ab * (d1 / safe(d1 - d3))[..., None]),
      ((vb <= 0) & (d2 >= 0) & (d6 <= 0),
       a + ac * (d2 / safe(d2 - d6))[..., None]),
      ((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0),
       b + (c - b) * ((d4 - d3) / safe((d4 - d3) + (d5 - d6)))[..., None])):
    mask = mask & ~done
    out = np.where(mask[..., None], value, out)
    done |= mask
  return out


def _hull_query(points, triangles, planes):
  closest = _closest_on_triangles(points, triangles)
  offsets = points[:, None, :] - closest
  squared = (offsets ** 2).sum(-1)
  k = squared.argmin(1)
  rows = np.arange(len(points))
  outside_d = np.sqrt(squared[rows, k])
  outside_g = offsets[rows, k] / np.maximum(outside_d, 1e-12)[:, None]
  # Inside a convex hull, the nearest face plane is the nearest surface.
  plane_d = points @ planes[:, :3].T + planes[:, 3]
  face = plane_d.argmax(1)
  inside = plane_d[rows, face] <= 0
  d = np.where(inside, plane_d[rows, face], outside_d)
  g = np.where(inside[:, None], planes[face, :3], outside_g)
  return d, g


def find_log(path):
  if op.isfile(path):
    return path
  found = sorted(glob.glob(op.join(path, '*log-*')))
  if not found:
    raise click.ClickException(f'no log in {path}')
  return found[0]


def ee_plan(msg):
  saved = msg.saved_traj
  for name in ('end_effector_position_target', 'ee_position_target'):
    if name in saved.trajectory_names:
      block = saved.trajectories[saved.trajectory_names.index(name)]
      return np.array(block.datapoints)[:3].T
  return None


def read_log(path):
  gantry, defl, contacts, loops, plans, raw, x0, clean = ([] for _ in range(8))
  t0 = None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'PRINTER_STATE_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      gantry.append((t, q['x_axis_joint'], q['y_axis_joint'],
                     q['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE))
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      defl.append((t, *msg.position[:2]))
    elif ch == 'CONTACT_RESULTS':
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      ramp = cone = 0.0
      for pair in msg.point_pair_contact_info:
        bodies = pair.body1_name + '|' + pair.body2_name
        if 'end_effector' not in bodies:
          continue
        force = np.linalg.norm(pair.contact_force)
        if 'ramp_link' in bodies:
          ramp = max(ramp, force)
        elif 'cone' in bodies:
          cone = max(cone, force)
      contacts.append((t, ramp, cone))
    elif ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      loops.append((t, msg.is_c3_mode, msg.jam_tripped))
    elif ch == 'TRACKING_TRAJECTORY_ACTOR':
      plans.append((t, ee_plan(dairlib.lcmt_timestamped_saved_traj.decode(
          event.data))))
    elif ch == 'C3_TRAJECTORY_ACTOR_CURR_PLAN':
      raw.append((t, ee_plan(dairlib.lcmt_timestamped_saved_traj.decode(
          event.data))))
    elif ch == 'C3_ACTUAL':
      msg = dairlib.lcmt_c3_state.decode(event.data)
      x0.append((t, *msg.state[:3]))
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      clean.append((t, *dairlib.lcmt_object_state.decode(
          event.data).position[4:7]))
  if not contacts or not defl:
    raise click.ClickException(
        f'{path} has no CONTACT_RESULTS or FINGER_DEFLECTION_SIMULATION: '
        'this scores compliant-finger sim logs only')
  return dict(gantry=np.array(gantry), defl=np.array(defl),
              contacts=np.array(contacts), loops=np.array(loops, dtype=float),
              plans=plans, raw=raw, x0=np.array(x0), clean=np.array(clean))


def path_distance(ramp, knots, samples=12):
  """Smallest distance along the straight segments through knots (k, 3)."""
  s = np.linspace(0, 1, samples)
  points = np.concatenate([knots[i] + np.outer(s, knots[i + 1] - knots[i])
                           for i in range(len(knots) - 1)])
  return ramp.distance(points).min()


def ramp_episodes(contacts, gap=0.3, min_force=0.5):
  t, on = contacts[:, 0], contacts[:, 1] > min_force
  out, i = [], 0
  while i < len(t):
    if not on[i]:
      i += 1
      continue
    j = i
    while True:
      k = j + 1
      while k < len(t) and not on[k] and t[k] - t[j] < gap:
        k += 1
      if k < len(t) and on[k]:
        j = k
      else:
        break
    out.append((t[i], t[j]))
    i = j + 1
  return out


def classify(ep):
  if ep['knot'] < -0.002:
    return 'knot_inside'
  if min(ep['segment'], ep['gantry_to_plan']) < -0.004:
    return 'segment_cut'
  if ep['gantry_min'] < -0.004:
    return 'gantry_only'
  return 'gantry_clear'


def score_episodes(ramp, log, min_bend=0.010):
  g, defl, contacts, loops = (log['gantry'], log['defl'], log['contacts'],
                              log['loops'])
  bend = np.hypot(defl[:, 1], defl[:, 2])
  gantry_gap = ramp.distance(g[:, 1:4]) - EE_RADIUS
  plan_t = np.array([p[0] for p in log['plans']])
  jam_rise = loops[1:, 0][(loops[1:, 2] > 0) & (loops[:-1, 2] == 0)]
  episodes = []
  for start, end in ramp_episodes(contacts):
    window = (defl[:, 0] >= start) & (defl[:, 0] <= end + 0.1)
    if not window.any() or bend[window].max() < min_bend:
      continue
    peak_i = np.flatnonzero(window)[bend[window].argmax()]
    peak_t = defl[peak_i, 0]
    onset = peak_i
    while onset > 0 and bend[onset] >= 0.003 and defl[onset, 0] > start - 1:
      onset -= 1
    in_ep = (g[:, 0] >= start - 0.2) & (g[:, 0] <= end)
    cone_window = (contacts[:, 0] >= start) & (contacts[:, 0] <= end)
    loop_i = max(np.searchsorted(loops[:, 0], start) - 1, 0)
    knot = segment = gantry_to_plan = np.inf
    for i in np.flatnonzero((plan_t >= start - 1.2) & (plan_t <= start)):
      t, knots = log['plans'][i]
      if knots is None:
        continue
      gi = max(np.searchsorted(g[:, 0], t) - 1, 0)
      knot = min(knot, ramp.distance(knots).min())
      segment = min(segment, path_distance(ramp, knots))
      gantry_to_plan = min(gantry_to_plan, path_distance(
          ramp, np.vstack([g[gi, 1:4], knots[:2]])))
    after = log['clean'][(log['clean'][:, 0] >= end) &
                         (log['clean'][:, 0] <= end + 1.0)]
    moved = (np.linalg.norm(after[:, 1:4] - after[0, 1:4], axis=1).max()
             if len(after) else 0.0)
    gi = max(np.searchsorted(g[:, 0], peak_t) - 1, 0)
    ep = dict(start=start, end=end, onset=defl[onset, 0], peak_t=peak_t,
              peak=bend[peak_i], c3=bool(loops[loop_i, 1]),
              latched=bool(loops[loop_i, 2]),
              cone=float((contacts[cone_window, 2] > 0.5).mean()),
              gantry_min=(gantry_gap[in_ep].min() if in_ep.any() else np.nan),
              gantry_at_peak=gantry_gap[gi],
              knot=knot - EE_RADIUS, segment=segment - EE_RADIUS,
              gantry_to_plan=gantry_to_plan - EE_RADIUS,
              piece=ramp.names[ramp.query(g[gi, 1:4])[2][0]],
              tripped=bool(((jam_rise >= defl[onset, 0] - 0.5) &
                            (jam_rise <= peak_t)).any()),
              launch=moved >= 0.050)
    ep['cause'] = classify(ep)
    episodes.append(ep)
  return episodes


def path_cut_rates(ramp, log):
  """{mode: [plans near the ramp, plans whose path runs into it]}."""
  loops, rates = log['loops'], {}
  for t, knots in log['plans']:
    if knots is None:
      continue
    i = max(np.searchsorted(loops[:, 0], t) - 1, 0)
    mode = ('c3' if loops[i, 1] else 'repos') + ('+jam' if loops[i, 2] else '')
    if ramp.distance(knots).min() > NEAR_RAMP:
      continue
    near, cut = rates.setdefault(mode, [0, 0])
    rates[mode] = [near + 1, cut + (path_distance(ramp, knots) < EE_RADIUS)]
  return rates


def hold_streaks(path):
  """(start, duration) of each logged hold, from the saved SC3 output."""
  stdout = op.join(op.dirname(path), 'sc3_stdout.txt')
  if not op.exists(stdout):
    return None
  streaks, reroutes = [], 0
  for line in open(stdout, errors='replace'):
    if not line.startswith('[fixed geometry path]'):
      continue
    m = re.search(r't=([\d.]+) .*hold released after ([\d.e-]+) s', line)
    if m:
      streaks.append((float(m.group(1)) - float(m.group(2)),
                      float(m.group(2))))
    elif 'rerouted' in line:
      reroutes += 1
  return streaks, reroutes


def clamp_to_workspace(p):
  return np.clip(p, WORKSPACE_LO + WORKSPACE_MARGIN,
                 WORKSPACE_HI - WORKSPACE_MARGIN)


def clear_plan(ramp, knots, knot_clearance, path_clearance):
  """Port of ClearEEPlanOfFixedGeometries (reposition.cc) with no exempt
  knots; returns (cleared knots, first blocked knot or -1)."""
  knots = knots.copy()
  for k in range(len(knots)):
    p = clamp_to_workspace(knots[k])
    for _ in range(8):
      d, g, _ = ramp.query(p)
      if d[0] >= knot_clearance:
        break
      p = clamp_to_workspace(p + (knot_clearance - d[0]) * g[0])
    knots[k] = p
  d = ramp.distance(knots[0])[0]
  for k in range(1, len(knots)):
    a, b = knots[k - 1], knots[k]
    length, s = np.linalg.norm(b - a), 0.0
    while True:
      s += max(d - path_clearance, 0.0005)
      if s >= length:
        d = ramp.distance(b)[0]
        if d < path_clearance:
          return knots, k
        break
      d = ramp.distance(a + s / length * (b - a))[0]
      if d < path_clearance:
        return knots, k
  return knots, -1


def path_check_holds(ramp, log, margins):
  """{margin: (C3 loops near the ramp, held, held from knot <= 2)}."""
  loops, x0 = log['loops'], log['x0']
  out = {m: [0, 0, 0] for m in margins}
  for t, knots in log['raw']:
    if knots is None or len(x0) == 0:
      continue
    i = max(np.searchsorted(loops[:, 0], t) - 1, 0)
    if not loops[i, 1]:
      continue
    knots = knots.copy()
    knots[0] = x0[max(np.searchsorted(x0[:, 0], t) - 1, 0), 1:4]
    if ramp.distance(knots).min() > NEAR_RAMP:
      continue
    for m in margins:
      _, blocked = clear_plan(ramp, knots, EE_RADIUS + m,
                              EE_RADIUS + WORKSPACE_MARGIN)
      out[m][0] += 1
      out[m][1] += blocked >= 0
      out[m][2] += 0 <= blocked <= 2
  return out


def ramp_tier_trips(ramp, log, episodes, trips, clear_gaps, loop_offset):
  """{(trip, clear_gap): (caught of eligible, no-bend trips, trips)}."""
  loops, g, defl = log['loops'], log['gantry'], log['defl']
  t = loops[:, 0] - loop_offset
  latched = loops[:, 2] > 0
  ee = np.stack([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)], 1)
  d, grad, _ = ramp.query(ee)
  bend_t, bend = defl[:, 0], np.hypot(defl[:, 1], defl[:, 2])
  eligible = [e for e in episodes if e['peak'] >= 0.020]
  out = {}
  for clear_gap in clear_gaps:
    estimator = FingerLoadEstimator(clear_gap)
    load = np.array([
        estimator.update(ee[i], d[i] - EE_RADIUS, grad[i], np.eye(3),
                         np.zeros(3), latched[i]) for i in range(len(t))])
    for trip in trips:
      armed = (np.nan_to_num(load) >= trip) & ~latched
      fired = np.flatnonzero(armed[1:] & ~armed[:-1]) + 1
      caught = sum(any(e['onset'] - 0.5 <= loops[i, 0] <= e['peak_t']
                       for i in fired) for e in eligible)
      no_bend = 0
      for i in fired:
        w = (bend_t >= loops[i, 0] - 0.3) & (bend_t <= loops[i, 0] + 1.0)
        no_bend += bend[w].max() < 0.008 if w.any() else 1
      out[(trip, clear_gap)] = (caught, len(eligible), no_bend, len(fired))
  return out


def score_log(arg, show_episodes, margins, tier_on, trips, gaps,
              loop_offset):
  """Everything main() reports for one log: (text, episodes, minutes,
  path cuts, hold streaks or None, reroutes, path-check holds, tier trips)."""
  ramp = Ramp()
  path = find_log(arg)
  log = read_log(path)
  duration = (log['loops'][-1, 0] - log['loops'][0, 0]) / 60
  eps = score_episodes(ramp, log)
  held = hold_streaks(path)
  big = [e for e in eps if e['peak'] >= 0.020]
  lines = [f'== {op.relpath(path, op.expanduser("~/3d_printer/logs/2026"))}: '
           f'{duration:.1f} min, {len(eps)} ramp bends >= 10 mm, '
           f'{len(big)} >= 20 mm, '
           f'{sum(e["peak"] >= 0.040 for e in eps)} >= 40 mm, '
           f'{sum(e["launch"] for e in eps)} launches'
           + ('' if held is None else
              f', {len(held[0])} holds, {held[1]} reroutes')]
  if show_episodes:
    for e in eps:
      lines.append(
          f'  {e["start"]:7.1f} s {e["end"] - e["start"]:4.1f} s  '
          f'peak {1e3 * e["peak"]:3.0f} mm  '
          f'{"c3" if e["c3"] else "repos"}{"+jam" if e["latched"] else ""}'
          f'  cone {e["cone"]:.2f}  {e["cause"]:12s} '
          f'gantry min/peak {1e3 * e["gantry_min"]:6.1f}/'
          f'{1e3 * e["gantry_at_peak"]:5.1f} mm  '
          f'plan knot/seg/from gantry {1e3 * e["knot"]:6.1f}/'
          f'{1e3 * e["segment"]:6.1f}/{1e3 * e["gantry_to_plan"]:6.1f} mm'
          f'  {e["piece"]}{"  TRIPPED" if e["tripped"] else ""}'
          f'{"  LAUNCH" if e["launch"] else ""}')
  return ('\n'.join(lines), eps, duration, path_cut_rates(ramp, log),
          None if held is None else held[0],
          0 if held is None else held[1],
          path_check_holds(ramp, log, margins) if margins else {},
          ramp_tier_trips(ramp, log, eps, trips, gaps, loop_offset)
          if tier_on else {})


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--episodes', 'show_episodes', is_flag=True,
              help='Print every episode, not just the totals.')
@click.option('--replay_path_check', default='',
              help='Comma-separated knot margins [m] to replay the path '
                   'check at, e.g. 0.002,0.004,0.005.')
@click.option('--replay_ramp_tier', is_flag=True,
              help='Replay a ramp FingerLoadEstimator trip.')
@click.option('--ramp_load_trip', default='0.008,0.010,0.015,0.020',
              help='Comma-separated trip loads [m] for --replay_ramp_tier.')
@click.option('--clear_gap', default='0.005',
              help='Comma-separated re-anchor gaps [m] for --replay_ramp_tier.')
@click.option('--loop_offset', default=0.09,
              help='How long before each SAMPLING_C3_DEBUG the controller '
                   'read the EE [s].')
@click.option('--jobs', default=1,
              help='Logs to score in parallel (the replays take minutes per '
                   'log).')
def main(logs, show_episodes, replay_path_check, replay_ramp_tier,
         ramp_load_trip, clear_gap, loop_offset, jobs):
  margins = [float(m) for m in replay_path_check.split(',') if m]
  trips = [float(x) for x in ramp_load_trip.split(',') if x]
  gaps = [float(x) for x in clear_gap.split(',') if x]
  args = [(arg, show_episodes, margins, replay_ramp_tier, trips, gaps,
           loop_offset) for arg in logs]
  if jobs > 1:
    with multiprocessing.Pool(jobs) as pool:
      results = pool.starmap(score_log, args)
  else:
    results = [score_log(*a) for a in args]
  all_eps, minutes, cuts, replay, tier = [], 0.0, {}, {}, {}
  all_streaks, all_reroutes, have_stdout = [], 0, False
  for text, eps, duration, log_cuts, streaks, reroutes, holds, trips_out in (
      results):
    print(text)
    all_eps += eps
    minutes += duration
    for mode, (near, cut) in log_cuts.items():
      total = cuts.setdefault(mode, [0, 0])
      cuts[mode] = [total[0] + near, total[1] + cut]
    if streaks is not None:
      have_stdout = True
      all_streaks += streaks
      all_reroutes += reroutes
    for m, counts in holds.items():
      total = replay.setdefault(m, [0, 0, 0])
      replay[m] = [a + b for a, b in zip(total, counts)]
    for key, counts in trips_out.items():
      total = tier.setdefault(key, [0, 0, 0, 0])
      tier[key] = [a + b for a, b in zip(total, counts)]

  big = [e for e in all_eps if e['peak'] >= 0.020]
  print(f'\nTotal {minutes:.1f} min over {len(logs)} logs')
  print(f'  ramp bends >= 20 mm: {len(big)} ({len(big) / minutes:.2f}/min), '
        f'>= 40 mm: {sum(e["peak"] >= 0.040 for e in all_eps)}, '
        f'launches: {sum(e["launch"] for e in all_eps)}, '
        f'tripped before the peak: {sum(e["tripped"] for e in big)}')
  print(f'  time in ramp bends >= 10 mm: '
        f'{sum(e["end"] - e["start"] for e in all_eps):.0f} s')
  print('  causes (>= 20 mm): ' + ', '.join(
      f'{k} {v}' for k, v in Counter(e['cause'] for e in big).most_common()))
  print('  published plans near the ramp whose path runs into it: ' + ', '.join(
      f'{mode} {cut}/{near} ({100 * cut / max(near, 1):.1f}%)'
      for mode, (near, cut) in sorted(cuts.items())))
  if have_stdout:
    durations = np.array([d for _, d in all_streaks])
    print(f'  holds: {len(all_streaks)}, ' + (
        f'median {np.median(durations):.2f} s, p90 '
        f'{np.percentile(durations, 90):.2f} s, max {durations.max():.1f} s'
        if len(durations) else 'none') + f'; reroutes: {all_reroutes}')
  for m, (near, held, early) in sorted(replay.items()):
    print(f'  replayed path check, knot margin {1e3 * m:.0f} mm: held '
          f'{held}/{near} C3 plans near the ramp ({100 * held / max(near, 1):.1f}%'
          f'), {early} from knot <= 2')
  for (trip, gap), (caught, eligible, no_bend, fired) in sorted(tier.items()):
    print(f'  ramp tier at {1e3 * trip:.0f} mm, clear gap {1e3 * gap:.0f} mm: '
          f'caught {caught}/{eligible} bends >= 20 mm, {fired} trips, '
          f'{no_bend} with no bend ({no_bend / minutes:.2f}/min)')


if __name__ == '__main__':
  main()
