"""Scores repositioning stalls in cone demo logs.

A repositioning target closer to the ramp than the plans' knots may go
(fixed_geometry_knot_margin, 4 mm of EE surface clearance) is never reached:
the path check projects the plan's last knots out, the EE parks short of the
target, and Reposition() never reports it finished, because that needs the
target within one knot (75 ms) of travel and the z axis covers only 1.1 mm in
that time.  Before the fix, samples were accepted 2 mm off the ramp, and the
2026-10-01 compliant sims 20, 25 and 26 repositioned for 46-129 s this way.

Per log this reports:
  repos:       time in repositioning mode, and the longest repositioning
               stretch with what started and ended it and how much of it the
               jam latch was set
  parked:      episodes >= 0.5 s in which the EE sat within 3 mm of a target
               closer to the ramp than the knot clearance, while the EE
               itself was further out (where the projection left it)
  blocked:     episodes >= 0.5 s in which the target had been reached (its
               cost carries finished_reposition_cost) but the controller
               stayed in repositioning.  The switch to C3 also needs the
               current location to be clear of the unsuccessful sample
               buffer, and an entry recorded after the target was chosen
               (a jam trip nearby, say) vetoes it until the object moves
               away from where the entry was recorded
  in clearance: the share of repositioning loops whose target is closer to
               the ramp than the knot clearance

Works on sim and hardware logs (it needs only SAMPLING_C3_DEBUG, C3_ACTUAL,
SAMPLE_LOCATIONS and SAMPLE_COSTS).  Distances use score_ramp_jams.Ramp, the union of the
ramp's convex collision hulls; the target's index in SAMPLE_LOCATIONS is
SampleIndex::kCurrentReposTarget.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_repos_stalls.py \\
      ~/3d_printer/logs/2026/10_01_26/0000{20,25,26}
"""

import multiprocessing
import os.path as op
import sys

import click
import numpy as np
from lcm import EventLog

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.dirname(__file__))
import dairlib  # noqa: E402
from score_finger_load_guard import decode_debug  # noqa: E402
from score_ramp_jams import EE_RADIUS, Ramp, find_log  # noqa: E402

CURRENT_REPOS_TARGET_INDEX = 1
MODE_SWITCH_REASON = {
    0: 'none', 1: 'C3: cost', 2: 'C3: reached target', 3: 'repos: cost',
    4: 'repos: unproductive', 5: 'C3: xbox', 6: 'repos: jam',
    7: 'repos: goal changed',
}
PARKED_RADIUS = 0.003   # m, EE centre to target
# Half of progress_params' finished_reposition_cost (1e9): a target cost above
# this was reached on the previous loop.
REACHED_COST = 5e8
MIN_EPISODE = 0.5       # s


def traj_block(msg, name):
  saved = msg.saved_traj
  block = saved.trajectories[saved.trajectory_names.index(name)]
  return np.array(block.datapoints)


def read_log(path):
  """One row per control loop that published all three channels."""
  loops, t0 = {}, None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    ch = event.channel
    if ch == 'SAMPLING_C3_DEBUG':
      msg = decode_debug(event.data)
      loops.setdefault(msg.utime, {})['debug'] = (
          (event.timestamp - t0) / 1e6, msg)
    elif ch == 'SAMPLE_LOCATIONS':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      locations = traj_block(msg, 'sample_locations')
      if locations.shape[1] > CURRENT_REPOS_TARGET_INDEX:
        loops.setdefault(msg.utime, {})['target'] = \
            locations[:3, CURRENT_REPOS_TARGET_INDEX]
    elif ch == 'SAMPLE_COSTS':
      msg = dairlib.lcmt_timestamped_saved_traj.decode(event.data)
      costs = traj_block(msg, 'sample_costs')[0]
      if len(costs) > CURRENT_REPOS_TARGET_INDEX:
        loops.setdefault(msg.utime, {})['target_cost'] = \
            costs[CURRENT_REPOS_TARGET_INDEX]
    elif ch == 'C3_ACTUAL':
      msg = dairlib.lcmt_c3_state.decode(event.data)
      loops.setdefault(msg.utime, {})['x0'] = np.array(msg.state[:3])
  rows = [v for _, v in sorted(loops.items())
          if all(k in v for k in ('debug', 'target', 'x0'))]
  return dict(
      t=np.array([r['debug'][0] for r in rows]),
      c3=np.array([bool(r['debug'][1].is_c3_mode) for r in rows]),
      reason=np.array([r['debug'][1].mode_switch_reason for r in rows]),
      jam=np.array([bool(r['debug'][1].jam_tripped) for r in rows]),
      target=np.array([r['target'] for r in rows]),
      target_cost=np.array([r.get('target_cost', np.nan) for r in rows]),
      x0=np.array([r['x0'] for r in rows]))


def runs(mask):
  """[first, last] index pairs of the True runs in mask."""
  edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
  return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1))


def score_log(args):
  arg, knot_margin = args
  path = find_log(arg)
  log = read_log(path)
  ramp = Ramp()
  t, repos = log['t'], ~log['c3']
  # Each loop stands for the time until the next one.
  dt = np.diff(t, append=t[-1])
  target_gap = ramp.distance(log['target']) - EE_RADIUS
  x0_gap = ramp.distance(log['x0']) - EE_RADIUS
  inside = target_gap < knot_margin - 1e-4
  parked = (repos & inside & (x0_gap > target_gap + 0.001) &
            (np.linalg.norm(log['target'] - log['x0'], axis=1)
             < PARKED_RADIUS))

  # A reached target with C3 still not taken on the following loop.
  reached = log['target_cost'] >= REACHED_COST
  blocked = repos & reached & np.append(repos[1:], True)

  def episodes_of(mask):
    return [(t[a], t[b] + dt[b]) for a, b in runs(mask)
            if t[b] + dt[b] - t[a] >= MIN_EPISODE]

  episodes = episodes_of(parked)
  blocked_episodes = episodes_of(blocked)
  stretches = [(a, b, t[b] + dt[b] - t[a]) for a, b in runs(repos)]
  out = [f'{op.basename(op.dirname(path))}: {t[-1] - t[0]:.0f} s, '
         f'repositioning {dt[repos].sum():.0f} s in {len(stretches)} '
         f'stretches']
  if stretches:
    a, b, length = max(stretches, key=lambda s: s[2])
    after = b + 1 if b + 1 < len(t) else None
    out.append(
        f'  longest stretch {length:.1f} s at {t[a]:.1f} s '
        f'({MODE_SWITCH_REASON.get(log["reason"][a], "?")} -> '
        f'{MODE_SWITCH_REASON.get(log["reason"][after], "?") if after else "end of log"}'
        f'), latched {100 * log["jam"][a:b + 1].mean():.0f}%')
  parked_seconds = sum(e - s for s, e in episodes)
  out.append(
      f'  parked short of a target: {parked_seconds:.1f} s in '
      f'{len(episodes)} episodes'
      + (': ' + ', '.join(f'{s:.1f}-{e:.1f} s' for s, e in episodes)
         if episodes else ''))
  blocked_seconds = sum(e - s for s, e in blocked_episodes)
  out.append(
      f'  reached a target but kept repositioning: {blocked_seconds:.1f} s in '
      f'{len(blocked_episodes)} episodes'
      + (': ' + ', '.join(f'{s:.1f}-{e:.1f} s' for s, e in blocked_episodes)
         if blocked_episodes else ''))
  share = (dt[repos & inside].sum() / max(dt[repos].sum(), 1e-9))
  out.append(f'  target inside the {1e3 * knot_margin:.0f} mm knot clearance '
             f'on {100 * share:.1f}% of repositioning time')
  return '\n'.join(out), dict(
      parked=parked_seconds, episodes=len(episodes),
      long_episodes=sum(1 for s, e in episodes if e - s > 2.0),
      blocked=blocked_seconds, blocked_episodes=len(blocked_episodes),
      long_blocked=sum(1 for s, e in blocked_episodes if e - s > 2.0),
      inside=share)


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--knot_margin', default=0.004, show_default=True,
              help='fixed_geometry_knot_margin [m], EE surface clearance.')
@click.option('--jobs', default=6, show_default=True)
def main(logs, knot_margin, jobs):
  with multiprocessing.Pool(min(jobs, len(logs))) as pool:
    results = pool.map(score_log, [(log, knot_margin) for log in logs])
  for text, _ in results:
    print(text)
  totals = [r for _, r in results]
  print(f'\nAll {len(logs)} logs: parked {sum(r["parked"] for r in totals):.1f}'
        f' s in {sum(r["episodes"] for r in totals)} episodes '
        f'({sum(r["long_episodes"] for r in totals)} over 2 s); reached but '
        f'kept repositioning {sum(r["blocked"] for r in totals):.1f} s in '
        f'{sum(r["blocked_episodes"] for r in totals)} episodes '
        f'({sum(r["long_blocked"] for r in totals)} over 2 s)')


if __name__ == '__main__':
  main()
