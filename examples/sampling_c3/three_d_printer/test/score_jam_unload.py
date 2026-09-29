"""Scores how cleanly the jam guard's retreat unloads the compliant finger.

Simulation only: it reads FINGER_DEFLECTION_SIMULATION, which the compliant
finger sim publishes and hardware has no equivalent of.  For every rising edge
of jam_tripped on SAMPLING_C3_DEBUG it reports:
  - cos:     the cosine between the gantry's approach (the 0.5 s before the
             trip) and its motion from 0.3 s after the trip to the release.
             Negative means it backed out the way it came in.
  - lat:     how far the gantry moved over that span along the deflection's
             direction at the trip [mm].  Positive unloads the finger.
  - dz:      its vertical motion over that span [mm].
  - d_trip / d_rel:  deflection magnitude at the trip and at the release [mm].
  - snap:    peak deflection rate within 1 s of the release [m/s].
A trip is flagged BAD when it released with more than --loaded_mm of
deflection left; those are the ones that snap.

On the 2026-09-29 logs 000005/7/8 (before unloading existed), 9 of 52 trips
were BAD, with 9-54 mm left at release.  8 were shallow-tier trips, whose
retreat followed the predicted EE's signed-distance gradient; most of those
kept pushing further in and rose over the cone (dz +10 to +22 mm), then
snapped at up to 1.5 m/s.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_jam_unload.py \\
      ~/3d_printer/logs/2026/09_29_26/00000{5,7,8}
"""

import glob
import os.path as op
import sys

import click
import numpy as np
from lcm import EventLog

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib  # noqa: E402
from archive import dairlib as archive_dairlib  # noqa: E402


def find_log(path):
  """Accepts a log file or the folder holding one."""
  if op.isfile(path):
    return path
  found = sorted(glob.glob(op.join(path, '*log-*')))
  if not found:
    raise click.ClickException(f'no log in {path}')
  return found[0]


def decode_debug(data):
  for lcmt in (dairlib.lcmt_sampling_c3_debug,
               archive_dairlib.lcmt_sampling_c3_debug_v6):
    try:
      return lcmt.decode(data)
    except ValueError:
      pass
  raise ValueError('SAMPLING_C3_DEBUG predates the deep jam tier')


def read_log(path):
  """(t, x, y, z) gantry, (t, dx, dy, vx, vy) deflection, (t, tripped)."""
  ee, defl, trip = [], [], []
  t0 = None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    if event.channel == 'PRINTER_STATE_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      ee.append((t, q['x_axis_joint'], q['y_axis_joint'], q['z_axis_joint']))
    elif event.channel == 'FINGER_DEFLECTION_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      defl.append((t, *msg.position[:2], *msg.velocity[:2]))
    elif event.channel == 'SAMPLING_C3_DEBUG':
      trip.append((t, decode_debug(event.data).jam_tripped))
  if not defl:
    raise click.ClickException(f'{path} has no FINGER_DEFLECTION_SIMULATION')
  return np.array(ee), np.array(defl), np.array(trip, dtype=float)


def at(series, t):
  return series[min(np.searchsorted(series[:, 0], t), len(series) - 1), 1:]


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--loaded_mm', default=5.0, show_default=True,
              help='Deflection left at release that counts as BAD [mm].')
def main(logs, loaded_mm):
  total, bad = 0, 0
  for log in logs:
    path = find_log(log)
    ee, defl, trip = read_log(path)
    edges = np.flatnonzero(np.diff(trip[:, 1])) + 1
    ons = [trip[i, 0] for i in edges if trip[i, 1]]
    offs = [trip[i, 0] for i in edges if not trip[i, 1]]
    print(f'{path}\n    trip  dur    cos    lat     dz  d_trip  d_rel  snap')
    for t_on in ons:
      t_off = next((t for t in offs if t > t_on), trip[-1, 0])
      approach = at(ee, t_on) - at(ee, t_on - 0.5)
      motion = at(ee, t_off) - at(ee, t_on + 0.3)
      cos = approach @ motion / (
          np.linalg.norm(approach) * np.linalg.norm(motion) + 1e-12)
      d_on = at(defl, t_on)[:2]
      lat = motion[:2] @ d_on / (np.linalg.norm(d_on) + 1e-12)
      d_rel = np.linalg.norm(at(defl, t_off)[:2])
      window = (defl[:, 0] >= t_off - 1) & (defl[:, 0] <= t_off + 1)
      snap = np.linalg.norm(defl[window, 3:5], axis=1).max()
      is_bad = 1e3 * d_rel > loaded_mm
      total += 1
      bad += is_bad
      print(f'  {t_on:6.2f} {t_off - t_on:4.1f}  {cos:+.2f} {1e3 * lat:+6.1f} '
            f'{1e3 * motion[2]:+6.1f}  {1e3 * np.linalg.norm(d_on):6.1f} '
            f'{1e3 * d_rel:6.1f}  {snap:4.2f}{"  BAD" if is_bad else ""}')
  print(f'{bad} of {total} trips released with more than {loaded_mm} mm of '
        f'deflection left.')


if __name__ == '__main__':
  main()
