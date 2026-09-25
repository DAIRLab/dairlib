"""Scores the jam guard's deep tier offline over cone demo logs.

The deep tier trips when the EE-object gap measured from the printer's
REPORTED EE position stays below deep_gap_trip for deep_trip_hold_seconds.
Logs recorded before the tier existed do not carry that gap, so this rebuilds
it from PRINTER_STATE and OBJECT_STATE against an analytic cone (a true cone,
so up to ~3 mm deeper than the hexagonal collision mesh the controller
queries).  Logs that do carry jam_ee_object_gap_measured are also scored on the
controller's own value.

Two things about these logs that are easy to get wrong:
  - The EE centre is PRINTER_STATE minus 0.110864 m in z, on both platforms.
  - PRINTER_STATE_SIMULATION lists the joints z, y, x; hardware lists x, y, z.
    Both are reordered by name here.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_deep_jam_guard.py \\
      ~/3d_printer/logs/2026/09_23_26_pbody/00000{0,1,2} \\
      ~/3d_printer/logs/2026/09_23_26/0000{04,05,06,09,10}

Scored on those logs at -0.011 m / 0.2 s: one trip per hardware run, each at
its launch event (hw0 150.95 s, hw1 155.64 s, hw2 112.09 s), none elsewhere on
hardware, and 1 across the five sim runs (sim05, a productive push seen through
10 mm of injected pose error).  The rebuilt gap reads ~0.5 mm shallower than
the controller's own (median, over the 2026-09-24 scenario runs).
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

# Cone geometry (examples/sampling_c3/urdf/cone): symmetry axis along body +x
# from the base at the origin.  EE_RADIUS is the printer controller's EE sphere
# (urdf/three_d_printer/three_d_printer_end_effector_simple_model.urdf) -- NOT
# the 19.5 mm ee_simple_model_urdf_*.urdf copied into each log folder, which is
# the Franka model.
CONE_HEIGHT = 0.0494
CONE_BASE_RADIUS = 0.0254
EE_RADIUS = 0.010
PRINTER_Z_TO_EE_CENTRE = 0.110864


def cone_signed_distance(ee_in_body):
  """Signed distance from the EE sphere's surface to a solid cone [m].

  @p ee_in_body is N x 3, the EE centre in the cone's body frame.
  """
  x = ee_in_body[:, 0]
  r = np.hypot(ee_in_body[:, 1], ee_in_body[:, 2])
  p = np.c_[x, r]
  # The slanted side, (0, R) to (H, 0), in the axial half-plane.
  a = np.array([0.0, CONE_BASE_RADIUS])
  ab = np.array([CONE_HEIGHT, -CONE_BASE_RADIUS])
  t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
  d_side = np.linalg.norm(p - (a + t[:, None] * ab), axis=1)
  # The base disk.
  d_base = np.linalg.norm(
      p - np.c_[np.zeros_like(r), np.minimum(r, CONE_BASE_RADIUS)], axis=1)
  d = np.minimum(d_side, d_base)
  inside = (x >= 0) & (x <= CONE_HEIGHT) & (
      r <= CONE_BASE_RADIUS * (1 - x / CONE_HEIGHT))
  return np.where(inside, -d, d) - EE_RADIUS


def find_log(folder_or_file):
  if op.isfile(folder_or_file):
    return folder_or_file
  matches = sorted(glob.glob(op.join(folder_or_file, '*log-*')))
  if not matches:
    raise click.BadParameter(f'no simlog/hwlog in {folder_or_file}')
  return matches[0]


def read_log(path):
  """Returns (tick times, logged measured gap or None, EE, object) arrays."""
  ticks, logged_gap, ee, obj = [], [], [], []
  for event in EventLog(path, 'r'):
    t = event.timestamp / 1e6
    channel = event.channel
    if channel == 'SAMPLING_C3_DEBUG':
      ticks.append(t)
      # v4 carries the deep tier but predates mode_switch_decision.
      for lcmt in (dairlib.lcmt_sampling_c3_debug,
                   archive_dairlib.lcmt_sampling_c3_debug_v4):
        try:
          msg = lcmt.decode(event.data)
          logged_gap.append(msg.jam_ee_object_gap_measured)
          break
        except ValueError:
          pass
      else:
        logged_gap.append(None)  # predates the deep tier
    elif channel in ('PRINTER_STATE', 'PRINTER_STATE_SIMULATION'):
      # 1 kHz in older sims; the tick rate is ~10-20 Hz, so thin it out.
      if ee and t - ee[-1][0] < 0.005:
        continue
      msg = dairlib.lcmt_robot_output.decode(event.data)
      by_name = dict(zip(msg.position_names, msg.position))
      ee.append([t, by_name['x_axis_joint'], by_name['y_axis_joint'],
                 by_name['z_axis_joint'] - PRINTER_Z_TO_EE_CENTRE])
    elif channel in ('OBJECT_STATE', 'OBJECT_STATE_SIMULATION'):
      msg = dairlib.lcmt_object_state.decode(event.data)
      obj.append([t] + list(msg.position))  # qw qx qy qz x y z
  has_logged = any(g is not None for g in logged_gap)
  logged = (np.array([np.nan if g is None else g for g in logged_gap])
            if has_logged else None)
  return np.array(ticks), logged, np.array(ee), np.array(obj)


def rebuilt_gap(ticks, ee, obj):
  ee_at = np.stack(
      [np.interp(ticks, ee[:, 0], ee[:, i]) for i in (1, 2, 3)], axis=1)
  latest = np.clip(np.searchsorted(obj[:, 0], ticks, side='right') - 1, 0,
                   len(obj) - 1)
  o = obj[latest]
  rotation = R.from_quat(np.c_[o[:, 2:5], o[:, 1]])
  return cone_signed_distance(rotation.inv().apply(ee_at - o[:, 5:8]))


def trips(ticks, gap, trip, hold):
  """Rising edges of `gap < trip held for hold seconds`, in log seconds."""
  out, since, latched = [], None, False
  for t, g in zip(ticks, gap):
    if np.isfinite(g) and g < trip:
      since = t if since is None else since
      if not latched and t - since >= hold - 1e-9:
        out.append(round(t - ticks[0], 2))
        latched = True
    else:
      since, latched = None, False
  return out


@click.command()
@click.argument('logs', nargs=-1, required=True)
@click.option('--deep_gap_trip', default=-0.011, show_default=True)
@click.option('--deep_trip_hold_seconds', default=0.2, show_default=True)
def main(logs, deep_gap_trip, deep_trip_hold_seconds):
  for log in logs:
    path = find_log(log)
    ticks, logged, ee, obj = read_log(path)
    gap = rebuilt_gap(ticks, ee, obj)
    line = (f'{path}\n  rebuilt: min {1e3 * np.nanmin(gap):6.1f} mm, trips at '
            f'{trips(ticks, gap, deep_gap_trip, deep_trip_hold_seconds)}')
    if logged is not None:
      line += (f'\n  logged:  min {1e3 * np.nanmin(logged):6.1f} mm, trips at '
               f'{trips(ticks, logged, deep_gap_trip, deep_trip_hold_seconds)}')
    print(line)


if __name__ == '__main__':
  main()
