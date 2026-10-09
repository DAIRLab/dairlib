"""Prints the loop-by-loop timeline around a jam trip in a compliant-finger
sim log.

Simulation only: it reads FINGER_DEFLECTION_SIMULATION, CONTACT_RESULTS and
OBJECT_STATE_SIMULATION_CLEAN, which hardware has no equivalent of.

One row per SAMPLING_C3_DEBUG message in [T1, T2] (seconds since the log's
first message), with the latest value of every other channel.  Lengths in mm,
forces in N:
  ctrl t      the controller's clock (the message's utime), which the t= of
              sc3_stdout.txt lines uses.  It runs about 0.35-0.5 s behind log
              time, by an amount that varies, so match stdout lines on this
              column.
  md          c3 or rp (repositioning)
  trp ld dp   jam_tripped, jam_tripped_by_load, jam_deep_armed
  push        jam_retreat_pushing (the unload leg is pushing the cone; the
              controller lifts instead)
  unl         jam_unload_distance: the reported EE's horizontal distance from
              the frozen entry point
  load        jam_finger_load: the load tier's estimate of how far the
              reported EE has run past the entry plane
  gap         jam_ee_object_gap_measured: the reported EE's gap to the
              estimated cone
  EE          the reported EE (C3_ACTUAL)
  defl        the true finger deflection
  tip         the largest finger-cone contact force and where it acts
  cone        the true cone position and symmetry axis
  est-true    the estimated cone position (C3_ACTUAL) minus the true one, and
              the angle between their axes [deg]
  plan        the published EE plan's (TRACKING_TRAJECTORY_ACTOR) first and
              last knots

To find trips, run score_jam_trips.py first.  For example, the unload leg that
stood the cone up in 10_06_26/000046 (trip at 19.1 s):
  python3 examples/sampling_c3/three_d_printer/test/diagnose_jam_trip.py \\
      ~/3d_printer/logs/2026/10_06_26/000046 18.8 20.5
"""

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
from score_ramp_jams import ee_plan  # noqa: E402

HEADER = ('      t  ctrl t md trp ld dp push |   unl  load   gap '
          '| EE x y z             | defl dx dy    | tip N  at x y z            '
          '| cone x y z           axis              | est-true x y z     deg '
          '| plan k0 x y z        kN x y z')


def fmt(v, width=6):
  return ' '.join(f'{x:{width}.1f}' for x in v)


def tip_cone_contact(data):
  msg = drake.lcmt_contact_results_for_viz.decode(data)
  best = (0.0, np.zeros(3))
  for c in msg.point_pair_contact_info:
    names = c.body1_name + c.body2_name
    if 'end_effector' in names and 'cone' in names:
      force = np.linalg.norm(c.contact_force)
      if force > best[0]:
        best = (force, 1e3 * np.array(c.contact_point))
  return best


def unit_axis(quat_wxyz):
  return axis_world(quat_wxyz / np.linalg.norm(quat_wxyz))


@click.command()
@click.argument('log')
@click.argument('t1', type=float)
@click.argument('t2', type=float)
def main(log, t1, t2):
  t0 = None
  state = defl = cone = plan = None
  tip = (0.0, np.zeros(3))
  print(HEADER)
  for event in EventLog(find_log(op.expanduser(log)), 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    if t < t1 - 1:
      continue
    if t > t2:
      break
    ch = event.channel
    if ch == 'C3_ACTUAL':
      state = np.array(dairlib.lcmt_c3_state.decode(event.data).state)
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      defl = 1e3 * np.array(
          dairlib.lcmt_robot_output.decode(event.data).position[:2])
    elif ch == 'CONTACT_RESULTS':
      tip = tip_cone_contact(event.data)
    elif ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      cone = np.array(dairlib.lcmt_object_state.decode(event.data).position[:7])
    elif ch == 'TRACKING_TRAJECTORY_ACTOR':
      knots = ee_plan(dairlib.lcmt_timestamped_saved_traj.decode(event.data))
      if knots is not None:
        plan = 1e3 * knots
    elif (ch == 'SAMPLING_C3_DEBUG' and t >= t1 and state is not None
          and cone is not None and defl is not None):
      d = decode_debug(event.data)
      true_axis, est_axis = unit_axis(cone[:4]), unit_axis(state[3:7])
      err_deg = np.degrees(np.arccos(np.clip(true_axis @ est_axis, -1, 1)))
      tip_text = (f'{tip[0]:5.1f} {fmt(tip[1])}' if tip[0] > 0.05
                  else ' ' * 26)
      plan_text = (f'{fmt(plan[0])} {fmt(plan[-1])}' if plan is not None
                   else '')
      print(f'{t:7.2f} {d.utime / 1e6:7.2f} '
            f'{"c3" if d.is_c3_mode else "rp"}  {int(d.jam_tripped)}  '
            f'{int(d.jam_tripped_by_load)}  {int(d.jam_deep_armed)}  '
            f'{int(d.jam_retreat_pushing)}   | '
            f'{1e3 * d.jam_unload_distance:5.1f} '
            f'{1e3 * d.jam_finger_load:5.1f} '
            f'{1e3 * d.jam_ee_object_gap_measured:5.1f} | '
            f'{fmt(1e3 * state[:3])} | {fmt(defl, 5)} | {tip_text} | '
            f'{fmt(1e3 * cone[4:7])} '
            f'{" ".join(f"{x:+.2f}" for x in true_axis)} | '
            f'{fmt(1e3 * (state[7:10] - cone[4:7]), 5)} {err_deg:5.1f} | '
            f'{plan_text}')


if __name__ == '__main__':
  main()
