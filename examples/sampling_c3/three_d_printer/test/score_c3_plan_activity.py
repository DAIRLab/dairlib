"""Scores whether C3 plans actually move the end effector where the cone stalls.

In the 2026-09-29 compliant-sim logs the cone stalls upright at the ramp toe
and lying on the ramp.  There, most C3 plans barely move the end effector while
the plan's own object trajectory (C3_TRAJECTORY_OBJECT_CURR_PLAN) predicts a
large tilt or slide:  the plan gets its progress for free from environment
contact forces, so it has no reason to push.  Per log, for the goal-2 phase:

  - toe / ramp:  C3 loops with the cone (clean pose) upright at the toe, or
    lying on the ramp.  passive = share of C3 loops whose executed EE plan
    (C3_EXECUTION_TRAJECTORY_ACTOR) moves under --passive_mm over the horizon;
    plan tilt = the plan's tracked-axis swing over the horizon (median);
    push side = share with the EE on the +x (away-from-ramp) side of the cone.
  - the goal steps reached (and when), and the control-loop period.
  - bouts:  each run of consecutive upright-at-toe C3 loops lasting 0.3 s or
    more, with the EE's start offset from the cone (x, z) [mm], the plan's
    median displacement (x, y, z) [mm], the peak real tilt within 2 s of the
    bout's end, the peak finger deflection over the bout and those 2 s (sim
    only, FINGER_DEFLECTION_SIMULATION), and the mode-switch reason that ended
    it (4 = unproductive, 6 = jam).

The cone's pose is the simulator's clean one (OBJECT_STATE_SIMULATION_CLEAN);
its symmetry axis is body +x.  EE = PRINTER_STATE_SIMULATION joints minus
0.110864 m in z (the gantry, not the fingertip).

Usage:
  python3 examples/sampling_c3/three_d_printer/test/score_c3_plan_activity.py \\
      ~/3d_printer/logs/2026/09_29_26/0000{09..13}
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

EE_Z_OFFSET = 0.110864
TOE_X = (0.215, 0.25)   # upright-at-toe band for the cone's base centre [m]
RAMP_X_MAX = 0.2        # lying cone with its base centre up-ramp of this [m]


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
               archive_dairlib.lcmt_sampling_c3_debug_v8,
               archive_dairlib.lcmt_sampling_c3_debug_v7,
               archive_dairlib.lcmt_sampling_c3_debug_v6):
    try:
      return lcmt.decode(data)
    except ValueError:
      pass
  raise ValueError('SAMPLING_C3_DEBUG predates the deep jam tier')


def trajectories(data):
  msg = dairlib.lcmt_timestamped_saved_traj.decode(data)
  return {t.trajectory_name: np.array(t.datapoints)
          for t in msg.saved_traj.trajectories}


def axis_of(quat_wxyz):
  """The body +x axis in world for a wxyz quaternion (rows or single)."""
  q = np.atleast_2d(quat_wxyz)
  q = q / np.linalg.norm(q, axis=1, keepdims=True)
  w, x, y, z = q.T
  return np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z),
                   2 * (x * z - w * y)], axis=1)


def swing_deg(a, b):
  return np.degrees(np.arccos(np.clip(np.sum(a * b, axis=-1), -1, 1)))


def read_log(path):
  """One row per SAMPLING_C3_DEBUG loop, plus the finger deflection series."""
  rows, deflection = [], []
  obj = ee = ee_plan = obj_plan_quat = None
  t0 = None
  for event in EventLog(path, 'r'):
    t0 = event.timestamp if t0 is None else t0
    t = (event.timestamp - t0) / 1e6
    ch = event.channel
    if ch == 'OBJECT_STATE_SIMULATION_CLEAN':
      obj = np.array(dairlib.lcmt_object_state.decode(event.data).position[:7])
    elif ch == 'PRINTER_STATE_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q = dict(zip(msg.position_names, msg.position))
      ee = np.array([q['x_axis_joint'], q['y_axis_joint'],
                     q['z_axis_joint'] - EE_Z_OFFSET])
    elif ch == 'FINGER_DEFLECTION_SIMULATION':
      msg = dairlib.lcmt_robot_output.decode(event.data)
      deflection.append((t, np.hypot(*msg.position[:2])))
    elif ch == 'C3_EXECUTION_TRAJECTORY_ACTOR':
      ee_plan = trajectories(event.data)['end_effector_position_target'][:3]
    elif ch == 'C3_TRAJECTORY_OBJECT_CURR_PLAN':
      obj_plan_quat = trajectories(event.data)['object_orientation_target_0']
    elif ch == 'SAMPLING_C3_DEBUG':
      if any(v is None for v in (obj, ee, ee_plan, obj_plan_quat)):
        continue
      msg = decode_debug(event.data)
      rows.append(dict(
          t=t, c3=msg.is_c3_mode, goal=msg.detected_goal_changes,
          switch=msg.mode_switch_reason, obj=obj, ee=ee,
          plan_disp=ee_plan[:, -1] - ee_plan[:, 0],
          plan_tilt=swing_deg(axis_of(obj_plan_quat[:, 0]),
                              axis_of(obj_plan_quat[:, -1]))[0]))
  if not rows:
    raise click.ClickException(f'{path}: no complete loops')
  return rows, np.array(deflection) if deflection else np.zeros((0, 2))


def summarize(name, rows, deflection, goal, passive_mm):
  t = np.array([r['t'] for r in rows])
  axis_z = axis_of(np.array([r['obj'][:4] for r in rows]))[:, 2]
  obj_x = np.array([r['obj'][4] for r in rows])
  in_goal = np.array([r['goal'] == goal for r in rows])
  c3 = np.array([r['c3'] for r in rows]) & in_goal
  upright_toe = (axis_z > 0.95) & (obj_x > TOE_X[0]) & (obj_x < TOE_X[1])
  on_ramp = (axis_z < 0.95) & (obj_x < RAMP_X_MAX)
  moved = np.array([1e3 * np.linalg.norm(r['plan_disp']) for r in rows])
  tilt = np.array([r['plan_tilt'] for r in rows])
  rel = np.array([1e3 * (r['ee'] - r['obj'][4:7]) for r in rows])

  goals = np.array([r['goal'] for r in rows])
  period = np.diff(t) * 1e3
  print(f'== {name}  goal {goal}: {in_goal.sum()} loops, C3 {c3.sum()};  '
        f'last goal step {goals.max()} (reached at '
        + ', '.join(f'{t[np.argmax(goals >= g)]:.0f}'
                    for g in range(1, goals.max() + 1)) +
        f' s);  loop period p50/p90 {np.median(period):.0f}/'
        f'{np.percentile(period, 90):.0f} ms')
  for label, mask in (('toe', c3 & upright_toe), ('ramp', c3 & on_ramp)):
    if not mask.any():
      print(f'   {label:4s}: no C3 loops')
      continue
    print(f'   {label:4s}: C3 loops {mask.sum():5d}  passive '
          f'{np.mean(moved[mask] < passive_mm):.2f}  plan tilt p50 '
          f'{np.median(tilt[mask]):4.0f} deg  push side '
          f'{np.mean(rel[mask, 0] > 5):.2f}')

  # Bouts of consecutive upright-at-toe C3 loops.
  bouts, current = [], []
  for i in range(len(rows)):
    if c3[i] and upright_toe[i]:
      current.append(i)
    elif current:
      bouts.append(current)
      current = []
  if current:
    bouts.append(current)
  bouts = [b for b in bouts if t[b[-1]] - t[b[0]] >= 0.3]
  if not bouts:
    return
  print('   toe bouts:  start  dur  EE-cone x,z [mm]  plan dx,dy,dz p50 '
        '[mm]  peak tilt  peak defl [mm]  end')
  for b in bouts:
    t_start, t_end = t[b[0]], t[b[-1]]
    after = (t >= t_start) & (t <= t_end + 2.0)
    peak_tilt = np.degrees(np.arccos(np.clip(axis_z[after].min(), -1, 1)))
    d = deflection[(deflection[:, 0] >= t_start) &
                   (deflection[:, 0] <= t_end + 2.0), 1] if len(
                       deflection) else np.zeros(0)
    peak_defl = 1e3 * d.max() if len(d) else float('nan')
    disp = np.median([1e3 * rows[i]['plan_disp'] for i in b], axis=0)
    end = rows[min(b[-1] + 1, len(rows) - 1)]['switch']
    print(f'            {t_start:6.1f} {t_end - t_start:4.1f}  '
          f'{rel[b[0], 0]:+5.0f} {rel[b[0], 2]:+4.0f}       '
          f'{disp[0]:+5.0f} {disp[1]:+5.0f} {disp[2]:+4.0f}          '
          f'{peak_tilt:4.0f}      {peak_defl:5.1f}        {end}')


@click.command()
@click.argument('paths', nargs=-1, required=True)
@click.option('--goal', default=2, show_default=True,
              help='Goal-sequence step to score (the toe/ramp phase).')
@click.option('--passive_mm', default=10.0, show_default=True,
              help='EE plan displacement below which a plan counts as '
                   'passive [mm].')
def main(paths, goal, passive_mm):
  for path in paths:
    log = find_log(path)
    rows, deflection = read_log(log)
    summarize(op.basename(log), rows, deflection, goal, passive_mm)


if __name__ == '__main__':
  main()
