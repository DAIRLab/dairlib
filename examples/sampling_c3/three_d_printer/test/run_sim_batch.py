"""Runs a batch of closed-loop cone demo sims, one at a time, and stops each
run early once the cone can no longer recover.

Each run starts the four processes that start_experiment_with_logs starts --
the logger (start_logging.py), the sim, the OSC and SC3 -- from the repo root,
and saves each one's output into the run's log folder (sim_stdout.txt,
osc_stdout.txt, sc3_stdout.txt).  Runs share LCM channels, so the batch refuses
to start while any of those processes is already running.

A monitor listens to the sim's ground truth (OBJECT_STATE_SIMULATION_CLEAN and
CONTACT_RESULTS) and ends the run, SC3 first, then the OSC, the sim and the
logger, when:
  tipped:   the cone has lain on the build plate, not on the ramp, for
            --tipped_seconds: its axis (body +x) is more than 60 deg from
            upright, it touches build_plate, and it never touches ramp_link.
            The upright cone knocked over before it reaches the ramp counts.
  flipped:  the cone has lain on the ramp tip-down-ramp -- apex pointing down
            and down-slope (axis x > 0.3, z < -0.7), the pole-vault flip --
            for --flipped_seconds.  No run that flipped this way ever made goal
            progress again (09-30 and 10-01 compliant-sim logs); the cone at
            most got knocked back to the toe and flipped again within ~10 s.
            The transient poses at the toe are not this: their axis z stays
            above -0.6.
  out:      the cone has left the workspace in x/y, or fallen below the table.
  success:  the cone has sat at the final goal (within 20 mm, axis within
            0.2 rad of the goal's) for --success_seconds.
  exited:   SC3 exited on its own (e.g. it threw).
  cap:      --cap_seconds since SC3 started.
The reason and time (s since the logger started, which is close to the log's
own clock) go into the log folder's run_record.yaml and, one line per run,
into --summary.

Build everything first (bazel build ...): the debug message's layout changes
between versions, and a partial build mixes binaries.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/run_sim_batch.py \\
      --runs 6 --summary /tmp/batch1.txt
"""

import glob
import os
import os.path as op
import signal
import subprocess
import sys
import time
from datetime import date

import click
import lcm
import numpy as np
import yaml

DAIRLIB_DIR = op.abspath(op.join(op.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'external', 'drake+',
                        'lcmtypes'))
import dairlib  # noqa: E402
import drake  # noqa: E402

BIN = 'bazel-bin/examples/sampling_c3'
# Anchored: an unanchored pattern also matches the shell running pgrep.
# The visualizer only listens, so it may stay open across runs.
RUNNING_PATTERNS = [f'^{BIN}/three_d_printer_(sim|osc|sampling)', '^lcm-logger',
                    '^python3 examples/sampling_c3/start_logging.py']
WORKSPACE_XY = (0.0, 0.35)


def axis_world(quat_wxyz):
  """The cone's symmetry axis (body +x) in world."""
  w, x, y, z = quat_wxyz
  return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z),
                   2 * (x * z - w * y)])


def final_goal(demo):
  params = op.join(DAIRLIB_DIR, 'examples', 'sampling_c3', 'three_d_printer',
                   demo, 'parameters', 'goal_params.yaml')
  goals = yaml.safe_load(open(params))
  position = np.array(goals['fixed_target_position_sequence'][-1][0])
  quat = np.array(goals['fixed_target_orientation_sequence'][-1][0])
  return position, axis_world(quat / np.linalg.norm(quat)), goals[
      'position_success_threshold'], goals['orientation_success_threshold']


class Monitor:
  """Tracks the cone from the sim's ground truth; see the module docstring."""

  def __init__(self, demo, tipped_seconds, flipped_seconds, success_seconds):
    self.tipped_seconds = tipped_seconds
    self.flipped_seconds = flipped_seconds
    self.success_seconds = success_seconds
    self.goal, self.goal_axis, self.goal_tol, self.goal_angle = final_goal(demo)
    self.pose = None
    self.on_plate = self.on_ramp = False
    self.tipped_since = self.flipped_since = None
    self.success_since = self.out_since = None
    self.lc = lcm.LCM()
    self.lc.subscribe('OBJECT_STATE_SIMULATION_CLEAN', self._pose)
    self.lc.subscribe('CONTACT_RESULTS', self._contacts)

  def _pose(self, _, data):
    self.pose = np.array(dairlib.lcmt_object_state.decode(data).position[:7])

  def _contacts(self, _, data):
    msg = drake.lcmt_contact_results_for_viz.decode(data)
    plate = ramp = False
    # Cone-ramp contact is hydroelastic since the ramp pieces went rigid
    # hydroelastic (8449aad55); reading point pairs alone cut runs whose cone
    # lay tipped toward the ramp, rim on the plate and apex on the ramp, as
    # tipped (10_02_26/000022, 23, 25, 27, 28), and never started the flipped
    # window.
    for pair in (list(msg.point_pair_contact_info) +
                 list(msg.hydroelastic_contacts)):
      # Names carry the model instance index, e.g. 'cone(5)'.
      bodies = {name.split('(')[0] for name in (pair.body1_name,
                                                pair.body2_name)}
      if 'cone' in bodies:
        plate |= 'build_plate' in bodies
        ramp |= 'ramp_link' in bodies
    self.on_plate, self.on_ramp = plate, ramp

  def poll(self, now):
    """Handles pending messages; returns a stop reason or None."""
    while self.lc.handle_timeout(0) > 0:
      pass
    if self.pose is None:
      return None
    axis = axis_world(self.pose[:4] / np.linalg.norm(self.pose[:4]))
    position = self.pose[4:7]

    out = (not WORKSPACE_XY[0] <= position[0] <= WORKSPACE_XY[1] or
           not WORKSPACE_XY[0] <= position[1] <= WORKSPACE_XY[1] or
           position[2] < -0.02)
    self.out_since = (self.out_since or now) if out else None
    if self.out_since is not None and now - self.out_since >= 1.0:
      return 'out'

    # A lying cone touches the ramp now and then while it is pushed about; it
    # has to stay off it for the whole window.
    tipped = axis[2] < 0.5 and self.on_plate and not self.on_ramp
    if not tipped or self.on_ramp:
      self.tipped_since = None
    elif self.tipped_since is None:
      self.tipped_since = now
    if (self.tipped_since is not None and
        now - self.tipped_since >= self.tipped_seconds):
      return 'tipped'

    # Contact with the ramp only starts the window: a lying cone loses it now
    # and then; leaving the pose is what resets it.
    flipped_pose = axis[0] > 0.3 and axis[2] < -0.7
    if not flipped_pose:
      self.flipped_since = None
    elif self.flipped_since is None and self.on_ramp:
      self.flipped_since = now
    if (self.flipped_since is not None and
        now - self.flipped_since >= self.flipped_seconds):
      return 'flipped'

    at_goal = (np.linalg.norm(position - self.goal) <= self.goal_tol and
               np.arccos(np.clip(axis @ self.goal_axis, -1, 1)) <=
               self.goal_angle)
    self.success_since = (self.success_since or now) if at_goal else None
    if (self.success_since is not None and
        now - self.success_since >= self.success_seconds):
      return 'success'
    return None


def already_running():
  found = []
  for pattern in RUNNING_PATTERNS:
    out = subprocess.run(['pgrep', '-af', pattern], capture_output=True,
                         text=True).stdout.strip()
    if out:
      found.append(out)
  return found


def log_dirs(logs_root):
  today = op.join(op.expanduser(logs_root), date.today().strftime('%Y'),
                  date.today().strftime('%m_%d_%y'))
  return today, {d for d in glob.glob(op.join(today, '[0-9]' * 6))}


def start(cmd, stdout_path):
  out = open(stdout_path, 'w') if stdout_path else subprocess.DEVNULL
  return subprocess.Popen(cmd, cwd=DAIRLIB_DIR, stdout=out,
                          stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, name, timeout=15.0):
  """SIGINT the process group, escalating if it does not exit."""
  if proc is None or proc.poll() is not None:
    return
  for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
    try:
      os.killpg(proc.pid, sig)
    except ProcessLookupError:
      return
    try:
      proc.wait(timeout=timeout)
      return
    except subprocess.TimeoutExpired:
      print(f'  {name} ignored {signal.Signals(sig).name}', flush=True)


def run_once(demo, logs_root, cap_seconds, tipped_seconds, flipped_seconds,
             success_seconds):
  today, before = log_dirs(logs_root)
  t0 = time.monotonic()
  logger = start(['python3', 'examples/sampling_c3/start_logging.py', 'sim',
                  f'three_d_printer/{demo}', op.expanduser(logs_root)], None)
  log_dir = None
  while log_dir is None and time.monotonic() - t0 < 20:
    time.sleep(0.2)
    new = log_dirs(logs_root)[1] - before
    log_dir = max(new) if new else None
  if log_dir is None:
    stop(logger, 'logger')
    raise click.ClickException(f'the logger made no new folder in {today}')
  time.sleep(1.0)

  procs = {}
  monitor = Monitor(demo, tipped_seconds, flipped_seconds, success_seconds)
  reason = None
  try:
    procs['sim'] = start([f'{BIN}/three_d_printer_sim', f'--demo_name={demo}'],
                         op.join(log_dir, 'sim_stdout.txt'))
    time.sleep(3.0)
    procs['osc'] = start([f'{BIN}/three_d_printer_osc_controller',
                          '--is_simulation=true', f'--demo_name={demo}'],
                         op.join(log_dir, 'osc_stdout.txt'))
    time.sleep(2.0)
    procs['sc3'] = start([f'{BIN}/three_d_printer_sampling_c3_controller',
                          '--is_simulation=true', f'--demo_name={demo}'],
                         op.join(log_dir, 'sc3_stdout.txt'))
    sc3_start = time.monotonic()
    while reason is None:
      time.sleep(0.05)
      now = time.monotonic()
      reason = monitor.poll(now)
      if procs['sc3'].poll() is not None:
        reason = reason or 'exited'
      elif now - sc3_start >= cap_seconds:
        reason = reason or 'cap'
      for name in ('sim', 'osc'):
        if procs[name].poll() is not None:
          reason = reason or f'{name} exited'
  except KeyboardInterrupt:
    reason = 'interrupted'
  finally:
    stopped_at = time.monotonic() - t0
    for name in ('sc3', 'osc', 'sim'):
      stop(procs.get(name), name)
    stop(logger, 'logger')
  record = dict(log=op.basename(log_dir), reason=reason,
                stopped_at=round(stopped_at, 1))
  with open(op.join(log_dir, 'run_record.yaml'), 'w') as f:
    yaml.safe_dump(record, f)
  return log_dir, record


@click.command()
@click.option('--demo', default='cone')
@click.option('--runs', default=6, help='Runs in the batch.')
@click.option('--cap_seconds', default=480.0)
@click.option('--tipped_seconds', default=5.0)
@click.option('--flipped_seconds', default=30.0)
@click.option('--success_seconds', default=5.0)
@click.option('--logs_root', default='~/3d_printer/logs')
@click.option('--summary', default=None,
              help='File to append one line per run to.')
def main(demo, runs, cap_seconds, tipped_seconds, flipped_seconds,
         success_seconds, logs_root, summary):
  running = already_running()
  if running:
    raise click.ClickException('already running:\n' + '\n'.join(running))
  for i in range(runs):
    log_dir, record = run_once(demo, logs_root, cap_seconds, tipped_seconds,
                               flipped_seconds, success_seconds)
    line = (f'{log_dir}  {record["reason"]}  at {record["stopped_at"]} s')
    print(f'run {i + 1}/{runs}: {line}', flush=True)
    if summary:
      with open(summary, 'a') as f:
        f.write(line + '\n')
    if record['reason'] == 'interrupted':
      break
    time.sleep(2.0)
    leftover = already_running()
    if leftover:
      raise click.ClickException('processes left running:\n' +
                                 '\n'.join(leftover))


if __name__ == '__main__':
  main()
