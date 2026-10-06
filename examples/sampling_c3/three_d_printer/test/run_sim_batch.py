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
  flipped:  the cone has lain on the ramp tip-down-ramp, apex pointing
            down-slope:  lying on its side (axis x > 0.5) for
            --flipped_lying_seconds, or steeply apex down and down-slope (axis
            x > 0.3, z < -0.7), the pole-vault flip, for --flipped_seconds.  No
            run that pole-vaulted ever made goal progress again (09-30 and 10-01
            compliant-sim logs); the cone at most got knocked back to the toe
            and flipped again within ~10 s.  The transient poses at the toe are
            not this: their axis z stays above -0.6.  Lying tip-down-ramp, the
            cone can still be pushed up the ramp but not seated
            (10_06_26/000018 spent its last 300 s so, up to the jig); the
            ratchet below misses it, since the seated goal's axis (apex down)
            is nearer this pose's than the tip-up-ramp one's.  On the path to
            the seat the apex points up the ramp or down, so axis x stays near
            or below 0; the lying pose has only lasted under 5 s while the cone
            slid back down (0.3 s, 10_06_26/000017).
  regressed: the cone has lost ground on the goal it is pursuing, against the
            pose it started the goal from, i.e. where the last goal left it, for
            --regressed_seconds:  its xy distance to the goal has grown by more
            than --regressed_mm, or its orientation has turned by more than
            --regressed_deg beyond what it spent approaching the goal's,
            error(now, start) + error(now, goal) - error(start, goal).  That is
            0 for turning straight toward the goal, and unlike the change in the
            goal error alone it also sees a cone turning crosswise or end for
            end under the seated goal, whose axis is vertical.  Orientation
            errors are judged as the goals judge them (IsObjectOnTarget):  by
            where goal_params' tracked_orientation_axis points, so a turn about
            that axis -- the cone rolling about its symmetry axis -- never
            counts, or by the whole rotation when no axis is tracked.  The cone
            should ratchet through the goal sequence; a run that slides back
            down the ramp or stands the cone back up on its base has to redo an
            earlier goal under settings that were not tuned for it, even when it
            gets there (10_06_26/000017 seated the cone after sliding from
            mid-slope back to the plate).  Over 10_02_26/000022-28,
            10_05_26/000007-12 and 10_06_26/000000-20, goals that held their
            ground lost at most 18 mm and turned at most 40 deg this way for
            5 s; the stand-ups, the cone lying crosswise or twisted in the
            trough (10_05_26/000011 @ 383 s, 10_06_26/000002 @ 385 s, 000006
            @ 431 s), the flips and the falls back off the toe turned
            68-258 deg, and the slide lost 84 mm.  A cone knocked over on the
            plate usually trips this about a second before tipped.  The goal
            index is SC3's own count of goal changes, from SAMPLING_C3_DEBUG;
            run_record.yaml keeps it as goal_index.
  out:      the cone has left the workspace in x/y, or fallen below the table.
  success:  the cone has sat at the final goal (within 20 mm, orientation
            within 0.2 rad of the goal's, judged as regressed judges it) for
            --success_seconds.
  exited:   SC3 exited on its own (e.g. it threw).
  cap:      --cap_seconds since SC3 started.
The reason and time (s since the logger started, a few seconds before the
log's first message) go into the log folder's run_record.yaml and, one line
per run, into --summary.

--arm interleaves parameter variants run by run (A, B, A, B, ...), so drift in
the machine's load or anything else over a batch lands on every arm alike.  An
arm is a name, optionally followed by top-level yaml keys to override:
  --arm wg20 --arm 'wg018=sampling_c3plus_options.yaml:w_G=0.18,w_G_position=0.18'
Files are looked up in the demo's parameters folder, then in
printer_shared_parameters; several files go after ';'.  Each override is
written into the yaml in place before the logger starts, so the copy the
logger saves into the log folder records the arm, and every patched file is
restored after each run.  The batch refuses to start if a file to be patched
differs from git HEAD.  run_record.yaml also gets the arm and the load
average at the start and end of the run.

Build everything first (bazel build ...): the debug message's layout changes
between versions, and a partial build mixes binaries.

Usage:
  python3 examples/sampling_c3/three_d_printer/test/run_sim_batch.py \\
      --runs 6 --summary /tmp/batch1.txt
"""

import glob
import os
import os.path as op
import re
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


def rotate(quat_wxyz, v):
  """v rotated by the unit quaternion quat_wxyz."""
  w, x, y, z = quat_wxyz
  return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
                    2 * (x * z + w * y)],
                   [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
                    2 * (y * z - w * x)],
                   [2 * (x * z - w * y), 2 * (y * z + w * x),
                    1 - 2 * (x * x + y * y)]]) @ v


def axis_world(quat_wxyz):
  """The cone's symmetry axis (body +x) in world."""
  return rotate(quat_wxyz, np.array([1.0, 0.0, 0.0]))


def goal_params(demo):
  return yaml.safe_load(open(op.join(
      DAIRLIB_DIR, 'examples', 'sampling_c3', 'three_d_printer', demo,
      'parameters', 'goal_params.yaml')))


def final_goal(demo):
  goals = goal_params(demo)
  position = np.array(goals['fixed_target_position_sequence'][-1][0])
  quat = np.array(goals['fixed_target_orientation_sequence'][-1][0])
  return position, quat, goals['position_success_threshold'], goals[
      'orientation_success_threshold']


def goal_sequence(demo):
  """[(xy position, orientation)] per goal."""
  goals = goal_params(demo)
  return [(np.array(p[0])[:2], np.array(q[0]))
          for p, q in zip(goals['fixed_target_position_sequence'],
                          goals['fixed_target_orientation_sequence'])]


def tracked_axis(demo):
  """The cone's tracked body axis, or None when it tracks the whole
  orientation."""
  axis = np.array(goal_params(demo).get('tracked_orientation_axis',
                                        [[0, 0, 0]])[0], dtype=float)
  return axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-6 else None


def misalignment(axis, goal_axis):
  return np.arccos(np.clip(axis @ goal_axis, -1, 1))


def orientation_error(quat_a, quat_b, axis_body):
  """The angle between two orientations as the goals judge it, mirroring
  SamplingC3GoalParams::IsObjectOnTarget:  between the tracked body axis' two
  directions in world when there is one, so a turn about that axis counts for
  nothing, and the whole rotation otherwise."""
  quat_a, quat_b = (q / np.linalg.norm(q) for q in (quat_a, quat_b))
  if axis_body is None:
    return 2 * np.arccos(np.clip(abs(quat_a @ quat_b), 0, 1))
  return misalignment(rotate(quat_a, axis_body), rotate(quat_b, axis_body))


class Monitor:
  """Tracks the cone from the sim's ground truth; see the module docstring."""

  def __init__(self, demo, tipped_seconds, flipped_seconds,
               flipped_lying_seconds, success_seconds, regressed_seconds,
               regressed_mm, regressed_deg):
    self.tipped_seconds = tipped_seconds
    self.flipped_seconds = flipped_seconds
    self.flipped_lying_seconds = flipped_lying_seconds
    self.success_seconds = success_seconds
    self.regressed_seconds = regressed_seconds
    self.regressed_distance = regressed_mm / 1e3
    self.regressed_angle = np.radians(regressed_deg)
    self.goal, self.goal_quat, self.goal_tol, self.goal_angle = final_goal(demo)
    self.goals = goal_sequence(demo)
    self.tracked_axis = tracked_axis(demo)
    self.pose = None
    self.goal_index = -1
    # (goal index, xy distance, orientation error, orientation) when the
    # current goal began.
    self.goal_start = None
    self.on_plate = self.on_ramp = False
    self.tipped_since = self.flipped_since = self.regressed_since = None
    self.success_since = self.out_since = None
    self.lc = lcm.LCM()
    self.lc.subscribe('OBJECT_STATE_SIMULATION_CLEAN', self._pose)
    self.lc.subscribe('CONTACT_RESULTS', self._contacts)
    self.lc.subscribe('SAMPLING_C3_DEBUG', self._debug)

  def _pose(self, _, data):
    self.pose = np.array(dairlib.lcmt_object_state.decode(data).position[:7])

  def _debug(self, _, data):
    self.goal_index = dairlib.lcmt_sampling_c3_debug.decode(
        data).detected_goal_changes

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
    quat = self.pose[:4] / np.linalg.norm(self.pose[:4])
    axis = axis_world(quat)
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
    lying_flipped = axis[0] > 0.5
    flipped_pose = lying_flipped or (axis[0] > 0.3 and axis[2] < -0.7)
    if not flipped_pose:
      self.flipped_since = None
    elif self.flipped_since is None and self.on_ramp:
      self.flipped_since = now
    if (self.flipped_since is not None and now - self.flipped_since >= (
        self.flipped_lying_seconds if lying_flipped else self.flipped_seconds)):
      return 'flipped'

    if self.goal_index >= 0:
      goal_xy, goal_quat = self.goals[min(self.goal_index, len(self.goals) - 1)]
      distance = np.linalg.norm(position[:2] - goal_xy)
      angle = orientation_error(quat, goal_quat, self.tracked_axis)
      if self.goal_start is None or self.goal_start[0] != self.goal_index:
        self.goal_start = (self.goal_index, distance, angle, quat)
        self.regressed_since = None
      _, start_distance, start_angle, start_quat = self.goal_start
      turned_away = (orientation_error(quat, start_quat, self.tracked_axis) +
                     angle - start_angle)
      regressed = (distance - start_distance > self.regressed_distance or
                   turned_away > self.regressed_angle)
      self.regressed_since = ((self.regressed_since or now) if regressed
                              else None)
      if (self.regressed_since is not None and
          now - self.regressed_since >= self.regressed_seconds):
        return 'regressed'

    at_goal = (np.linalg.norm(position - self.goal) <= self.goal_tol and
               orientation_error(quat, self.goal_quat, self.tracked_axis) <=
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


def parameter_file(demo, name):
  for folder in (op.join(demo, 'parameters'), 'printer_shared_parameters'):
    path = op.join(DAIRLIB_DIR, 'examples', 'sampling_c3', 'three_d_printer',
                   folder, name)
    if op.exists(path):
      return path
  raise click.ClickException(f'no parameter file {name} for demo {demo}')


def parse_arm(demo, spec):
  """'name' or 'name=file.yaml:k=v,k=v;file.yaml:k=v' -> (name, {path:
  {key: value}})."""
  name, _, rest = spec.partition('=')
  patches = {}
  for part in filter(None, rest.split(';')):
    file_name, _, pairs = part.partition(':')
    keys = patches.setdefault(parameter_file(demo, file_name.strip()), {})
    for pair in filter(None, pairs.split(',')):
      key, _, value = pair.partition('=')
      if not value:
        raise click.ClickException(f'arm {name}: no value in "{pair}"')
      keys[key.strip()] = value.strip()
  return name.strip(), patches


def patched(text, keys, path):
  """Overrides top-level keys in a yaml's text, keeping comments."""
  for key, value in keys.items():
    pattern = re.compile(rf'^({re.escape(key)}:[ \t]*)([^#\n]*?)([ \t]*#.*)?$',
                         re.MULTILINE)
    if len(pattern.findall(text)) != 1:
      raise click.ClickException(f'{path}: top-level key {key} not found '
                                 'exactly once')
    text = pattern.sub(lambda m: m.group(1) + value + (m.group(3) or ''),
                       text)
    if yaml.safe_load(text).get(key) != yaml.safe_load(value):
      raise click.ClickException(f'{path}: {key} did not parse as {value}')
  return text


def interrupt(*_):
  raise KeyboardInterrupt


def run_once(demo, logs_root, cap_seconds, tipped_seconds, flipped_seconds,
             flipped_lying_seconds, success_seconds, regressed_seconds,
             regressed_mm, regressed_deg, arm=None):
  today, before = log_dirs(logs_root)
  load_start = os.getloadavg()
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
  monitor = Monitor(demo, tipped_seconds, flipped_seconds,
                    flipped_lying_seconds, success_seconds, regressed_seconds,
                    regressed_mm, regressed_deg)
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
                stopped_at=round(stopped_at, 1), goal_index=monitor.goal_index,
                load_average=[round(v, 2) for v in load_start + os.getloadavg()])
  if arm is not None:
    name, patches = arm
    record['arm'] = name
    record['arm_overrides'] = {op.basename(path): keys
                               for path, keys in patches.items()}
  with open(op.join(log_dir, 'run_record.yaml'), 'w') as f:
    yaml.safe_dump(record, f)
  return log_dir, record


@click.command()
@click.option('--demo', default='cone')
@click.option('--runs', default=6, help='Runs in the batch.')
@click.option('--cap_seconds', default=480.0)
@click.option('--tipped_seconds', default=5.0)
@click.option('--flipped_seconds', default=30.0)
@click.option('--flipped_lying_seconds', default=5.0)
@click.option('--success_seconds', default=5.0)
@click.option('--regressed_seconds', default=5.0)
@click.option('--regressed_mm', default=30.0)
@click.option('--regressed_deg', default=60.0)
@click.option('--logs_root', default='~/3d_printer/logs')
@click.option('--summary', default=None,
              help='File to append one line per run to.')
@click.option('--arm', 'arm_specs', multiple=True,
              help='A parameter variant; runs cycle through the arms in order.'
                   '  See the module docstring.')
def main(demo, runs, cap_seconds, tipped_seconds, flipped_seconds,
         flipped_lying_seconds, success_seconds, regressed_seconds,
         regressed_mm, regressed_deg, logs_root, summary, arm_specs):
  running = already_running()
  if running:
    raise click.ClickException('already running:\n' + '\n'.join(running))
  arms = [parse_arm(demo, spec) for spec in arm_specs]
  originals = {}
  for _, patches in arms:
    for path in patches:
      if subprocess.run(['git', 'diff', '--quiet', 'HEAD', '--', path],
                        cwd=DAIRLIB_DIR).returncode != 0:
        raise click.ClickException(f'{path} differs from HEAD; commit or '
                                   'revert it before patching it per arm')
      originals[path] = open(path).read()
  for _, patches in arms:  # Fail on a bad override before the first run.
    for path, keys in patches.items():
      patched(originals[path], keys, path)

  def restore():
    for path, text in originals.items():
      with open(path, 'w') as f:
        f.write(text)

  # SIGTERM restores the yaml too; SIGINT is a KeyboardInterrupt already.
  signal.signal(signal.SIGTERM, interrupt)
  try:
    for i in range(runs):
      arm = arms[i % len(arms)] if arms else None
      if arm is not None:
        for path, keys in arm[1].items():
          with open(path, 'w') as f:
            f.write(patched(originals[path], keys, path))
      try:
        log_dir, record = run_once(demo, logs_root, cap_seconds,
                                   tipped_seconds, flipped_seconds,
                                   flipped_lying_seconds, success_seconds,
                                   regressed_seconds, regressed_mm,
                                   regressed_deg, arm)
      finally:
        restore()
      line = (f'{log_dir}  {record["reason"]}  at {record["stopped_at"]} s'
              + (f'  arm {arm[0]}' if arm else ''))
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
  finally:
    restore()


if __name__ == '__main__':
  main()
