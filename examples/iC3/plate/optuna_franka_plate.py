#!/usr/bin/env python3
"""
Optuna optimization over the franka plate MPC workflow.

Replaces the manual procman loop:
  1. "press setup until the robot settles"  -> launch sim + osc, retry until settled
  2. "press hybrid_mpc_controller"          -> launch hybrid mpc controllers
  3. score the trial on the object's final state
"""
import atexit

import os
import sys
import time
import signal
import subprocess
import tempfile
import threading
from collections import deque

import numpy as np
import yaml
import lcm
import optuna

DAIRLIB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_ALL_PROCS = []


def _prepend_if_exists(path):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


# When run directly with `python3`, Bazel does not populate PYTHONPATH for us.
# Add the repo-local generated LCM bindings so `import dairlib` works.
_prepend_if_exists(os.path.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))

# --- generated LCM types ---
from dairlib import lcmt_object_state   # OBJECT_STATE_SIMULATION : double position[7] (wxyz, xyz)
from dairlib import lcmt_osc_output     # OSC_DEBUG_FRANKA : tracking_data[0].y[3] = ee position


# ======================================================================
# Config
# ======================================================================
REPO = DAIRLIB_DIR
BIN = os.path.join(REPO, "bazel-bin/examples/iC3/plate")

LCM_URL = "udpm://239.255.76.67:7667?ttl=0"

# workflow timing
SETTLE_TIMEOUT = 0.5      # s to wait for the robot to settle after setup
SETTLE_ATTEMPTS = 5       # how many times to retry "setup"
RUN_TIME = 4.0          # s to run the hybrid mpc controller per trial

SETTLE_TARGET = np.array([0.53, -0.35, 0.47])   # object xyz to settle at
SETTLE_POS_TOL = 0.01                           # 1 cm
SETTLE_HOLD = 0.25                               # seconds it must stay within tol
HYBRID_STARTUP_DELAY = 0.25                       # let setup fully stabilize
EPISODE_RETRY_DELAY = 0.25                        # pause before retrying a dead episode
MAX_EPISODE_RETRIES = 5

# objective target (the pose you want the object to end at)
TARGET_XYZ = np.array([0.53, -0.35, 0.47])
TARGET_QUAT = np.array([0.0, 0.0, -1.0, 0.0])   # set to None to ignore orientation
OBJ_POS_W = 8
Y_POS_MULTIPLIER = 2

FAILED_SETTLE_PENALTY = 1e4
CRASH_PENALTY = 1e4

N_TRIALS = 5000
N_RUNS = 10

OSC_PARAMS = os.path.join(REPO, "examples/iC3/plate/parameters/franka_plate_osc_controller_params_optuna.yaml")
MPC_PARAMS = os.path.join(REPO, "examples/iC3/plate/parameters/plate_hybrid_mpc_options_optuna.yaml")

def _crashed_procs(procs):
    """Return list of (name, returncode) for processes that died on their own."""
    dead = []
    for p in procs:
        rc = p.p.poll()
        if rc is not None and rc != 0:      # exited nonzero or killed by signal
            dead.append((p.name, rc))
    return dead

def _reap_all():
    """Force-kill every launched process group that is still alive.
    Guards against PID recycling: never signal a PID we've already reaped."""
    for p in list(_ALL_PROCS):
        try:
            if p.p.poll() is None:            # still running & unreaped -> safe
                os.killpg(os.getpgid(p.p.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
    _ALL_PROCS.clear()

def _signal_handler(signum, frame):
    print(f"\n[signal {signum}] killing all launched processes...")
    _reap_all()
    os._exit(1)   # hard exit; do NOT run more Python (avoids reentrancy)

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
atexit.register(_reap_all)   # also cleans up on normal exit / uncaught errors

# ======================================================================
# LCM monitor (background thread)
# ======================================================================
class Monitor:
    def __init__(self, lcm_url):
        self.lc = lcm.LCM(lcm_url)
        self.lock = threading.Lock()

        self.obj_buf = deque(maxlen=500)   # (t, pos[7])
        self.ee_buf = deque(maxlen=500)    # (t, y[3])
        self._stop = False

        self.lc.subscribe("OBJECT_STATE_SIMULATION", self._on_object)
        self.lc.subscribe("OSC_DEBUG_FRANKA", self._on_osc)

        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self._stop:
            self.lc.handle_timeout(100)

    def _on_object(self, channel, data):
        try:
            msg = lcmt_object_state.decode(data)
        except Exception:
            return
        pos = np.array(msg.position[:7])   # [qw,qx,qy,qz, x,y,z]
        with self.lock:
            self.obj_buf.append((time.time(), pos))

    def _on_osc(self, channel, data):
        try:
            msg = lcmt_osc_output.decode(data)
        except Exception:
            return
        if not msg.tracking_data:
            return
        y = np.array(msg.tracking_data[0].y[:3])
        with self.lock:
            self.ee_buf.append((time.time(), y))

    # ---- settling: little motion in a recent window ----
    def is_settled(self):
            """True if the object's xyz has stayed within SETTLE_POS_TOL of
            SETTLE_TARGET for the entire trailing SETTLE_HOLD-second window."""
            now = time.time()
            with self.lock:
                recent = [(t, v) for (t, v) in self.obj_buf
                          if now - t <= SETTLE_HOLD]

            # every sample in the window must be within tolerance
            for (_, pos) in recent:
                xyz = pos[4:7]                       # [qw,qx,qy,qz, x,y,z] -> xyz
                print(xyz)
                if np.linalg.norm(xyz - SETTLE_TARGET) > SETTLE_POS_TOL:
                    return False
            return True

    def object_final_state(self, tail=20):
        """Average the last `tail` object poses for a stable final estimate."""
        with self.lock:
            if len(self.obj_buf) < tail:
                if not self.obj_buf:
                    return None
                tail = len(self.obj_buf)
            poses = np.array([v for (_, v) in list(self.obj_buf)[-tail:]])
        quat = poses[:, 0:4].mean(0)
        n = np.linalg.norm(quat)
        quat = quat / n if n > 0 else quat
        xyz = poses[:, 4:7].mean(0)
        return quat, xyz

    def clear(self):
        with self.lock:
            self.obj_buf.clear()
            self.ee_buf.clear()

    def stop(self):
        self._stop = True
        self.thread.join(timeout=1.0)
        # Explicitly delete the LCM instance to free its socket descriptor
        if hasattr(self, 'lc'):
            del self.lc


# ======================================================================
# Process management
# ======================================================================
class Proc:
    def __init__(self, name, exec_path, env, args=None, logdir=None):
        self.name = name
        if logdir:
            out = open(os.path.join(logdir, f"{name}.log"), "w")
        else:
            out = subprocess.DEVNULL
        cmd = [exec_path] + (args or [])
        self.p = subprocess.Popen(
            cmd, cwd=REPO, env=env, stdout=out,
            stderr=subprocess.STDOUT if logdir else subprocess.DEVNULL,
            start_new_session=True,
        )
        _ALL_PROCS.append(self)          # <-- track globally

    def alive(self):
        return self.p.poll() is None

    def kill(self):
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
                time.sleep(0.5)
                if self.p.poll() is None:
                    os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self in _ALL_PROCS:
            _ALL_PROCS.remove(self)      # <-- untrack when cleanly killed

# ======================================================================
# Objective computation
# ======================================================================
def body_z_axis(quat):
    """World-frame direction of the body z-axis, given quat = [w, x, y, z]."""
    w, x, y, z = quat
    return np.array([
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ])

def z_axis_error(quat, target_quat):
    """Angle (radians) between current and target body z-axes.
    0 = aligned, pi = fully flipped. Ignores spin about z."""
    z_cur = body_z_axis(quat)
    z_tgt = body_z_axis(target_quat)
    z_cur /= np.linalg.norm(z_cur)
    z_tgt /= np.linalg.norm(z_tgt)
    d = np.clip(np.dot(z_cur, z_tgt), -1.0, 1.0)
    return np.arccos(d)   # 0 .. pi

def compute_objective(monitor):
    result = monitor.object_final_state()
    if result is None:
        return FAILED_SETTLE_PENALTY
    quat, xyz = result

    d = xyz - TARGET_XYZ
    d_weighted = np.array([d[0], Y_POS_MULTIPLIER * d[1]])
    pos_cost = 6 + OBJ_POS_W * np.linalg.norm(d_weighted)

    ori_weight = 0
    pos_weight = 1
    if (d[0]**2 + d[1]**2 < 0.06 and xyz[2] > 0.42):
        print("ON PLATE")
        ori_weight = 3
        pos_weight = 0

    if TARGET_QUAT is not None:
        dq = min(1.0, abs(float(np.dot(quat, TARGET_QUAT))))
        ori_err_z = z_axis_error(quat, TARGET_QUAT)   # radians, 0..pi
        ori_err = 2.0 * np.arccos(dq)   # radians
        return pos_weight * pos_cost + ori_weight * (0.25 * ori_err + 0.75 * ori_err_z)
        
    return pos_cost


def _run_episode(trial_number, run_idx, env, logdir):
    """One full setup -> settle -> MPC run -> score cycle. Returns a scalar cost."""
    monitor = Monitor(LCM_URL)
    procs = []

    def launch(name, binary, args=None):
        p = Proc(name, os.path.join(BIN, binary), env, args=args, logdir=logdir)
        procs.append(p)
        return p

    def kill_all():
        for p in reversed(procs):
            p.kill()
        procs.clear()

    try:
        # --- 1) setup with retries until settled ---
        settled = False
        for attempt in range(SETTLE_ATTEMPTS):
            monitor.clear()
            launch("franka_osc", "franka_plate_osc_controller",
                   args=["--controller_parameters="
                         "examples/iC3/plate/parameters/"
                         "franka_plate_osc_controller_params_optuna.yaml"])
            launch("franka_sim", "franka_plate_sim")

            t0 = time.time()
            while time.time() - t0 < SETTLE_TIMEOUT:
                if monitor.is_settled():
                    settled = True
                    break
                crashed = _crashed_procs(procs)
                if crashed:
                    print(f"[trial {trial_number} run {run_idx}] "
                          f"crash during setup: {crashed} -> discarding episode")
                    return None
                time.sleep(0.05)

            if settled:
                break
            kill_all()
            time.sleep(0.5)

        if not settled:
            print(f"[trial {trial_number} run {run_idx}] failed to settle")
            return FAILED_SETTLE_PENALTY

        # The procman workflow gives the setup a little extra time before
        # switching over to the hybrid controller. That grace period matters
        # when a process is close to initialization boundaries.
        time.sleep(HYBRID_STARTUP_DELAY)
        crashed = _crashed_procs(procs)
        if crashed:
            print(f"[trial {trial_number} run {run_idx}] "
                  f"crash after settle: {crashed} -> discarding episode")
            return None

        # --- 2) run hybrid mpc controller ---
        launch("hybrid_mpc_controller", "franka_hybrid_mpc_controller",
               args=["--hybrid_mpc_options_file="
                     "examples/iC3/plate/parameters/"
                     "plate_hybrid_mpc_options_optuna.yaml"])

        # --- 3) run for the horizon ---
        monitor.clear()
        t0 = time.time()
        while time.time() - t0 < RUN_TIME:
            crashed = _crashed_procs(procs)
            if crashed:
                print(f"[trial {trial_number} run {run_idx}] "
                      f"crash during run: {crashed} -> discarding episode")
                return None
            time.sleep(0.05)

        # --- 4) score this episode ---
        score = compute_objective(monitor)
        print(f"[trial {trial_number} run {run_idx}] score = {score:.4f}")
        return score

    finally:
        kill_all()
        monitor.stop()
        time.sleep(1.0)   # let multicast buffers drain before next episode


def _run_episode_with_retries(trial_number, run_idx, env, logdir):
    """Retry a crashed episode until we get a clean score or hit a limit."""
    for episode_attempt in range(1, MAX_EPISODE_RETRIES + 1):
        try:
            score = _run_episode(trial_number, run_idx, env, logdir)
        except Exception as e:
            print(f"[trial {trial_number} run {run_idx}] "
                  f"episode attempt {episode_attempt} raised {type(e).__name__}: {e}")
            score = None

        if score is not None:
            return score

        print(f"[trial {trial_number} run {run_idx}] "
              f"retrying episode attempt {episode_attempt}/{MAX_EPISODE_RETRIES}")
        time.sleep(EPISODE_RETRY_DELAY)

    print(f"[trial {trial_number} run {run_idx}] "
          f"giving up after {MAX_EPISODE_RETRIES} failed episode attempts")
    return CRASH_PENALTY


def run_trial(params, trial_number, trial):
    # ---- write hyperparameters into the controller param files (once) ----
    with open(MPC_PARAMS, "r") as f:
        mpc_params = yaml.load(f, Loader=yaml.FullLoader)
    mpc_params["w_R"] = params["w_R"]
    mpc_params["q_vector"][0] = params["xy_cost"]
    mpc_params["q_vector"][1] = params["xy_cost"]
    mpc_params["q_vector"][2] = params["z_cost"]
    mpc_params["q_vector"][3] = params["rot_cost"]
    mpc_params["q_vector"][4] = params["rot_cost"]
    mpc_params["quaternion_weight"] = params["quat_weight"]
    with open(MPC_PARAMS, "w") as f:
        yaml.dump(mpc_params, f, default_flow_style=True)

    with open(OSC_PARAMS, "r") as f:
        osc_params = yaml.load(f, Loader=yaml.FullLoader)
    Kp_xy, Kd_xy = params["Kp_xy"], params["Kd_xy"]
    Kp_z, Kd_z = params["Kp_z"], params["Kd_z"]
    osc_params["EndEffectorKp"] = [Kp_xy, 0, 0, 0, Kp_xy, 0, 0, 0, Kp_z]
    osc_params["EndEffectorKd"] = [Kd_xy, 0, 0, 0, Kd_xy, 0, 0, 0, Kd_z]
    Kp_rot, Kd_rot = params["Kp_rot"], params["Kd_rot"]
    osc_params["EndEffectorRotKp"] = [Kp_rot, 0, 0, 0, Kp_rot, 0, 0, 0, Kp_rot]
    osc_params["EndEffectorRotKd"] = [Kd_rot, 0, 0, 0, Kd_rot, 0, 0, 0, Kd_rot]
    with open(OSC_PARAMS, "w") as f:
        yaml.dump(osc_params, f,  default_flow_style=True)

    # ---- shared env ----
    env = os.environ.copy()
    env["LCM_DEFAULT_URL"] = LCM_URL

    # REMOVED logdir CREATION HERE

    total = 0.0
    for run_idx in range(N_RUNS):
        # Pass None instead of logdir
        s = _run_episode_with_retries(trial_number, run_idx, env, None)
        total += s

    print(f"[trial {trial_number}] TOTAL cost = {total:.4f}  params = {params}")
    return total

# ======================================================================
# Optuna driver
# ======================================================================
def objective(trial):
    params = {
        "w_R": trial.suggest_int("w_R", 10, 1000, step=10),
        "xy_cost": trial.suggest_int("xy_cost", 500, 50000, step=500),
        "z_cost": trial.suggest_int("z_cost", 500, 100000, step=500),
        "rot_cost": trial.suggest_int("rot_cost", 500, 25000, step=500),
        "quat_weight": trial.suggest_int("quat_weight", 50, 1000, step=50),
        "Kp_xy": trial.suggest_int("Kp_xy", 20, 500, step=20),
        "Kd_xy": trial.suggest_int("Kd_xy", 5, 100, step=5),
        "Kp_z": trial.suggest_int("Kp_z", 20, 800, step=20),
        "Kd_z": trial.suggest_int("Kd_z", 5, 200, step=5),
        "Kp_rot": trial.suggest_int("Kp_rot", 20, 800, step=20),
        "Kd_rot": trial.suggest_int("Kd_rot", 2, 100, step=2), 
    }
    return run_trial(params, trial.number, trial)


def log_best_callback(study, trial):
    """
    This runs automatically after every trial. 
    It checks if the trial that just finished is the new best trial.
    """
    try: 
      # Check if the trial that just ended is the absolute best one so far
      if study.best_trial.number == trial.number:
          print(f"--> New best metric found: {study.best_value}. Saving to file...")
          
          # Open in "w" (write) mode to overwrite the file with the fresh best data
          with open("examples/iC3/plate/optuna/best_params_plate.txt", "w") as f:
              f.write("=========================================\n")
              f.write("       BEST HYPERPARAMETERS SO FAR       \n")
              f.write("=========================================\n")
              f.write(f"Best Trial Number: {study.best_trial.number}\n")
              f.write(f"Best Metric Value: {study.best_value}\n\n")
              f.write("Parameters:\n")
              
              for key, value in study.best_params.items():
                  f.write(f"  {key}: {value}\n")
              f.write("=========================================\n")
    except ValueError:
        print(f"Initial trial completed. Trial {trial.number} with value: {trial.value}")


def main():
    optuna.logging.set_verbosity(optuna.logging.DEBUG)

    STORAGE_URL = "sqlite:///examples/iC3/plate/optuna/optuna_plate.db"
    sampler = optuna.samplers.TPESampler(multivariate=True)

    pruner = optuna.pruners.MedianPruner(
        n_startup_trials=10,   # collect 10 full trials before pruning anything
        n_warmup_steps=4,      # don't prune until at least 4 episodes have run
    )

    study = optuna.create_study(
        direction="minimize",
        study_name="optuna_plate",
        storage=STORAGE_URL,   
        load_if_exists=True,
        sampler=sampler,
        pruner=pruner
    )
    study.optimize(objective, 
                   n_trials=N_TRIALS, 
                   callbacks=[log_best_callback],
                   catch=(Exception,), )

    print("\n--- Optimization Complete ---")
    print(f"Best Trial Value: {study.best_value}")
    print("Best Hyperparameters:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")


# python3 examples/iC3/plate/optuna_franka_plate.py

if __name__ == "__main__":
    # graceful Ctrl-C: make sure we don't leave zombies
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted -- cleaning up any surviving processes...")
        sys.exit(1)
