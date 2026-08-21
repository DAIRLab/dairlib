#!/usr/bin/env python3
"""
Run batch simulation experiments for TriFinger 180 yaw rotation task.

Features:
- No settling phase: launches sim, osc, and 180 hybrid MPC controller directly.
- Noise on initial object position (x, y) and yaw angle.
- Configures OSC neutral_position and LambdaEndEffectorW (100 on diagonal).
- Starts background LCM logging to /mnt/iC3_logs/sim_data/trifinger_180.
- Saves results to examples/iC3/trifinger/sim_experiment_results_180/sim_experiments_results_180_<idx>.csv.
- Restores original YAML parameter files on exit.
"""
import argparse
import atexit
import csv
import os
import signal
import subprocess
import sys
import tempfile
import time
import threading
from collections import deque

import numpy as np
import yaml
import lcm

DAIRLIB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_ALL_PROCS = []


def _prepend_if_exists(path):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


# Add repo-local generated LCM bindings
_prepend_if_exists(os.path.join(DAIRLIB_DIR, "bazel-bin", "lcmtypes"))

from dairlib import lcmt_object_state

# ======================================================================
# Config & Paths
# ======================================================================
REPO = DAIRLIB_DIR
BIN = os.path.join(REPO, "bazel-bin/examples/iC3/trifinger")
LCM_URL = "udpm://239.255.76.67:7667?ttl=0"

# Timing configuration
RUN_TIME = 5.0              # seconds to run hybrid MPC controller per trial
STARTUP_DELAY = 0.3         # seconds before launching hybrid MPC after sim/osc
EPISODE_RETRY_DELAY = 0.25  # seconds before retrying crashed episode
MAX_EPISODE_RETRIES = 5

# Hardcoded experiment parameters
NUM_RUNS = 10               # Number of experiment runs per batch
STD_X = 0.003               # Gaussian noise std in X (meters)
STD_Y = 0.003               # Gaussian noise std in Y (meters)
STD_YAW = 0.05              # Gaussian noise std in yaw (radians, ~3 deg)
LOG_FOLDER = "/mnt/iC3_logs/sim_data/trifinger_180"
OUTPUT_DIR = os.path.join(REPO, "examples/iC3/trifinger/sim_experiment_results_180")

# Task specific target orientation (180 yaw about z: [w=0, x=0, y=0, z=1])
TARGET_QUAT = np.array([0.0, 0.0, 0.0, 1.0])
TARGET_XYZ = np.array([0.0, 0.0, 0.052])
EXAMPLE_IDX = 1

# Task specific OSC parameters for 180
OSC_NEUTRAL_POS = [
    [0.0, 0.07, 0.05],
    [0.07, -0.055, 0.05],
    [-0.07, -0.055, 0.05]
]
OSC_LAMBDA_W_DIAG = 100.0

# Parameter file paths
SIM_PARAMS = os.path.join(REPO, "examples/iC3/trifinger/parameters/trifinger_sim_params.yaml")
OSC_PARAMS = os.path.join(REPO, "examples/iC3/trifinger/parameters/trifinger_osc_controller_params.yaml")


def _crashed_procs(procs):
    dead = []
    for p in procs:
        rc = p.p.poll()
        if rc is not None and rc != 0:
            err = p.get_stderr()
            dead.append(f"{p.name} (exit {rc})\n--- stderr ---\n{err.strip()}\n--------------" if err else f"{p.name} (exit {rc})")
    return dead


def _reap_all():
    for p in list(_ALL_PROCS):
        try:
            if p.p.poll() is None:
                os.killpg(os.getpgid(p.p.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
    _ALL_PROCS.clear()


# ======================================================================
# LCM Monitor
# ======================================================================
class Monitor:
    def __init__(self, lcm_url):
        self.lc = lcm.LCM(lcm_url)
        self.lock = threading.Lock()
        self.obj_buf = deque(maxlen=500)   # (t, pos[7])
        self._stop = False

        self.lc.subscribe("OBJECT_STATE_SIMULATION", self._on_object)
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
        pos = np.array(msg.position[:7])   # [qw, qx, qy, qz, x, y, z]
        with self.lock:
            self.obj_buf.append((time.time(), pos))

    def object_final_state(self, tail=20):
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

    def stop(self):
        self._stop = True
        self.thread.join(timeout=1.0)
        if hasattr(self, 'lc'):
            del self.lc


# ======================================================================
# Process Management
# ======================================================================
class Proc:
    def __init__(self, name, exec_path, env, args=None):
        self.name = name
        self.stderr_file = tempfile.TemporaryFile(mode="w+t")
        cmd = [exec_path] + (args or [])
        self.p = subprocess.Popen(
            cmd, cwd=REPO, env=env,
            stdout=subprocess.DEVNULL,
            stderr=self.stderr_file,
            start_new_session=True,
        )
        _ALL_PROCS.append(self)

    def alive(self):
        return self.p.poll() is None

    def get_stderr(self):
        try:
            self.stderr_file.seek(0)
            return self.stderr_file.read()
        except Exception:
            return ""

    def kill(self):
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
                time.sleep(0.3)
                if self.p.poll() is None:
                    os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            self.stderr_file.close()
        except Exception:
            pass
        if self in _ALL_PROCS:
            _ALL_PROCS.remove(self)


# ======================================================================
# Orientation Metrics
# ======================================================================
def body_z_axis(quat):
    w, x, y, z = quat
    return np.array([
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ])


def z_axis_error(quat, target_quat=TARGET_QUAT):
    z_cur = body_z_axis(quat)
    z_tgt = body_z_axis(target_quat)
    norm_cur = np.linalg.norm(z_cur)
    norm_tgt = np.linalg.norm(z_tgt)
    if norm_cur == 0 or norm_tgt == 0:
        return 0.0
    z_cur = z_cur / norm_cur
    z_tgt = z_tgt / norm_tgt
    d = np.clip(np.dot(z_cur, z_tgt), -1.0, 1.0)
    return float(np.arccos(d))


def quat_angular_distance(q1, q2=TARGET_QUAT):
    dot = min(1.0, abs(float(np.dot(q1, q2))))
    return float(2.0 * np.arccos(dot))


# ======================================================================
# Single Episode Execution (No settling)
# ======================================================================
def _run_single_episode(trial_idx, env):
    monitor = Monitor(LCM_URL)
    procs = []

    def launch(name, binary, args=None):
        p = Proc(name, os.path.join(BIN, binary), env, args=args)
        procs.append(p)
        return p

    def kill_all():
        for p in reversed(procs):
            p.kill()
        procs.clear()

    try:
        monitor.clear()
        # 1. Launch Sim & OSC
        launch("trifinger_osc", "trifinger_osc_controller")
        launch("trifinger_sim", "trifinger_sim")

        time.sleep(STARTUP_DELAY)
        crashed = _crashed_procs(procs)
        if crashed:
            print(f"[{trial_idx}] Crash during startup: {crashed}")
            return None

        # 2. Launch Hybrid MPC Controller
        launch("trifinger_180_hybrid_mpc_controller", "trifinger_hybrid_mpc_controller",
               args=[f"--example_idx={EXAMPLE_IDX}"])

        # 3. Horizon Execution
        t0 = time.time()
        while time.time() - t0 < RUN_TIME:
            crashed = _crashed_procs(procs)
            if crashed:
                print(f"[{trial_idx}] Crash during MPC execution: {crashed}")
                return None
            time.sleep(0.05)

        # 4. Extract Final State
        result = monitor.object_final_state()
        if result is None:
            return {
                "crashed": False,
                "quat": np.zeros(4),
                "xyz": np.zeros(3),
            }

        quat, xyz = result
        return {
            "crashed": False,
            "quat": quat,
            "xyz": xyz,
        }

    finally:
        kill_all()
        monitor.stop()
        time.sleep(1.0)


def run_episode_with_retries(trial_idx, env):
    for attempt in range(1, MAX_EPISODE_RETRIES + 1):
        try:
            res = _run_single_episode(trial_idx, env)
        except Exception as e:
            print(f"[{trial_idx}] Exception during attempt {attempt}: {e}")
            res = None

        if res is not None:
            return res

        print(f"[{trial_idx}] Retrying episode ({attempt}/{MAX_EPISODE_RETRIES})...")
        time.sleep(EPISODE_RETRY_DELAY)

    print(f"[{trial_idx}] Episode failed after max retries")
    return {
        "crashed": True,
        "quat": np.zeros(4),
        "xyz": np.zeros(3),
    }


# ======================================================================
# Main Experiment Runner
# ======================================================================
def main():
    parser = argparse.ArgumentParser(description="Run TriFinger 180 simulation experiments with noisy initial object pose.")
    parser.add_argument("exp_idx", type=int, nargs="?", default=None, help="Experiment/batch index (e.g. 0, 1, 2)")
    parser.add_argument("-i", "--index", "--exp_idx", type=int, default=None, dest="opt_exp_idx", help="Experiment/batch index")
    parser.add_argument("--no_log", action="store_true", help="Disable start_logging background process")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for Gaussian noise")
    args = parser.parse_args()

    exp_idx = args.exp_idx if args.exp_idx is not None else args.opt_exp_idx

    if args.seed is not None:
        np.random.seed(args.seed)

    idx_str = f"_{exp_idx:02d}" if exp_idx is not None else ""
    output_csv = os.path.join(OUTPUT_DIR, f"sim_experiments_results_180{idx_str}.csv")

    # 1. Read and preserve base simulation and OSC parameters
    with open(SIM_PARAMS, "r") as f:
        original_sim_yaml_text = f.read()
    sim_params = yaml.safe_load(original_sim_yaml_text)

    with open(OSC_PARAMS, "r") as f:
        original_osc_yaml_text = f.read()
    osc_params = yaml.safe_load(original_osc_yaml_text)

    # Set task-specific OSC parameters (neutral_position and LambdaEndEffectorW)
    osc_params["neutral_position"] = OSC_NEUTRAL_POS
    osc_params["LambdaEndEffectorW"] = (np.eye(9) * OSC_LAMBDA_W_DIAG).flatten().tolist()
    with open(OSC_PARAMS, "w") as f:
        yaml.dump(osc_params, f, default_flow_style=None)

    base_q_init_object = list(sim_params["q_init_object"])
    base_x = base_q_init_object[4]
    base_y = base_q_init_object[5]
    base_z = base_q_init_object[6]

    print("==================================================")
    print("      TRIFINGER 180 SIMULATION EXPERIMENTS RUNNER ")
    print("==================================================")
    if exp_idx is not None:
        print(f"Exp Index:   {exp_idx}")
    print(f"Num Runs:    {NUM_RUNS}")
    print(f"Noise std_x: {STD_X:.4f} m, std_y: {STD_Y:.4f} m, std_yaw: {STD_YAW:.4f} rad ({np.rad2deg(STD_YAW):.2f} deg)")
    print(f"Base Pos:    x={base_x:.4f}, y={base_y:.4f}, z={base_z:.4f}")
    print(f"Log Folder:  {LOG_FOLDER}")
    print(f"Output CSV:  {output_csv}")
    print("==================================================\n")

    # Ensure parameters are restored on exit
    def restore_params():
        try:
            with open(SIM_PARAMS, "w") as f:
                f.write(original_sim_yaml_text)
            with open(OSC_PARAMS, "w") as f:
                f.write(original_osc_yaml_text)
            print("[Clean-up] Restored original sim and OSC parameters.")
        except Exception as e:
            print(f"[Clean-up] Error restoring parameters: {e}")

    atexit.register(restore_params)
    atexit.register(_reap_all)

    # 2. Start LCM Logging if enabled
    logger_proc = None
    env = os.environ.copy()
    env["LCM_DEFAULT_URL"] = LCM_URL

    if not args.no_log:
        print(f"[Logging] Starting LCM logger via start_logging.py to {LOG_FOLDER}...")
        logger_cmd = [sys.executable, "examples/iC3/start_logging.py", "sim", LOG_FOLDER]
        logger_p = subprocess.Popen(
            logger_cmd, cwd=REPO, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True
        )
        logger_proc = logger_p
        time.sleep(1.0)

    def stop_logger():
        if logger_proc and logger_proc.poll() is None:
            print("\n[Logging] Stopping LCM logger...")
            try:
                os.killpg(os.getpgid(logger_proc.pid), signal.SIGINT)
                logger_proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(logger_proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    atexit.register(stop_logger)

    # 3. Prepare CSV output
    csv_dir = os.path.dirname(output_csv)
    if csv_dir:
        os.makedirs(csv_dir, exist_ok=True)

    csv_fields = [
        "trial_idx",
        "delta_x",
        "delta_y",
        "delta_yaw",
        "init_x",
        "init_y",
        "init_z",
        "init_qw",
        "init_qx",
        "init_qy",
        "init_qz",
        "final_x",
        "final_y",
        "final_z",
        "final_qw",
        "final_qx",
        "final_qy",
        "final_qz",
        "ori_diff_to_target",
        "z_axis_diff",
        "crashed"
    ]

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_fields)
        writer.writeheader()
        csvfile.flush()

        # 4. Run trials
        for trial_idx in range(1, NUM_RUNS + 1):
            delta_x = float(np.random.normal(0.0, STD_X))
            delta_y = float(np.random.normal(0.0, STD_Y))
            delta_yaw = float(np.random.normal(0.0, STD_YAW))

            init_x = base_x + delta_x
            init_y = base_y + delta_y
            init_z = base_z

            # Rotation by delta_yaw about z-axis
            init_qw = float(np.cos(delta_yaw / 2.0))
            init_qx = 0.0
            init_qy = 0.0
            init_qz = float(np.sin(delta_yaw / 2.0))

            # Update q_init_object in sim_params
            sim_params["q_init_object"] = [
                init_qw,
                init_qx,
                init_qy,
                init_qz,
                init_x,
                init_y,
                init_z
            ]

            with open(SIM_PARAMS, "w") as f:
                yaml.dump(sim_params, f, default_flow_style=None)

            print(f"\n--- [Run {trial_idx}/{NUM_RUNS}] dx: {delta_x:+.4f}, dy: {delta_y:+.4f}, dyaw: {np.rad2deg(delta_yaw):+.2f} deg ---")
            print(f"    Initial Object Pose: ({init_x:.4f}, {init_y:.4f}, {init_z:.4f}) | quat: [{init_qw:.4f}, {init_qx:.4f}, {init_qy:.4f}, {init_qz:.4f}]")

            res = run_episode_with_retries(trial_idx, env)

            final_xyz = res["xyz"]
            final_quat = res["quat"]
            ori_diff = quat_angular_distance(final_quat, TARGET_QUAT)
            z_diff = z_axis_error(final_quat, TARGET_QUAT)

            print(f"    Final Object Pos: ({final_xyz[0]:.4f}, {final_xyz[1]:.4f}, {final_xyz[2]:.4f})")
            print(f"    Ori Diff to Target: {ori_diff:.4f} rad ({np.rad2deg(ori_diff):.2f} deg) | Z-axis Diff: {z_diff:.4f} rad | Crashed: {res['crashed']}")

            row = {
                "trial_idx": trial_idx,
                "delta_x": delta_x,
                "delta_y": delta_y,
                "delta_yaw": delta_yaw,
                "init_x": init_x,
                "init_y": init_y,
                "init_z": init_z,
                "init_qw": init_qw,
                "init_qx": init_qx,
                "init_qy": init_qy,
                "init_qz": init_qz,
                "final_x": float(final_xyz[0]),
                "final_y": float(final_xyz[1]),
                "final_z": float(final_xyz[2]),
                "final_qw": float(final_quat[0]),
                "final_qx": float(final_quat[1]),
                "final_qy": float(final_quat[2]),
                "final_qz": float(final_quat[3]),
                "ori_diff_to_target": float(ori_diff),
                "z_axis_diff": float(z_diff),
                "crashed": res["crashed"]
            }
            writer.writerow(row)
            csvfile.flush()

    print("\n==================================================")
    print(f"All {NUM_RUNS} trials complete. Results saved to {output_csv}")
    print("==================================================")


if __name__ == "__main__":
    def signal_handler(signum, frame):
        print(f"\n[Interrupted by signal {signum}] Cleaning up and exiting...")
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    main()
