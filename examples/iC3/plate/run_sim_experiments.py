#!/usr/bin/env python3
"""
Run batch simulation experiments with Gaussian noise on the object's initial position.

Workflow:
1. Start background LCM logging via start_logging.py.
2. For each run N:
   a) Sample delta_x, delta_y ~ N(0, std^2) and update q_init_object in franka_plate_sim_params.yaml
   b) Run standard simulation + OSC settling, followed by hybrid MPC controller.
   c) Record the final object pose and append to a CSV file.
3. Stop logging and restore original simulation parameters.
"""
import argparse
import atexit
import csv
import os
import signal
import subprocess
import sys
import time
import tempfile
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
from dairlib import lcmt_osc_output

# ======================================================================
# Config & Paths
# ======================================================================
REPO = DAIRLIB_DIR
BIN = os.path.join(REPO, "bazel-bin/examples/iC3/plate")
LCM_URL = "udpm://239.255.76.67:7667?ttl=0"

# Timing configuration
SETTLE_TIMEOUT = 2.0        # seconds to wait for settling per attempt
SETTLE_ATTEMPTS = 5         # retry setup if not settled
RUN_TIME = 5.0              # seconds to run hybrid MPC controller
SETTLE_POS_TOL = 0.02       # 1 cm tolerance
SETTLE_HOLD = 1.5          # seconds it must hold within tolerance
HYBRID_STARTUP_DELAY = 0.25 # seconds before launching hybrid MPC
EPISODE_RETRY_DELAY = 0.25  # seconds before retrying crashed episode
MAX_EPISODE_RETRIES = 5

# Hardcoded experiment parameters
NUM_RUNS = 10               # Number of experiment runs per batch
STD_X = 0.002                # Gaussian noise std in X (meters)
STD_Y = 0.002                # Gaussian noise std in Y (meters)
LOG_FOLDER = "/mnt/iC3_logs/sim_data/plate/"
OUTPUT_DIR = "examples/iC3/plate/sim_experiment_results"

# Standard parameter file paths (must be relative for FindResourceOrThrow in dairlib binaries)
SIM_PARAMS = os.path.join(REPO, "examples/iC3/plate/parameters/franka_plate_sim_params.yaml")
OSC_PARAMS = "examples/iC3/plate/parameters/franka_plate_osc_controller_params.yaml"
MPC_PARAMS = "examples/iC3/plate/parameters/plate_hybrid_mpc_options.yaml"


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
        pos = np.array(msg.position[:7])   # [qw, qx, qy, qz, x, y, z]
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

    def is_settled(self, target_xyz):
        """True if the object's xyz has stayed within tolerance of target_xyz for SETTLE_HOLD window."""
        now = time.time()
        with self.lock:
            recent = [(t, v) for (t, v) in self.obj_buf if now - t <= SETTLE_HOLD]

        if not recent:
            return False

        for (_, pos) in recent:
            xyz = pos[4:7]
            if np.linalg.norm(xyz - target_xyz) > SETTLE_POS_TOL:
                return False
        return True

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
            self.ee_buf.clear()

    def stop(self):
        self._stop = True
        self.thread.join(timeout=1.0)
        if hasattr(self, 'lc'):
            del self.lc


# ======================================================================
# Process Management
# ======================================================================
class Proc:
    def __init__(self, name, exec_path, env, args=None, logdir=None):
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


TARGET_QUAT = np.array([0.0, 0.0, -1.0, 0.0])


def body_z_axis(quat):
    """World-frame direction of the body z-axis, given quat = [w, x, y, z]."""
    w, x, y, z = quat
    return np.array([
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ])


def z_axis_error(quat, target_quat=TARGET_QUAT):
    """Angle (radians) between current and target body z-axes.
    0 = aligned, pi = fully flipped. Ignores spin about z."""
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


# ======================================================================
# Single Episode Execution
# ======================================================================
def _run_single_episode(trial_idx, settle_target_xyz, env):
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
        # 1. Setup & Settling
        settled = False
        for attempt in range(SETTLE_ATTEMPTS):
            monitor.clear()
            launch("franka_osc", "franka_plate_osc_controller",
                   args=["--controller_parameters=" + OSC_PARAMS])
            launch("franka_sim", "franka_plate_sim")

            t0 = time.time()
            while time.time() - t0 < SETTLE_TIMEOUT:
                if monitor.is_settled(settle_target_xyz):
                    settled = True
                    break
                crashed = _crashed_procs(procs)
                if crashed:
                    print(f"[{trial_idx}] Crash during setup attempt {attempt + 1}: {crashed}")
                    return None
                time.sleep(0.05)

            if settled:
                break
            kill_all()
            time.sleep(0.5)

        if not settled:
            print(f"[{trial_idx}] Failed to settle after {SETTLE_ATTEMPTS} attempts")
            return {
                "settled": False,
                "crashed": False,
                "quat": np.zeros(4),
                "xyz": np.zeros(3),
            }

        time.sleep(HYBRID_STARTUP_DELAY)
        crashed = _crashed_procs(procs)
        if crashed:
            print(f"[{trial_idx}] Crash after settle: {crashed}")
            return None

        # 2. Run Hybrid MPC Controller
        launch("hybrid_mpc_controller", "franka_hybrid_mpc_controller",
               args=["--hybrid_mpc_options_file=" + MPC_PARAMS])

        # 3. Horizon Execution
        monitor.clear()
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
                "settled": True,
                "crashed": False,
                "quat": np.zeros(4),
                "xyz": np.zeros(3),
            }

        quat, xyz = result
        return {
            "settled": True,
            "crashed": False,
            "quat": quat,
            "xyz": xyz,
        }

    finally:
        kill_all()
        monitor.stop()
        time.sleep(1.0)


def run_episode_with_retries(trial_idx, settle_target_xyz, env):
    for attempt in range(1, MAX_EPISODE_RETRIES + 1):
        try:
            res = _run_single_episode(trial_idx, settle_target_xyz, env)
        except Exception as e:
            print(f"[{trial_idx}] Exception during attempt {attempt}: {e}")
            res = None

        if res is not None:
            return res

        print(f"[{trial_idx}] Retrying episode ({attempt}/{MAX_EPISODE_RETRIES})...")
        time.sleep(EPISODE_RETRY_DELAY)

    print(f"[{trial_idx}] Episode failed after max retries")
    return {
        "settled": False,
        "crashed": True,
        "quat": np.zeros(4),
        "xyz": np.zeros(3),
    }


# ======================================================================
# Main Experiment Runner
# ======================================================================
def main():
    parser = argparse.ArgumentParser(description="Run batch simulation experiments with noisy initial object position.")
    parser.add_argument("-i", "--exp_idx", "--index", type=int, default=None, help="Experiment/batch index to identify this run (e.g. 0, 1, 2)")
    parser.add_argument("-n", "--num_runs", type=int, default=10, help="Number of experiment runs (default: 10)")
    parser.add_argument("--std_x", type=float, default=0.01, help="Gaussian noise standard deviation in X (meters, default: 0.01)")
    parser.add_argument("--std_y", type=float, default=0.01, help="Gaussian noise standard deviation in Y (meters, default: 0.01)")
    parser.add_argument("--output_dir", type=str, default="examples/iC3/plate/sim_experiment_results",
                        help="Directory to store results (default: examples/iC3/plate/sim_experiment_results)")
    parser.add_argument("--output_csv", type=str, default=None,
                        help="Explicit CSV file path (overrides output_dir and exp_idx)")
    parser.add_argument("--log_folder", type=str, default="/mnt/iC3_logs/sim_data/plate/",
                        help="Target directory for LCM logs (default: /mnt/iC3_logs/sim_data/plate/)")
    parser.add_argument("--no_log", action="store_true", help="Disable start_logging background process")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for Gaussian noise")
    args = parser.parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    # Determine CSV output path
    if args.output_csv is not None:
        output_csv = args.output_csv
    else:
        idx_str = f"_{args.exp_idx:02d}" if args.exp_idx is not None else ""
        output_csv = os.path.join(args.output_dir, f"sim_experiments_results{idx_str}.csv")

    # 1. Read and preserve base simulation parameters
    with open(SIM_PARAMS, "r") as f:
        original_sim_yaml_text = f.read()
    sim_params = yaml.safe_load(original_sim_yaml_text)

    base_q_init_object = list(sim_params["q_init_object"])
    base_x = base_q_init_object[4]
    base_y = base_q_init_object[5]
    base_z = base_q_init_object[6]

    print("==================================================")
    print("           SIMULATION EXPERIMENTS RUNNER          ")
    print("==================================================")
    if args.exp_idx is not None:
        print(f"Exp Index:   {args.exp_idx}")
    print(f"Num Runs:    {args.num_runs}")
    print(f"Noise std_x: {args.std_x:.4f} m, std_y: {args.std_y:.4f} m")
    print(f"Base Pos:    x={base_x:.4f}, y={base_y:.4f}, z={base_z:.4f}")
    print(f"Output CSV:  {output_csv}")
    print("==================================================\n")

    # Ensure sim_params is restored on exit
    def restore_sim_params():
        try:
            with open(SIM_PARAMS, "w") as f:
                f.write(original_sim_yaml_text)
            print("[Clean-up] Restored original sim parameters.")
        except Exception as e:
            print(f"[Clean-up] Error restoring sim parameters: {e}")

    atexit.register(restore_sim_params)
    atexit.register(_reap_all)

    # 2. Start LCM Logging if enabled
    logger_proc = None
    env = os.environ.copy()
    env["LCM_DEFAULT_URL"] = LCM_URL

    if not args.no_log:
        print("[Logging] Starting LCM logger via start_logging.py...")
        logger_cmd = [sys.executable, "examples/iC3/start_logging.py", "sim", args.log_folder]
        logger_p = subprocess.Popen(
            logger_cmd, cwd=REPO, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True
        )
        logger_proc = logger_p
        time.sleep(1.0)  # let logger initialize

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
        "init_x",
        "init_y",
        "init_z",
        "final_x",
        "final_y",
        "final_z",
        "final_qw",
        "final_qx",
        "final_qy",
        "final_qz",
        "z_axis_diff",
        "settled",
        "crashed"
    ]

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_fields)
        writer.writeheader()
        csvfile.flush()

        # 4. Run trials
        for trial_idx in range(1, args.num_runs + 1):
            delta_x = float(np.random.normal(0.0, args.std_x))
            delta_y = float(np.random.normal(0.0, args.std_y))

            init_x = base_x + delta_x
            init_y = base_y + delta_y
            init_z = base_z

            # Update q_init_object in sim_params
            sim_params["q_init_object"] = [
                base_q_init_object[0],
                base_q_init_object[1],
                base_q_init_object[2],
                base_q_init_object[3],
                init_x,
                init_y,
                init_z
            ]

            with open(SIM_PARAMS, "w") as f:
                yaml.dump(sim_params, f, default_flow_style=None)

            # Settle target has the perturbed xy and settled plate height (0.47m)
            settle_target = np.array([init_x, init_y, 0.47])

            print(f"\n--- [Run {trial_idx}/{args.num_runs}] delta_x: {delta_x:+.4f}, delta_y: {delta_y:+.4f} ---")
            print(f"    Initial Object Pose: ({init_x:.4f}, {init_y:.4f}, {init_z:.4f})")

            res = run_episode_with_retries(trial_idx, settle_target, env)

            final_xyz = res["xyz"]
            final_quat = res["quat"]
            z_diff = z_axis_error(final_quat)

            print(f"    Final Object Pos: ({final_xyz[0]:.4f}, {final_xyz[1]:.4f}, {final_xyz[2]:.4f})")
            print(f"    Z-axis Diff to Target: {z_diff:.4f} rad | Settled: {res['settled']} | Crashed: {res['crashed']}")

            row = {
                "trial_idx": trial_idx,
                "delta_x": delta_x,
                "delta_y": delta_y,
                "init_x": init_x,
                "init_y": init_y,
                "init_z": init_z,
                "final_x": float(final_xyz[0]),
                "final_y": float(final_xyz[1]),
                "final_z": float(final_xyz[2]),
                "final_qw": float(final_quat[0]),
                "final_qx": float(final_quat[1]),
                "final_qy": float(final_quat[2]),
                "final_qz": float(final_quat[3]),
                "z_axis_diff": float(z_diff),
                "settled": res["settled"],
                "crashed": res["crashed"]
            }
            writer.writerow(row)
            csvfile.flush()

    print("\n==================================================")
    print(f"All {NUM_RUNS} trials complete. Results saved to {output_csv}")
    print("==================================================")

# examples/iC3/plate/run_sim_experiments.py

if __name__ == "__main__":
    def signal_handler(signum, frame):
        print(f"\n[Interrupted by signal {signum}] Cleaning up and exiting...")
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    main()
