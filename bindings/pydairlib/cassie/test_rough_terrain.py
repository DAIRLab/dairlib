import numpy as np
import torch
import gym
import matplotlib.pyplot as plt
import argparse

import pydairlib.cassie.cassie_gym.cassie_env_state as cassie_env_state
from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.radio_swing_foot_env import RadioSwingFootEnv, \
    SWING_FOOT_ACTION_DIM, make_radio_swing_ft_env
from pydairlib.cassie.cassie_gym.swing_foot_env import make_swing_ft_env_fn
from pydairlib.cassie.cassie_gym.high_level_reward import HighLevelReward
from scipy.spatial.transform import Rotation as R

def calc_stability_metrics(state):
    quat = state[cassie_env_state.CASSIE_QUATERNION_SLICE]
    euler = R.from_quat(quat).as_euler("xyz")  # roll pitch yaw
    # return roll, pitch magnitude
    return np.abs(euler[0]), np.abs(euler[1])
    

# Probably best to log the states instead of metrics, so I can plot more
# metrics post hoc rather than having to re-gather the data 
def main(magnitude):
    # fwd_radio_cmds = [0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.8, 1.0]
    fwd_radio_cmds = [0.5, 0.8, 1.0]
    num_rollouts = 10
    num_steps_per_rollout = 15
    avg_angles = np.zeros((len(fwd_radio_cmds), 2))
    n_envs = 1
    for idx, cmd in enumerate(fwd_radio_cmds):
        print(f"on cmd {cmd}")
        avg_roll, avg_pitch = 0, 0
        for i in range(num_rollouts):
            # regenerate environment for different terrain
            # env = RadioSwingFootEnv(HighLevelReward(0.0), visualize=True, max_step_magnitude=magnitude, goal=2.0)
            env = make_vec_env(make_radio_swing_ft_env, n_envs, 42, visualize=True, magnitude=magnitude, goal=2.0)
            s = env.reset()
            for step in range(num_steps_per_rollout):
                action = np.zeros((n_envs, SWING_FOOT_ACTION_DIM + 3))
                action[:,0] = cmd
                s, r, d, infos = env.step(action)

                rolls, pitches = [], []
                for info in infos:
                    rolls.append(info["total_roll_error"])
                    pitches.append(info["total_pitch_error"])
                roll = np.mean(rolls)
                pitch = np.mean(pitches)

                if np.any(d):  # this is super inefficient as it necessitates resetting all of them.
                    s = env.reset()  # should reset to all new seeds for new data collection
                avg_roll += roll
                avg_pitch += pitch

        # might be some subtleties here in terms of the units of what I'm averaging.
        # roll is currently rad/s
        avg_roll = avg_roll/(num_steps_per_rollout * num_rollouts)
        avg_pitch = avg_pitch/(num_steps_per_rollout * num_rollouts)
        avg_angles[idx][0] = avg_roll
        avg_angles[idx][1] = avg_pitch

    np.save(f"angles_wrt_gains_{magnitude}.npy", avg_angles)
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(fwd_radio_cmds, avg_angles[:,0]) 
    axs[0].set_title("Roll")
    axs[1].plot(fwd_radio_cmds, avg_angles[:,1]) 
    axs[1].set_title("Pitch")
    axs[1].set_xlabel("Fwd radio command")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser("script to evaluate body stability on rough terrains")
    parser.add_argument("--file", type=str)
    parser.add_argument("--mag", type=float, default=0.0)
    args = parser.parse_args()
    fname = args.file
    mag = args.mag
    if fname is None:
        main(mag)
    else:
        fwd_radio_cmds = [0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.8, 1.0]
        avg_angles = np.load(fname)
        fig, axs = plt.subplots(2, 1)
        axs[0].plot(fwd_radio_cmds, avg_angles[:,0])
        axs[0].set_title("Roll")
        axs[1].plot(fwd_radio_cmds, avg_angles[:,1]) 
        axs[1].set_title("Pitch")
        axs[1].set_xlabel("Fwd radio command")
        fig.tight_layout()
        plt.show()
