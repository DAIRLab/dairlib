import numpy as np
import torch
import gym
from stable_baselines3 import PPO
from datetime import datetime
import os
import argparse

from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.radio_swing_foot_env import RadioSwingFootEnv, \
    make_radio_swing_ft_env
from pydairlib.cassie.cassie_gym.swing_foot_env import make_swing_ft_env_fn
from pydairlib.cassie.cassie_gym.high_level_reward import HighLevelReward

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval", type=bool, default=False)
    parser.add_argument("--path", type=str)
    args = parser.parse_args()
    to_eval = args.eval

    if not to_eval:
        print("Training....")
        # make a vec env for the radio swing foot env
        # env = make_vec_env(make_radio_swing_ft_env, 15, 50, visualize=False)
        env = RadioSwingFootEnv(HighLevelReward(0.5), visualize=True)
        eval_env = make_vec_env(make_radio_swing_ft_env, 1, 30, visualize=False)
        # create a logger
        logdir = "./rl_logging/" + datetime.now().strftime("%b_%d_%Y_%H%M") +  "/"
        # create ppo with the swing foot vec env
        model = PPO("MlpPolicy",
                    env,
                    verbose=1,
                    tensorboard_log=logdir,
                    n_steps=128,
                    batch_size=64)

        # run ppo with logger
        try:
            model.learn(100000, eval_env=eval_env, eval_freq=100)
            model.save(os.path.join(logdir, "ppo_model"))
        except KeyboardInterrupt:
            # env.env_method("save_action_log", 0) 
            # env.save_action_log()
            model.save(os.path.join(logdir, "ppo_model"))
    else:
        directory = args.path
        model = PPO.load(directory)
        # eval_env = make_vec_env(make_radio_swing_ft_env, 1, 10)
        eval_env = RadioSwingFootEnv(HighLevelReward(0.5), visualize=True)
        for ep in range(10):
            state = eval_env.reset()
            while True:
                action, _states = model.predict(state)
                state, r, d, i = eval_env.step(action)
                if d:
                    break 


if __name__ == "__main__":
    main()
