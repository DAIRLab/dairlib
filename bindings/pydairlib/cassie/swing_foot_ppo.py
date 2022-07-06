import numpy as np
import torch
import gym
from stable_baselines3 import PPO
from datetime import datetime

from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.radio_swing_foot_env import RadioSwingFootEnv, \
    make_radio_swing_ft_env
from pydairlib.cassie.cassie_gym.swing_foot_env import make_swing_ft_env_fn

def main():
    # make a vec env for the radio swing foot env
    env = make_vec_env(make_radio_swing_ft_env, 1, 42)
    # env = make_vec_env(make_swing_ft_env_fn, 8, 42)
    # create a logger
    logdir = "./rl_logging/" + datetime.now().strftime("%b_%d_%Y_%H%M") +  "/"
    # create ppo with the swing foot vec env
    model = PPO("MlpPolicy",
                env,
                verbose=1,
                tensorboard_log=logdir,
                n_steps=32,
                batch_size=32)

    # run ppo with logger
    model.learn(10000)
    

if __name__ == "__main__":
    main()
