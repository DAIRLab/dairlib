import numpy as np
import torch
import gym
from stable_baselines3 import PPO
from datetime import datetime
import os
import argparse

from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.footstep_env import CassieFootstepEnv, \
    make_footstep_env_fn

# TODO(hersh500): save model checkpoints, 
# also implement yaml configs for these so I remember experiment settings.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval", type=bool, default=False)
    parser.add_argument("--path", type=str)
    parser.add_argument("--n_envs", type=int)
    parser.add_argument("--pretrained", type=bool, default=False)
    args = parser.parse_args()
    to_eval = args.eval

    if to_eval:
        directory = args.path
        model = PPO.load(directory)
        # eval_env = make_vec_env(make_radio_swing_ft_env, 1, 10)
        eval_env = CassieFootstepEnv(visualize=True, debug=True, blind=False)
        for ep in range(10):
            state = eval_env.reset()
            while True:
                action, _states = model.predict(state)
                state, r, d, i = eval_env.step(action)
                if d:
                    break
    else:    
        n_envs = args.n_envs
        pretrained = args.pretrained
        logdir = "./footstep_rl_logging/" + datetime.now().strftime("%b_%d_%Y_%H%M") +  "/"
        if pretrained:
            directory = args.path
            model = PPO.load(directory)
            env = make_vec_env(make_footstep_env_fn, n_envs, 50, max_step_magnitude=0.05, blind=False, terrain_randomize=True, reward_weights=[0.2,-0.4])
            model.set_env(env)
        else:
            env = make_vec_env(make_footstep_env_fn, n_envs, 50, max_step_magnitude=0.05, blind=False, terrain_randomize=True, reward_weights=[0.2,-0.4])
            model = PPO("MlpPolicy",
                        env,
                        verbose=1,
                        tensorboard_log=logdir,
                        n_steps=int(5096/n_envs),
                        batch_size=1024)
        eval_env = make_vec_env(make_footstep_env_fn, 1, 30, blind=False)
        try:
            model.learn(1.5e6, eval_env=eval_env, eval_freq=256)
        except KeyboardInterrupt:
            pass
        model.save(os.path.join(logdir, "ppo_model"))
        print("bye bye")


if __name__ == "__main__":
    main()
