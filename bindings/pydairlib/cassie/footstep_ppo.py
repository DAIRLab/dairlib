import numpy as np
import torch
import gym
from stable_baselines3 import PPO
from datetime import datetime
import os
import argparse
import shutil
import yaml

from pydairlib.cassie.cassie_gym.footstep_env_utils import make_vec_env_from_config, \
    make_eval_env_from_config, ProgressCallback, evaluate_policy

# TODO(hersh500): also add action rate to learning config
# TODO(hersh500): consolidate eval callbacks so we're not doing multiple different evals!
# TODO(hersh500): save model checkpoints

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval", type=bool, default=False)
    parser.add_argument("--path", type=str)
    parser.add_argument("--pretrained", type=bool, default=False)
    parser.add_argument("--learning_config", type=str, default="./learning_configs/footstep_ppo.yaml")
    args = parser.parse_args()
    to_eval = args.eval
    learning_config_f = args.learning_config

    # Load params from the yaml file
    with open(learning_config_f, 'r') as f:
        learning_config = yaml.safe_load(f)

    if to_eval:
        directory = args.path
        evaluate_policy(10, learning_config, directory)
    else:    
        pretrained = args.pretrained
        if learning_config["blind"]:
            policy_type = "MlpPolicy"
        else:
            policy_type = "MultiInputPolicy"
        logdir = "./footstep_rl_logging/" + datetime.now().strftime("%b_%d_%Y_%H%M") +  "/"
        if pretrained:
            directory = args.path
            model = PPO.load(directory)
            env = make_vec_env_from_config(learning_config)
            model.set_env(env)
            reset_num_timesteps = False
        else:
            env = make_vec_env_from_config(learning_config)
            model = PPO(policy_type,
                        env,
                        verbose=1,
                        tensorboard_log=logdir,
                        n_steps=learning_config["n_steps_per_update"],
                        batch_size=learning_config["batch_size"])
            reset_num_timesteps = True
        eval_env = make_eval_env_from_config(learning_config)
        progress_callback = ProgressCallback(eval_env, eval_freq=learning_config["eval_freq"], n_eval_episodes=learning_config["n_eval_episodes"], fig_lims=[(-2, 4), (-3, 3)])
        try:
            model.learn(learning_config["total_training_steps"], eval_env=eval_env, eval_freq=learning_config["eval_freq"], n_eval_episodes=learning_config["n_eval_episodes"], reset_num_timesteps=reset_num_timesteps, callback=progress_callback)
        except KeyboardInterrupt:
            pass
        shutil.copyfile(args.learning_config, logdir+"/learning_config.yaml")
        model.save(os.path.join(logdir, "ppo_model"))
        print("bye bye")


if __name__ == "__main__":
    main()
