"""
Train a policy for //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
"""
import argparse
import os
from os import path
import numpy as np
import torch as th
import torch.nn as nn

import gymnasium as gym
from gymnasium import spaces
from typing import Callable, Tuple

import stable_baselines3
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.RPO.rpo import RPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, CallbackList
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.monitor import Monitor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import VecMonitor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecVideoRecorder,
    VecNormalize,
)
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.policies import ActorCriticPolicy

from pydrake.geometry import Meshcat
from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource
)

from pydairlib.perceptive_locomotion.systems.cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from typing import Callable

def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Linear learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
      current learning rate depending on remaining progress
    """
    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias_const)
    return layer

class CustomNetwork(nn.Module):
    def __init__(self, last_layer_dim_pi: int = 128, last_layer_dim_vf: int = 128):
        super().__init__()

        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        self.alip_state_dim = 6
        self.heightmap_size = 80
        # CNN for heightmap observations
        n_input_channels = 3
        self.actor_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
        )
        self.critic_cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
        )

        # MLP for ALIP state + Vdes (6,)
        self.actor_alip_mlp = nn.Sequential(
            layer_init(nn.Linear(self.alip_state_dim, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, 64)),
            nn.Tanh(),
        )

        self.critic_alip_mlp = nn.Sequential(
            layer_init(nn.Linear(self.alip_state_dim, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, 128)),
            nn.Tanh(),
            layer_init(nn.Linear(128, 64)),
            nn.Tanh(),
        )

        # Combined MLP for actor
        self.actor_combined_mlp = nn.Sequential(
            layer_init(nn.Linear(3264, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, self.latent_dim_pi), std = 0.1),
            nn.Tanh(),
        )

        # Combined MLP for critic
        self.critic_combined_mlp = nn.Sequential(
            layer_init(nn.Linear(3264, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, 256)),
            nn.Tanh(),
            layer_init(nn.Linear(256, self.latent_dim_vf), std = 1.),
            nn.Tanh(),
        )
        
    def forward(self, observations: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        return self.forward_actor(observations), self.forward_critic(observations)

    def forward_actor(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        actor_cnn_output = self.actor_cnn(image_obs)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]
        actor_alip_mlp_output = self.actor_alip_mlp(alip_state)
        actor_combined_features = th.cat((actor_cnn_output, actor_alip_mlp_output), dim=1)
        actor_actions = self.actor_combined_mlp(actor_combined_features)
        return actor_actions

    def forward_critic(self, observations: th.Tensor) -> th.Tensor:
        batch_size = observations.size(0)
        image_obs = observations[:, :3 * self.heightmap_size * self.heightmap_size].reshape(batch_size, 3, self.heightmap_size, self.heightmap_size)
        critic_cnn_output = self.critic_cnn(image_obs)
        alip_state = observations[:, 3 * self.heightmap_size * self.heightmap_size:]
        critic_alip_mlp_output = self.critic_alip_mlp(alip_state)
        critic_combined_features = th.cat((critic_cnn_output, critic_alip_mlp_output), dim=1)
        critic_actions = self.critic_combined_mlp(critic_combined_features)
        return critic_actions
    
class CustomActorCriticPolicy(ActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        optimizer_class= th.optim.Adam, #torch.optim.RAdam Try RAdam with decoupled_weight_decay=True
        optimizer_kwargs = {'weight_decay': 1e-3, 'epsilon': 1e-5},
        *args,
        **kwargs,
    ):
        kwargs["ortho_init"] = True
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            *args,
            **kwargs,
        )

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork()

def bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKSPACE_DIRECTORY'])

def _run_training(config, args):
    env_name = config["env_name"]
    num_env = config["num_workers"]
    log_dir = config["local_log_dir"]
    policy_type = config["policy_type"]
    total_timesteps = config["total_timesteps"]
    eval_freq = config["model_save_freq"]
    sim_params_train = config["sim_params"]
    sim_params_eval = config["sim_params"]
    
    if not args.test:
        input("Starting...")
        # if visualize is True > TypeError: cannot pickle 'pydrake.geometry.Meshcat' object
        sim_params_train.visualize = False 
        sim_params_train.meshcat = None
        env = make_vec_env(
                           env_name,
                           n_envs=num_env,
                           seed=0,
                           vec_env_cls=SubprocVecEnv,
                           env_kwargs={
                            'sim_params': sim_params_train,
                           })
        env = VecNormalize(venv=env, norm_obs=False)

    else:
        input("Testing...")
        sim_params_train.visualize = False
        sim_params_train.meshcat = None
        env = gym.make(env_name,
                       sim_params = sim_params_train,
                       )
        #env = VecNormalize(venv=env, norm_obs=False)
    
    if args.test:
        model = PPO(
            policy_type, env, learning_rate = linear_schedule(3e-4), n_steps=int(512/num_env), n_epochs=10,
            batch_size=16*num_env, ent_coef=0.01, use_sde=True, verbose=1,
            )
    else:
        tensorboard_log = f"{log_dir}runs/test"
        
        test_folder = "rl/vdes_depth_angle_penalty"
        model_path = path.join(test_folder, 'latest_model.zip')
        #model_path = 'PPO_depth_vdes.zip'
        
        model = PPO.load(model_path, env, learning_rate = linear_schedule(1e-5), max_grad_norm = 0.2,
                        clip_range = 0.2, target_kl = 0.1, ent_coef=0.03, use_sde=True,
                        n_steps=int(256*num_env/num_env), n_epochs=10,
                        batch_size=128*num_env, seed=111,
                        tensorboard_log=tensorboard_log)
        
        print("Open tensorboard (optional) via " f"`tensorboard --logdir {tensorboard_log}`" "in another terminal.")

    sim_params_eval.visualize = True
    sim_params_eval.meshcat = Meshcat()
    eval_env = gym.make(env_name, sim_params = sim_params_eval,)

    eval_env = DummyVecEnv([lambda: eval_env])
    eval_env = VecNormalize(venv=eval_env, norm_obs=False)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=log_dir+f'eval_logs/test',
        log_path=log_dir+f'eval_logs/test',
        eval_freq=eval_freq,
        deterministic=True,
        render=False)

    checkpoint_callback = CheckpointCallback(
        save_freq=eval_freq*0.5,
        save_path="./logs/",
        name_prefix="rl_model",
    )

    callback = CallbackList([checkpoint_callback, eval_callback])

    input("Start learning...")

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
    )

    model.save("latest_model")
    eval_env.close()

def _main():
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--log_path', help="path to the logs directory.",
                        default="./rl/tmp/DrakeCassie/")
    args = parser.parse_args()

    if args.test:
        num_env = 1
    else:
        num_env = 20

    # https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html
    config = {
        "policy_type": CustomActorCriticPolicy,
        "total_timesteps": 2e6 if not args.test else 5,
        "env_name": "DairCassie-v0",
        "num_workers": num_env,
        "local_log_dir": args.log_path,
        "model_save_freq": 1500,
        "sim_params" : sim_params
    }
    _run_training(config, args)

gym.envs.register(
        id="DairCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.DrakeCassieEnv:DrakeCassieEnv",
        )

if __name__ == '__main__':
    _main()
