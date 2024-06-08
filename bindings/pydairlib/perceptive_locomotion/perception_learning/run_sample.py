"""
Train a policy for //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
"""
import argparse
import os
from os import path

import gymnasium as gym

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.ppo_recurrent.ppo_recurrent import RecurrentPPO

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.callbacks import EvalCallback
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecVideoRecorder,
)
import torch as th
import numpy as np

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

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

def bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKSPACE_DIRECTORY'])

def sample(sim_params):
    terrain = 'params/stair_curriculum.yaml'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
    env = gym.make("DrakeCassie-v0",
        sim_params = sim_params,
    )
    rate = 1.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 1000
    obs, _ = env.reset()
    input("Start..")
    for i in range(int(max_steps)):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            input("The environment will reset. Press Enter to continue...")
            obs, _ = env.reset()

def run_play(sim_params, model_path=None):
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    #terrain = 'params/stair_curriculum.yaml'
    #sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    rate = 1.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 3e4
    #test_folder = "rl/vdes_depth/"
    #model_path = path.join(test_folder, 'best_model.zip')
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)

    model_path = 'RPPO_initialize_no_batch_norm.zip'
    model = RecurrentPPO.load(model_path, env, verbose=1)
    
    obs, _ = env.reset()
    input("Start..")
    total_reward = 0

    for _ in range(int(max_steps)):
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, _states = model.predict(obs, deterministic=True)

        obs, reward, terminated, truncated, info = env.step(action)

        if lstm:
            episode_starts = terminated
            
        total_reward += reward
        if terminated or truncated:
            if lstm:
                lstm_states = None
                episode_starts = np.ones((1,), dtype=bool)
            obs, _ = env.reset()
            total_reward = 0
            
def _main(model_path=None):
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv")  # noqa

    #sample(sim_params)
    run_play(sim_params, model_path=None)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--model_path',
        type=str,
        default=None,
    )
    args = parser.parse_args()

    _main(args.model_path)
