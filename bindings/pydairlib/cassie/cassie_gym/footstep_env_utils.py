import numpy as np
import skimage
from skimage.measure import block_reduce
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.evaluation import evaluate_policy as sb3_evaluate_policy
from stable_baselines3.common.logger import Figure
import matplotlib.pyplot as plt

from pydairlib.cassie.cassie_gym.drake_cassie_gym import make_vec_env
from pydairlib.cassie.cassie_gym.footstep_env import CassieFootstepEnv, \
    make_footstep_env_fn, FootstepGymParams

def make_vec_env_from_config(learning_config):
    return make_vec_env(make_footstep_env_fn,
                        learning_config["n_envs"],
                        learning_config["train_seed"],
                        blind=learning_config["blind"],
                        terrain_randomize=learning_config["terrain_randomize"],
                        reward_weights=learning_config["reward_weights"],
                        terrain_config_file=learning_config["terrain_config_path"],
                        curriculum_schedule=learning_config["curriculum_schedule"],
                        goal_x=learning_config["goal_x"])


def make_eval_env_from_config(learning_config, debug=False):
    def env_fn():
        params = FootstepGymParams(goal_x=learning_config["goal_x"], terrain_randomize=learning_config["eval_randomize"], reward_weights=learning_config["reward_weights"])
        env =  Monitor(CassieFootstepEnv(visualize=True, debug=debug, blind=learning_config["blind"], terrain_config_file=learning_config["terrain_config_path"], params=params), info_keywords=["is_success"])
        env.set_seed(learning_config["eval_seed"])
        return env

    # eval_env = SubprocVecEnv([env_fn])
    eval_env = env_fn()
    return eval_env


def evaluate_policy(num_episodes, learning_config, directory):
    model = PPO.load(directory)
    eval_env = make_eval_env_from_config(learning_config, debug=True)
    for ep in range(10):
        state = eval_env.reset()
        while True:
            action, _states = model.predict(state)
            state, r, d, i = eval_env.step(action)
            if d:
                break

def csv_ic_to_new_ic(csv_ic):
    new_ic = np.zeros(45)
    new_ic[0:7] = csv_ic[0:7]
    new_ic[7:15] = csv_ic[7:22][::2]
    new_ic[15:23] = csv_ic[8:23][::2]
    # Assuming all zero velocities! This is a hack.
    new_ic[23:] = csv_ic[23:]
    return new_ic

### Callback functions for logging while training ###
class BucketCountCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(BucketCallback, self).__init__(verbose)
        self.counts = []
    
    def _on_step(self) -> bool:
        # Log scalar value (here a random variable)
        self.logger.record('train/curriculum_bucket', self.model.get_env().bucket)
        return True


class ProgressCallback(BaseCallback):
    def __init__(self,
                 eval_env,
                 fig_lims,
                 verbose=0,
                 n_eval_episodes=10,
                 eval_freq=256,
                 deterministic=True,
                 render=False):
        super(ProgressCallback, self).__init__(verbose)
        self.eval_env = eval_env
        self.n_eval_episodes = n_eval_episodes
        self.eval_freq = eval_freq
        self.deterministic = deterministic
        self.render = render
        self.fig_lims = fig_lims  # [(x_min, x_max), (y_min, y_max)]

        self.progress_logs = []
        self.all_com_paths = []
        self.all_cmd_step_lists = []
        self.warn = True
        

    # Don't have to worry about vec envs since this is only with eval_envs
    def _log_progress_callback(self, locals_, globals_):
        info = locals_["info"]
        if locals_["done"]:
            com_states = info.get("final_com_pos")
            # just see what this looks like with vec envs
            self.progress_logs.append(com_states)
            self.all_com_paths.append(info.get("com_path"))
            self.all_cmd_step_lists.append(info.get("cmd_footsteps"))

    def _on_step(self):
        self.progress_logs = []
        self.all_com_paths = []
        self.all_cmd_step_lists = []
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            episode_rewards, episode_lengths = sb3_evaluate_policy(
                self.model,
                self.eval_env,
                n_eval_episodes=self.n_eval_episodes,
                render=self.render,
                deterministic=self.deterministic,
                return_episode_rewards=True,
                warn=self.warn,
                callback=self._log_progress_callback,
            )
            arr = np.array(self.progress_logs)
            self.logger.record("eval/avg_x_progress", np.mean(arr[:,0]))
            self.logger.record("eval/avg_y_progress", np.mean(arr[:,1]))

            # Plot paths
            fig = plt.figure()
            ax = fig.gca()
            ax.set_ylabel("x")
            ax.set_ylim(self.fig_lims[0][0], self.fig_lims[0][1])
            ax.set_xlabel("y")
            ax.set_xlim(self.fig_lims[1][0], self.fig_lims[1][1])
            ax.set_title("Center of Mass paths during evaluations")
            plt.grid(True)
            for path in self.all_com_paths:
                arr = np.array(path)
                ax.plot(arr[:,1], arr[:,0], color="lightblue", linewidth=3)
            self.logger.record("eval/com_paths", Figure(fig, close=True), exclude=("stdout", "log", "json", "csv"))

            # fig, ax = plt.subplots(1, 5, figsize=(10, 2))
            plt.grid(True)
            for i in range(5):
                com_path = np.array(self.all_com_paths[i])
                fig = plt.figure()
                ax = fig.gca()
                ax.set_ylabel("x")
                ax.set_ylim(self.fig_lims[0][0], self.fig_lims[0][1])
                ax.set_xlabel("y")
                ax.set_xlim(self.fig_lims[1][0], self.fig_lims[1][1])
                ax.set_title(f"List of commanded footsteps during eval episode {i}")
                arr = np.array(self.all_cmd_step_lists[i])
                scatter = ax.scatter(arr[:,1], arr[:,0], c=arr[:,2], cmap="gray")
                # also overlay the corresponding com path
                ax.plot(com_path[:,1], com_path[:,0], color="lightblue", linewidth=3)
                fig.colorbar(scatter)
                self.logger.record(f"eval/cmd_footsteps_{i}", Figure(fig, close=True), exclude=("stdout", "log", "json", "csv"))
            fig.tight_layout()
            plt.close()
        return True


