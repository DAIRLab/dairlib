import numpy as np
from pydairlib.cassie.cassie_gym.reward_osudrl import RewardOSUDRL
import copy

class HighLevelReward(RewardOSUDRL):
    def __init__(self, fwd_vel_des):
        super().__init__()
        self.fwd_vel_des = fwd_vel_des
        self.gain = 2

    def compute_reward(self, timestep, cassie_env_state, prev_cassie_env_state, swing_foot_error=None):
        low_level_reward = super().compute_reward(timestep, cassie_env_state, prev_cassie_env_state,
                                                  swing_foot_error)
        print(f"low level reward = {low_level_reward}")
        high_level_reward = self.gain * (self.fwd_vel_des - cassie_env_state.get_velocities()[0])**2
        print(f"high level reward = {high_level_reward}")
        return low_level_reward + high_level_reward
