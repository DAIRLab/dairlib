import numpy as np
from pydairlib.cassie.cassie_gym.cassie_env_state import \
    CASSIE_QUATERNION_SLICE, CASSIE_POSITION_SLICE, CASSIE_OMEGA_SLICE,\
    CASSIE_VELOCITY_SLICE, CASSIE_JOINT_POSITION_SLICE,\
    CASSIE_JOINT_VELOCITY_SLICE, CASSIE_FB_POSITION_SLICE,\
    CASSIE_FB_VELOCITY_SLICE

class VisualLocoReward:
    def __init__(self, weights, goal_state):
        self.weights = weights
        self.goal_state = goal_state

    def calc_reward(self, cassie_state, yaw_des):
        # TODO(hersh500): implement yaw following.
        vel = cassie_state.get_velocities()
        vel_clip = np.clip(vel[0], -1, 1)
        return self.weights[0] * vel_clip + self.weights[1] * vel[1]
