import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pickle

CASSIE_QUATERNION_SLICE = slice(0, 4)
CASSIE_POSITION_SLICE = slice(4, 23)
CASSIE_OMEGA_SLICE = slice(23, 26)
CASSIE_VELOCITY_SLICE = slice(26, 45)
CASSIE_JOINT_POSITION_SLICE = slice(7, 23)
CASSIE_JOINT_VELOCITY_SLICE = slice(29, 45)
CASSIE_FB_POSITION_SLICE = slice(4, 7)
CASSIE_FB_VELOCITY_SLICE = slice(26, 29)
LOSS_WEIGHTS_FOLDER = 'examples/contact_parameter_learning/cassie_loss_weights/'

class RewardOSUDRL():

    def __init__(self, weights_filename=None):
        if weights_filename:
            weights = self.load_weights(weights_filename)
        return

    def load_weights(self, filename):
        with open(LOSS_WEIGHTS_FOLDER + filename + '.pkl', 'rb') as f:
            return pickle.load(f)

    def compute_reward(self, cassie_env_state, prev_cassie_env_state):


        pos = cassie_env_state.get_positions()
        vel = cassie_env_state.get_velocities()
        com_pos = cassie_env_state.get_fb_positions()
        com_vel = cassie_env_state.get_fb_velocities()
        joint_pos = cassie_env_state.get_joint_positions()
        joint_vel = cassie_env_state.get_joint_velocities()
        torques = cassie_env_state.get_inputs()
        prev_torques = prev_cassie_env_state.get_inputs()

        des_forward_vel = cassie_env_state.get_desired_forward_velocity()

        orient_targ = np.array([1, 0, 0, 0])

        com_orient_error  = 0
        foot_orient_error  = 0
        com_vel_error     = 0
        straight_diff     = 0
        foot_vel_error    = 0
        foot_frc_error    = 0

        # com orient error
        com_orient_error += 15 * (1 - np.inner(orient_targ, pos[CASSIE_QUATERNION_SLICE]) ** 2)

        # foot orient error
        # foot_orient_error += 10 * (self.l_foot_orient_cost + self.r_foot_orient_cost)
        foot_orient_error += 0

        # com vel error
        com_vel_error += np.linalg.norm(com_vel - des_forward_vel)

        # straight difference penalty
        straight_diff = np.abs(com_pos[1])  # straight difference penalty
        if straight_diff < 0.05:
            straight_diff = 0
        height_diff = np.abs(com_pos[2] - 0.9)  # height deadzone is range from 0.05 to 0.2 meters depending on speed
        deadzone_size = 0.05 + 0.05 * des_forward_vel
        if height_diff < deadzone_size:
            height_diff = 0
        pelvis_acc = 0.25 * (np.abs(com_vel.sum()))
        pelvis_motion = straight_diff + height_diff + pelvis_acc

        hip_roll_penalty = np.abs(joint_vel[0]) + np.abs(joint_vel[7])

        torque_penalty = 0.25 * (sum(np.abs(prev_torques - torques)) / len(torques))

        reward = 0.200 * np.exp(-(com_orient_error + foot_orient_error)) + \
                 0.150 * np.exp(-pelvis_motion) + \
                 0.150 * np.exp(-com_vel_error) + \
                 0.050 * np.exp(-hip_roll_penalty) + \
                 0.025 * np.exp(-torque_penalty)
        return reward