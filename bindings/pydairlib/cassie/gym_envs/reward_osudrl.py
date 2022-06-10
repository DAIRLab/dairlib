import numpy as np
import copy
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
        self.neutral_foot_orient = np.array([-0.24790886454547323, -0.24679713195445646, -0.6609396704367185, 0.663921021343526])
        self.l_foot_orient = 0
        self.r_foot_orient = 0
        self.l_foot_frc = 0
        self.r_foot_frc = 0
        self.l_foot_pos = np.zeros(3)
        self.r_foot_pos = np.zeros(3)
        self.l_foot_vel = np.zeros(3)
        self.r_foot_vel = np.zeros(3)
        self.l_foot_orient_cost = 0
        self.r_foot_orient_cost = 0

        return

    def load_weights(self, filename):
        with open(LOSS_WEIGHTS_FOLDER + filename + '.pkl', 'rb') as f:
            return pickle.load(f)

    def reset_clock_reward(self):
        self.l_foot_frc = 0
        self.r_foot_frc = 0
        foot_pos = np.zeros(6)
        self.l_foot_pos = np.zeros(3)
        self.r_foot_pos = np.zeros(3)
        self.l_foot_vel = np.zeros(3)
        self.r_foot_vel = np.zeros(3)
        self.l_foot_orient_cost = 0
        self.r_foot_orient_cost = 0

    def update_clock_reward(self, foot_forces, foot_pos, l_foot_quat, r_foot_quat):
        prev_foot = copy.deepcopy(np.hstack((self.l_foot_pos, self.r_foot_pos)))
        self.l_foot_frc += foot_forces[0]
        self.r_foot_frc += foot_forces[1]
        # Relative Foot Position tracking
        self.l_foot_pos += foot_pos[0:3]
        self.r_foot_pos += foot_pos[3:6]
        # Foot Orientation Cost
        self.l_foot_orient_cost += (1 - np.inner(self.neutral_foot_orient, l_foot_quat) ** 2)
        self.r_foot_orient_cost += (1 - np.inner(self.neutral_foot_orient, r_foot_quat) ** 2)
        self.l_foot_vel = (foot_pos[0:3] - prev_foot[0:3]) / 0.0005
        self.r_foot_vel = (foot_pos[3:6] - prev_foot[3:6]) / 0.0005

    def compute_reward(self, timestep, cassie_env_state, prev_cassie_env_state):
        pos = cassie_env_state.get_positions()
        vel = cassie_env_state.get_velocities()
        com_pos = cassie_env_state.get_fb_positions()
        com_vel = cassie_env_state.get_fb_velocities()
        joint_pos = cassie_env_state.get_joint_positions()
        joint_vel = cassie_env_state.get_joint_velocities()
        torques = cassie_env_state.get_inputs()
        prev_torques = prev_cassie_env_state.get_inputs()
        com_angular_velocity = cassie_env_state.get_omegas()

        des_forward_vel = cassie_env_state.get_desired_forward_velocity()

        orient_targ = np.array([1, 0, 0, 0])

        com_orient_error  = 0
        # hip_yaw_penalty  = 0
        com_vel_error     = 0
        straight_diff     = 0
        foot_vel_error    = 0
        foot_frc_error    = 0
        foot_orient_error  = 0


        self.l_foot_frc /= timestep
        self.r_foot_frc /= timestep
        self.l_foot_pos /= timestep
        self.r_foot_pos /= timestep
        self.l_foot_orient_cost /= timestep
        self.r_foot_orient_cost /= timestep

        # com orient error
        com_orient_error += 10 * (1 - np.inner(orient_targ, cassie_env_state.get_orientations()) ** 2)
        # foot orient error
        # hip_yaw_penalty += 10 * (self.l_foot_orient_cost + self.r_foot_orient_cost)
        foot_orient_error += 10 * (self.l_foot_orient_cost + self.r_foot_orient_cost)
        # hip_yaw_penalty += np.abs(joint_vel[1]) + np.abs(joint_vel[8])

        # com vel error
        com_vel_error += np.linalg.norm(com_vel[0] - des_forward_vel)

        # straight difference penalty
        straight_diff = np.abs(com_pos[1])  # straight difference penalty
        if straight_diff < 0.05:
            straight_diff = 0
        height_diff = np.abs(com_pos[2] - 0.85)  # height deadzone is range from 0.05 to 0.2 meters depending on speed
        deadzone_size = 0.05 + 0.05 * des_forward_vel
        if height_diff < deadzone_size:
            height_diff = 0
        pelvis_acc = 0.25 * (np.abs(com_angular_velocity.sum()))
        pelvis_motion = straight_diff + height_diff + pelvis_acc
        # print("vel_tracking")
        # print(com_vel_error)
        # print("pelvis_motion")
        # print(pelvis_motion)

        hip_roll_penalty = np.abs(joint_vel[0]) + np.abs(joint_vel[7])

        torque_penalty = 0.5 * (sum(np.abs(prev_torques - torques)) / len(torques))

        reward = 0.200 * np.exp(-(com_orient_error + foot_orient_error)) + \
                 0.150 * np.exp(-pelvis_motion) + \
                 0.250 * np.exp(-com_vel_error) + \
                 0.100 * np.exp(-hip_roll_penalty) + \
                 0.025 * np.exp(-torque_penalty)


        # print(0.200 * np.exp(-(com_orient_error + hip_yaw_penalty)))
        # print(0.150 * np.exp(-pelvis_motion))
        # print(0.150 * np.exp(-com_vel_error))
        # print(0.050 * np.exp(-hip_roll_penalty))
        # print(0.025 * np.exp(-torque_penalty))
        # import pdb; pdb.set_trace()
        return reward