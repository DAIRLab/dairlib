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

class RewardBase():

    def __init__(self, weights_filename):
        weights = self.load_weights(weights_filename)

    def compute_reward(self, state):
        reward = 0
        return reward