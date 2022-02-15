import numpy as np
import os
from cassie_sim_data.cassie_traj import *
from pydrake.trajectories import PiecewisePolynomial

# TODO(yangwill): verify the indices, all simulators shoud output the final trajectory data in the same format
CASSIE_QUATERNION_SLICE = slice(0, 4)
CASSIE_POSITION_SLICE = slice(4, 23)
CASSIE_OMEGA_SLICE = slice(3, 7)
CASSIE_VELOCITY_SLICE = slice(3, 7)

CASSIE_NX = 45
CASSIE_NU = 10
CASSIE_NL = 12

CASSIE_DTS = 100  # 2000 * 0.05

DATASET_DIR = os.path.join(
    os.getenv('HOME'),
    'Documents/research/projects/impact_uncertainty/data/curated_trajectories/')

class CassieHardwareTraj(CassieTraj):

    def __init__(self, dataset_num):
        # Shape of member variables
        # self.t = np.zeros((CASSIE_DTS,))
        # self.x_samples = np.zeros((CASSIE_DTS, CASSIE_NX))
        # self.u_samples = np.zeros((CASSIE_DTS, CASSIE_NU))
        self.dataset_num = dataset_num
        self.t = np.load(DATASET_DIR + 't_' + dataset_num + '.npy')
        self.x_samples = np.load(DATASET_DIR + 'x_' + dataset_num + '.npy')
        self.u_samples = np.load(DATASET_DIR + 'u_' + dataset_num + '.npy')
        self.lambda_traj = None

    def get_initial_state(self):
        initial_state = np.copy(self.x_samples[0, :])
        return initial_state

    def get_action(self, t):
        return self.u_samples[self.time_to_index(t), :]

    def generate_input_traj(self):
        return PiecewisePolynomial.FirstOrderHold(self.t, self.u_samples.T)
        # return PiecewisePolynomial.FirstOrderHold(self.t, np.zeros((CASSIE_NU, CASSIE_DTS)))

    def has_lambda(self):
        return False
