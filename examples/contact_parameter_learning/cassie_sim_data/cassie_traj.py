import numpy as np
import matplotlib.pyplot as plt

# TODO(yangwill): verify the indices, all simulators should output the final trajectory data in the same format
CASSIE_QUATERNION_SLICE = slice(0, 4)
CASSIE_POSITION_SLICE = slice(4, 23)
CASSIE_OMEGA_SLICE = slice(26, 45)
CASSIE_VELOCITY_SLICE = slice(23, 26)

CASSIE_NX = 45
CASSIE_NU = 10
CASSIE_NL = 12

CASSIE_DTS = 2000 * 0.05


class CassieTraj():

    def __init__(self):
        self.t = np.zeros((CASSIE_DTS,))
        self.x_samples = np.zeros((CASSIE_DTS, CASSIE_NX))  # Cannot be empty
        self.u_samples = np.zeros((CASSIE_DTS, CASSIE_NU))  # Cannot be empty
        self.lambda_traj = np.zeros((CASSIE_DTS, CASSIE_NL))  # May be empty

    def time_to_index(self, t):
        return int(t * 2000)

    def get_positions(self):
        return

    def get_orientations(self):
        return

    def get_velocities(self):
        return

    def get_omegas(self):
        return

    def plot_positions(self):
        plt.plot(self.t, self.x_samples[:, CASSIE_POSITION_SLICE])

    def plot_velocities(self):
        plt.plot(self.t, self.x_samples[:, CASSIE_VELOCITY_SLICE])

    def plot_efforts(self):
        plt.plot(self.t, self.u_samples)

