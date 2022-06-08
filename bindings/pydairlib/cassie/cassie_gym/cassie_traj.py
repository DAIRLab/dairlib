import numpy as np
import matplotlib.pyplot as plt
from bindings.pydairlib.cassie.gym.cassie_env_state import *

# 10000 dts / 2000Hz = 5 seconds
CASSIE_EPS_LENGTH = 100000


class CassieTraj():
    def __init__(self):
        self.t = np.zeros((CASSIE_EPS_LENGTH,))
        self.x_samples = np.zeros((CASSIE_EPS_LENGTH, CASSIE_NX))  # Cannot be empty
        self.u_samples = np.zeros((CASSIE_EPS_LENGTH, CASSIE_NU))  # Cannot be empty
        self.lambda_traj = np.zeros((CASSIE_EPS_LENGTH, CASSIE_NL))  # May be empty

    def time_to_index(self, t):
        if int(t * 2000) >= self.u_samples.shape[0]:
            print("time %.2f is out of bounds" % t)
        return int(t * 2000)

    def get_positions(self):
        return self.x_samples[:, CASSIE_POSITION_SLICE]

    def get_orientations(self):
        return self.x_samples[:, CASSIE_QUATERNION_SLICE]

    def get_velocities(self):
        return self.x_samples[:, CASSIE_VELOCITY_SLICE]

    def get_omegas(self):
        return self.x_samples[:, CASSIE_OMEGA_SLICE]

    def plot_positions(self):
        plt.plot(self.t, self.x_samples[:, CASSIE_POSITION_SLICE])

    def plot_velocities(self):
        plt.plot(self.t, self.x_samples[:, CASSIE_VELOCITY_SLICE])

    def plot_efforts(self):
        plt.plot(self.t, self.u_samples)

    def update(self, t, state, action):
        index = self.time_to_index(t)
        self.x_samples[index] = state
        self.u_samples[index] = action
        self.t[index] = t