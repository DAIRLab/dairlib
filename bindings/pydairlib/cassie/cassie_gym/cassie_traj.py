import numpy as np
import matplotlib.pyplot as plt
from pydairlib.cassie.cassie_gym.cassie_env_state import \
    CassieEnvState, CASSIE_NX, CASSIE_NU, CASSIE_NL, CASSIE_POSITION_SLICE,\
    CASSIE_VELOCITY_SLICE, CASSIE_QUATERNION_SLICE, CASSIE_OMEGA_SLICE

CASSIE_EPS_LENGTH = 1000

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


class CassieStateHistory:
    def __init__(self):
        self.x = []
        self.u = []
        self.t = []
        self.r = []
        self.action = []

    def append(self, s, r=0):
        self.x.append(s.x)
        self.u.append(s.u)
        self.t.append(s.t)
        self.action.append(s.action)
        self.r.append(r)

    def make_traj(self):
        traj = CassieTraj()
        traj.t = np.array(self.t)
        traj.x_samples = np.array(self.x)
        traj.u_samples = np.array(self.u)
        traj.lambda_traj = np.zeros((len(self.t), CASSIE_NL))
        return traj

    def get_reward(self):
        return np.array(self.r)
