import numpy as np
import matplotlib.pyplot as plt

# TODO(yangwill): verify the indices, all simulators should output the final trajectory data in the same format
CASSIE_QUATERNION_SLICE = slice(0, 4)
CASSIE_POSITION_SLICE = slice(4, 23)
CASSIE_OMEGA_SLICE = slice(23, 26)
CASSIE_VELOCITY_SLICE = slice(26, 45)

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
        # np.save('t_hardware', self.t)
        # np.save('q_hardware', self.x_samples[:, CASSIE_POSITION_SLICE])
        # np.save('t_sim', self.t)
        # np.save('q_sim', self.x_samples[:, CASSIE_POSITION_SLICE])
        plt.plot(self.t, self.x_samples[:, CASSIE_POSITION_SLICE])

    def plot_velocities(self):
        plt.plot(self.t, self.x_samples[:, CASSIE_VELOCITY_SLICE])

    def plot_efforts(self):
        # np.save('t_sim', self.t)
        # np.save('u_sim', self.u_samples)
        plt.plot(self.t, self.u_samples)

