import numpy as np
from cassie_sim_data.cassie_traj import *

CASSIE_NX = 45
CASSIE_NU = 10
CASSIE_NL = 12

CASSIE_DTS = 100 #2000 * 0.05

class CassieSimTraj(CassieTraj):

    def __init__(self):
        self.t = np.zeros((CASSIE_DTS,))
        self.x_samples = np.zeros((CASSIE_DTS, CASSIE_NX))
        self.u_samples = np.zeros((CASSIE_DTS, CASSIE_NU))
        self.lambda_traj = np.zeros((CASSIE_DTS, CASSIE_NL))

    def update(self, t, state, action):
        index = self.time_to_index(t)
        self.x_samples[index] = state
        self.u_samples[index] = action
        self.t[index] = t
