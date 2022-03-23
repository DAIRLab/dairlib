import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

CASSIE_QUATERNION_SLICE = slice(0, 4)
CASSIE_POSITION_SLICE = slice(4, 23)
CASSIE_OMEGA_SLICE = slice(23, 26)
CASSIE_VELOCITY_SLICE = slice(26, 45)
CASSIE_JOINT_POSITION_SLICE = slice(7, 23)
CASSIE_JOINT_VELOCITY_SLICE = slice(29, 45)
CASSIE_FB_POSITION_SLICE = slice(4, 7)
CASSIE_FB_VELOCITY_SLICE = slice(26, 29)

CASSIE_NX = 45
CASSIE_NQ = 23
CASSIE_NV = 22
CASSIE_NU = 10
CASSIE_NACTION = 18


class CassieEnvState():
    def __init__(self, t, x, u, action):
        self.t = t
        self.x = x
        self.u = u
        self.action = action

    def get_positions(self):
        return self.x[CASSIE_POSITION_SLICE]

    def get_orientations(self):
        return self.x[CASSIE_QUATERNION_SLICE]

    def get_velocities(self):
        return self.x[CASSIE_VELOCITY_SLICE]

    def get_omegas(self):
        return self.x[CASSIE_OMEGA_SLICE]

    def get_inputs(self):
        return self.u

    def get_action(self):
        return self.action

    def get_fb_positions(self):
        return self.x[CASSIE_FB_POSITION_SLICE]

    def get_fb_velocities(self):
        return self.x[CASSIE_FB_VELOCITY_SLICE]

    def get_joint_positions(self):
        return self.x[CASSIE_JOINT_POSITION_SLICE]

    def get_joint_velocities(self):
        return self.x[CASSIE_JOINT_VELOCITY_SLICE]

    def get_desired_forward_velocity(self):
        return self.action[2]

    def get_desired_lateral_velocity(self):
        return self.action[3]
