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
CASSIE_NRADIO = 18


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


def quat_to_rotation(q):
    return R.from_quat([q[1], q[2], q[3], q[0]])


def reexpress_state_local_to_global_omega(state):
    new_state = state.flatten()
    rot = quat_to_rotation(new_state[CASSIE_QUATERNION_SLICE])
    new_state[CASSIE_OMEGA_SLICE] = rot.apply(new_state[CASSIE_OMEGA_SLICE])
    return new_state


def reexpress_state_global_to_local_omega(state):
    new_state = state.flatten()
    rot = quat_to_rotation(new_state[CASSIE_QUATERNION_SLICE])
    new_state[CASSIE_OMEGA_SLICE] = rot.apply(new_state[CASSIE_OMEGA_SLICE], inverse=True)
    return new_state
