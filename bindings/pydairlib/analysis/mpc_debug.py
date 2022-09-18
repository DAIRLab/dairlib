import numpy as np

from pydrake.trajectories import PiecewisePolynomial


class LcmTrajectoryBlock:
    def __init__(self, block):
        self.lcm_trajectory_block = block

    @staticmethod
    def sample(traj, npoints):
        t = np.linspace(traj.start_time(), traj.end_time(), npoints)
        samples = np.zeros((t.shape[0], traj.value(traj.start_time()).shape[0]))
        for i in range(t.shape[0]):
            samples[i] = traj.value(t[i])[:, 0]

        return t, samples

    def get_traj_as_cubic_shape_preserving(self, npoints):
        pp_traj = PiecewisePolynomial.CubicShapePreserving(
            self.lcm_trajectory_block.time_vec,
            self.lcm_trajectory_block.datapoints
        )
        return self.sample(pp_traj, npoints)


class LcmTrajectory:
    def __init__(self, msg):
        self.trajectories = {}
        for block in msg.trajectories:
            self.trajectories[block.trajectory_name] = LcmTrajectoryBlock(block)


class MpcDebug:
    def __init__(self):
        self.t_mpc = []
        self.fsm = {}
        self.x0 = {}
        self.p0 = {}
        self.xxs = {}
        self.uus = {}
        self.pps = {}
        self.tts = {}

    def to_numpy(self):
        for t in self.t_mpc:
            self.x0[t] = np.array(self.x0[t])
            self.p0[t] = np.array(self.p0[t])
            self.xxs[t] = self.recursive_list_to_numpy(
                self.xxs[t]
            )
            self.uus[t] = self.recursive_list_to_numpy(
                self.uus[t]
            )
            self.pps[t] = self.recursive_list_to_numpy(
                self.pps[t]
            )
            self.tts[t] = np.array(self.tts[t])

    def append(self, msg):
        t = msg.utime / 1e6
        self.t_mpc.append(t)
        self.fsm[t] = msg.fsm_state
        self.x0[t] = msg.x0
        self.p0[t] = msg.p0
        self.xxs[t] = msg.xx
        self.uus[t] = msg.uu
        self.pps[t] = msg.pp
        self.tts[t] = msg.tt

    @staticmethod
    # Convert the inner-most list to a numpy array - leaving the rest as a list
    def recursive_list_to_numpy(lst):
        if lst[0] is list:
            lst = [MpcDebug.recursive_list_to_numpy(l) for l in lst]
        else:
            lst = np.array(lst)
        return lst
