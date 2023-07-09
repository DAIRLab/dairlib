import pdb

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

    def get_traj_as_first_order_hold(self, npoints):
        pp_traj = PiecewisePolynomial.FirstOrderHold(
            self.lcm_trajectory_block.time_vec,
            self.lcm_trajectory_block.datapoints
        )
        return self.sample(pp_traj, npoints)


class LcmTrajectory:
    def __init__(self, msg):
        self.trajectories = {}
        for block in msg.trajectories:
            self.trajectories[block.trajectory_name] = LcmTrajectoryBlock(block)


class MpcSolution:
    def __init__(self):
        self.t_mpc = []
        self.xxs = {}
        self.uus = {}
        self.pps = {}
        self.tts = {}

    def append(self, t, msg):
        self.t_mpc.append(t)
        self.xxs[t] = msg.xx
        self.uus[t] = msg.uu
        self.pps[t] = msg.pp
        self.tts[t] = msg.tt

    def to_numpy(self):
        for t in self.t_mpc:
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

    @staticmethod
    # Convert the inner-most list to a numpy array - leaving the rest as a list
    def recursive_list_to_numpy(lst):
        if lst[0] is list:
            lst = [MpcDebug.recursive_list_to_numpy(l) for l in lst]
        else:
            lst = np.array(lst)
        return lst


def foothold_constraint_active(msg):
    footholds = [
        (np.array(f.A), np.array(f.b)) for f in msg.footholds.footholds
    ]

    p1 = np.array(msg.solution.pp[1])
    p2 = np.array(msg.solution.pp[2])
    viol_p1 = np.array([np.max(A @ p1 - b) for (A, b) in footholds])
    viol_p2 = np.array([np.max(A @ p2 - b) for (A, b) in footholds])

    return np.min(viol_p1) > -1e-6 or np.min(viol_p2) > -1e-6


class MpcDebug:
    def __init__(self):
        self.t_mpc = []
        self.solve_time = []
        self.fsm = {}
        self.x0 = {}
        self.p0 = {}
        self.mpc_trajs = {
            "solution": MpcSolution(),
            "guess": MpcSolution(),
            "desired": MpcSolution()
        }
        self.nfootholds = {}
        self.constraint_activation = {}

    def to_numpy(self):
        for t in self.t_mpc:
            self.x0[t] = np.array(self.x0[t])
            self.p0[t] = np.array(self.p0[t])
        for key in self.mpc_trajs.keys():
            self.mpc_trajs[key].to_numpy()

    def append(self, msg):
        t = msg.utime / 1e6
        self.t_mpc.append(t)
        self.solve_time.append(msg.solve_time_us / 1e6)
        self.x0[t] = msg.x0
        self.p0[t] = msg.p0
        self.fsm[t] = msg.fsm_state
        self.nfootholds[t] = msg.footholds.n
        self.mpc_trajs["solution"].append(t, msg.solution)
        self.mpc_trajs["guess"].append(t, msg.guess)
        self.mpc_trajs["desired"].append(t, msg.desired)
        self.constraint_activation[t] = foothold_constraint_active(msg)
