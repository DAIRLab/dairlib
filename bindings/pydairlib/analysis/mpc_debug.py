import numpy as np
from pydairlib.common import plot_styler, plotting_utils
from pydrake.trajectories import PiecewisePolynomial


class mpc_trajectory_block:

    def __init__(self, block):
        self.trajectory_name = block.trajectory_name
        self.time_vec = np.array(block.time_vec)
        self.datapoints = np.array(block.datapoints)
        self.datatypes = block.datatypes


class mpc_trajectory:

    def __init__(self, msg):
        self.trajectories = {}
        for block in msg.trajectories:
            self.trajectories[block.trajectory_name] = \
                mpc_trajectory_block(block)

    @staticmethod
    def sample(traj, npoints):
        t = np.linspace(traj.start_time(), traj.end_time(), npoints)
        samples = np.zeros((t.shape[0], traj.value(traj.start_time()).shape[0]))
        for i in range(t.shape[0]):
            samples[i] = traj.value(t[i])[:, 0]
        return t, samples

    def traj_as_cubic_with_continuous_second_derivatives(
            self, trajectory_name, npoints):

        traj_block = self.trajectories[trajectory_name]
        dim = int(traj_block.datapoints.shape[0] / 2)

        pp_traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            traj_block.time_vec,
            traj_block.datapoints[0:dim, :],
            traj_block.datapoints[dim:2*dim, 0],
            traj_block.datapoints[dim:2*dim, -1])

        return self.sample(pp_traj, npoints)

    def traj_as_cubic_hermite(self, trajectory_name, npoints):
        traj_block = self.trajectories[trajectory_name]
        dim = int(traj_block.datapoints.shape[0] / 2)

        pp_traj = PiecewisePolynomial.CubicHermite(
            traj_block.time_vec, traj_block.datapoints[0:dim, :],
            traj_block.datapoints[dim:2*dim, :])

        return self.sample(pp_traj, npoints)

    def point_as_zoh(self, trajectory_name, dt):
        traj_block = self.trajectories[trajectory_name]
        t = np.array([traj_block.time_vec[0], traj_block.time_vec[0] + dt])
        samples = np.vstack((traj_block.datapoints.ravel(),
                             traj_block.datapoints.ravel()))
        return t, samples


def process_mpc_channel(data, mpc_channel, input_traj='input_traj',
                        npoints=50, default_dt=0.05):
    mpc_data = data[mpc_channel]
    mpc_solutions = {}
    for name in mpc_data[-1].trajectory_names:
        mpc_solutions[name] = []
    ti = -1
    for msg in mpc_data:
        if not msg.trajectories:
            continue
        if msg.trajectories[0].time_vec[0] > ti:
            ti = msg.trajectories[0].time_vec[0]
            mpc_sol = mpc_trajectory(msg)
            for traj in mpc_sol.trajectories:
                if mpc_sol.trajectories[traj].time_vec.size > 1 and \
                        traj != input_traj:
                    t, x = mpc_sol.traj_as_cubic_hermite(traj, npoints)
                elif traj == input_traj:
                    t, x = mpc_sol.trajectories[traj].time_vec, \
                           mpc_sol.trajectories[traj].datapoints.T
                else:
                    t, x = mpc_sol.point_as_zoh(traj, default_dt)
                mpc_solutions[traj].append({'t': t, traj: x})

    return mpc_solutions


def plot_mpc_traj(mpc_solutions, traj_name, dim):
    time_keys = ['t' for _ in mpc_solutions[traj_name]]
    time_slices = [slice(len(data['t'])) for data in mpc_solutions[traj_name]]
    keys_to_plot = [[traj_name] for _ in mpc_solutions[traj_name]]
    slices_to_plot = [{traj_name: dim} for _ in mpc_solutions[traj_name]]
    legend_entries = [[i] for i in range(len(mpc_solutions[traj_name]))]

    ps = plot_styler.PlotStyler()
    plotting_utils.make_mixed_data_plot(
        mpc_solutions[traj_name],
        time_keys,
        time_slices,
        keys_to_plot,
        slices_to_plot,
        legend_entries,
        {'title': traj_name + " mpc sol " + str(dim),
         'xlabel': 't(s)',
         'ylabel': 'y'}, ps)

    return ps




