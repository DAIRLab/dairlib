"""
    Sandbox for testing different ways of creating minimum snap
    trajectories with the abilityu to plot the trajectories inm time and space
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pydairlib.common.plot_styler import PlotStyler
from pydairlib.systems.footstep_planning import \
    AdaptSwingFootTraj

from pydrake.all import PiecewisePolynomial, PathParameterizedTrajectory


def plot_traj_time(traj, npoints, deriv=0):

    t = np.linspace(traj.start_time(), traj.end_time(), npoints)
    y = np.zeros((t.size, traj.rows()))

    for i in range(t.size):
        y[i] = traj.EvalDerivative(t[i], derivative_order=deriv).ravel()
    plt.plot(t, y)


def plot_space_traj(traj, npoints):
    t = np.linspace(traj.start_time(), traj.end_time(), npoints)
    y = np.zeros((t.size, traj.rows()))

    for i in range(t.size):
        y[i] = traj.value(t[i]).ravel()

    ax = plt.axes(projection='3d')
    ax.plot3D(y[:, 0], y[:, 1], y[:, 2])
    return ax


def test_traj(p, t, h):
    assert(h > 0)

    start_time = 3.0

    pp = PiecewisePolynomial(np.zeros((3,)))

    prev_traj = PathParameterizedTrajectory(pp, PiecewisePolynomial.FirstOrderHold(
        [0., 5.], [np.array([[0.]]), np.array([[5.]])]
    ))
    s = time.time()

    # print(prev_traj.EvalDerivative(0.01, 2))
    traj = AdaptSwingFootTraj(
        prev_traj=prev_traj,
        prev_time=start_time + 0.0,
        curr_time=start_time + .015,
        t_start=start_time,
        t_end=start_time + t,
        swing_foot_clearance=h,
        z_vel_final=0,
        z_pos_final_offset=0,
        footstep_target=p.ravel()
    )

    traj2 = AdaptSwingFootTraj(
        prev_traj=traj,
        prev_time=start_time + 0.1,
        curr_time=start_time + .015,
        t_start=start_time,
        t_end=start_time + t,
        swing_foot_clearance=h,
        z_vel_final=0,
        z_pos_final_offset=0,
        footstep_target=p.ravel() + np.array([0.1, 0.05, 0])
    )

    e = time.time()
    print(f't = {e - s}')
    plot_traj_time(traj2, 1000)
    plt.figure()
    ax3d = plot_space_traj(traj2, 1000)
    plt.show()


def test():
    test_traj(np.array([[-0.1], [-0.05], [0]]), 0.3, 0.18)


if __name__ == "__main__":
    test()