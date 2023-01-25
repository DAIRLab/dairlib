import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pydairlib.perceptive_locomotion.controllers import \
    MakeMinSnapTrajFromWaypoints

from pydrake.all import PiecewisePolynomial


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


def test():
    t = [0, 0.5, 1.0]
    wp = np.array([[0, 0, -0.85], [0.05, 0.05, -0.79], [0.1, 0.1, -0.85]]).T
    traj = MakeMinSnapTrajFromWaypoints(wp, t)

    plot_traj_time(traj, 100)
    plt.figure()
    plot_space_traj(traj, 100)
    plt.show()


if __name__ == "__main__":
    test()
