"""
    Sandbox for testing different ways of creating minimum snap
    trajectories with the abilityu to plot the trajectories inm time and space
"""

import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

from pydairlib.common.plot_styler import PlotStyler
from pydairlib.systems.footstep_planning import SwingFootTrajSolver

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


def plot_space_trajs(trajs, starts, npoints):
    dpi = 300  # Adjust the DPI as needed
    fig_width_inches = 8  # Adjust the figure width in inches
    fig_height_inches = 8  # Adjust the figure height in inches

    # Create a figure and a 3D axis with custom DPI and size
    fig = plt.figure(figsize=(fig_width_inches, fig_height_inches), dpi=dpi)
    ax = fig.add_subplot(111, projection='3d')

    cmap = plt.get_cmap('Reds')
    for i, traj in enumerate(trajs):
        t = np.linspace(starts[i], traj.end_time(), npoints)
        y = np.zeros((t.size, traj.rows()))

        for j in range(t.size):
            y[j] = traj.value(t[j]).ravel()
        ax.plot3D(
            y[:, 0], y[:, 1], y[:, 2], color=cmap(float(i + 2) / len(trajs))
        )
        ax.plot3D(y[0, 0], y[0,1], y[0,2], marker='.', color='black')

    return fig, ax


def plot_time_trajs(trajs, starts, npoints, plot_name, dim=0, deriv=0):
    font = {'size': 10, 'family': 'serif', 'serif': ['Computer Modern']}
    matplotlib.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"
    matplotlib.rc('text.latex', preamble=r'\usepackage{underscore}')
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('font', **font)
    matplotlib.rcParams['lines.linewidth'] = 2
    matplotlib.rcParams['lines.markersize'] = 10
    matplotlib.rcParams['axes.titlesize'] = 40
    matplotlib.rcParams['axes.labelsize'] = 35
    matplotlib.rcParams['xtick.labelsize'] = 20
    matplotlib.rcParams['ytick.labelsize'] = 20

    fig = plt.figure(figsize=(15, 7))
    cmap = plt.get_cmap('Reds')
    for i, traj in enumerate(trajs):
        t = np.linspace(starts[i], traj.end_time(), npoints)
        y = np.zeros((t.size, traj.rows()))

        for j in range(t.size):
            y[j] = traj.EvalDerivative(t[j], derivative_order=deriv).ravel()
        plt.plot(t, y[:, dim], color=cmap(float(i + 2) / len(trajs)))
        plt.plot(starts[i], y[0, dim], marker='.', color='black')

    dims = 'XYZ'
    derivs = ['Position', 'Velocity', 'Acceleration']
    units = 'm' + '/s' * deriv
    if deriv == 0:
        plt.title(f'Swing Foot {dims[dim]}  Trajectory')
    if deriv == 2:
        plt.xlabel('Time (s)')
    if deriv != 2:
        plt.gca().set_xticks([])
    plt.ylabel(f'{derivs[deriv]} ({units})')
    fig.tight_layout()

    plt.savefig(
        f'../{plot_name}_{dims[dim]}_{derivs[deriv].lower()}.png',
        dpi=300
    )

    return fig


def make_video(trajs, starts, video_name):
    fig, ax = plot_space_trajs(trajs, starts, 50)

    # Remove the grid
    ax.grid(False)

    # Hide pane backgrounds
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    # Hide pane lines (the lines on the walls of the plot)
    ax.xaxis.pane.set_edgecolor('w')
    ax.yaxis.pane.set_edgecolor('w')
    ax.zaxis.pane.set_edgecolor('w')

    # Remove ticks
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    # Show only the Cartesian axes
    ax.xaxis.line.set_color((0.0, 0.0, 0.0, 0.0))  # X axis
    ax.yaxis.line.set_color((0.0, 0.0, 0.0, 0.0))  # Y axis
    ax.zaxis.line.set_color((0.0, 0.0, 0.0, 0.0))  # Z axis

    # Rotation function
    def rotate(angle):
        ax.view_init(elev=30., azim=angle)

    # Create animation
    angles = np.linspace(0, 360, 360)
    ani = FuncAnimation(fig, rotate, frames=angles, interval=10)

    # Save the animation as an MP4
    ani.save(f'../{video_name}.mp4', writer='ffmpeg', fps=30, bitrate=10000)


def multi_spline_figure(save_video=True):
    p0 = np.zeros((3,))
    p1 = np.array([0.1, 0.3, 0.0])
    t = 0.3
    h = 0.15

    start_time = 0.0
    pp = PiecewisePolynomial(p0)

    prev_traj = PathParameterizedTrajectory(pp, PiecewisePolynomial.FirstOrderHold(
        [0., 5.], [np.array([[0.]]), np.array([[5.]])]
    ))

    solver = SwingFootTrajSolver()
    cont_trajs = []
    pp_trajs = []
    starts = []

    n = 35
    dt = 0.005
    rng = np.random.default_rng(52125)
    for i in range(n):
        dp = rng.uniform(low=-0.01, high=0.01, size=(3,))
        dp[-1] = rng.uniform(low=-0.001, high=0.001)
        s = time.time()
        traj = solver.AdaptSwingFootTraj(
            prev_traj=prev_traj,
            prev_time=start_time + i * dt,
            t_start=start_time,
            t_end=start_time + t,
            swing_foot_clearance=h,
            z_vel_final=0,
            z_pos_final_offset=0,
            initial_pos=p0,
            footstep_target=p1 + dp,
        )
        e = time.time()
        print(f't: {e - s}')

        prev_traj = traj
        cont_trajs.append(traj)

        # breaks = [start_time, (start_time + t) / 2, start_time + t]
        # samples = [
        #     p0.reshape((3, 1)),
        #     (0.5 * (p0 + p1 + dp) + h * np.array([0, 0, 1.0])).reshape((3, 1)),
        #     (p1 + dp).reshape((3, 1))
        # ]
        # sample_dot_at_start = np.zeros((3, 1))
        # sample_dot_at_end = np.zeros((3, 1))
        # pp_traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        #     breaks=breaks,
        #     samples=samples,
        #     sample_dot_at_start=sample_dot_at_start,
        #     sample_dot_at_end=sample_dot_at_end
        # )
        # pp_trajs.append(pp_traj)
        starts.append(start_time + (i * dt))
        # p1 = p1 + dp

    fx = plot_time_trajs(
        cont_trajs, starts, 50, 'continous_traj', dim=2, deriv=0
    )
    fdx = plot_time_trajs(
        cont_trajs, starts, 50, 'continous_traj', dim=2, deriv=1
    )
    fddx = plot_time_trajs(
        cont_trajs, starts, 50, 'continous_traj', dim=2, deriv=2
    )

    # fx_pp = plot_time_trajs(pp_trajs, starts, 50, 'pp_traj', dim=0, deriv=0)
    # fdx_pp = plot_time_trajs(pp_trajs, starts, 50, 'pp_traj', dim=0, deriv=1)
    # fddx_pp = plot_time_trajs(pp_trajs, starts, 50, 'pp_traj', dim=0, deriv=2)
    #
    if save_video:
        make_video(cont_trajs, starts, 'continuous_splines')
        make_video(pp_trajs, starts, 'prev_splines')

    plt.show()


def test_traj(p0, p1, t, h):
    assert(h > 0)

    start_time = 3.0

    pp = PiecewisePolynomial(p0)

    prev_traj = PathParameterizedTrajectory(pp, PiecewisePolynomial.FirstOrderHold(
        [0., 5.], [np.array([[0.]]), np.array([[5.]])]
    ))

    solver = SwingFootTrajSolver()

    s = time.time()

    # print(f'offset: {co}')
    # print(prev_traj.EvalDerivative(0.01, 2))
    traj = solver.AdaptSwingFootTraj(
        prev_traj=prev_traj,
        prev_time=start_time + 0.0,
        t_start=start_time,
        t_end=start_time + t,
        swing_foot_clearance=h,
        z_vel_final=0,
        z_pos_final_offset=0,
        initial_pos=p0,
        footstep_target=p1,
    )

    traj2 = solver.AdaptSwingFootTraj(
        prev_traj=traj,
        prev_time=start_time + 0.2,
        t_start=start_time,
        t_end=start_time + t,
        swing_foot_clearance=h,
        z_vel_final=0,
        z_pos_final_offset=0,
        initial_pos=p0,
        footstep_target=p1 + np.array([0.01, 0.03, 0]),
    )

    e = time.time()
    print(f't = {e - s}')
    plot_traj_time(traj2, 1000)
    plt.figure()
    ax3d = plot_space_traj(traj2, 1000)
    plt.show()


def test():
    p0 = np.array([-0.1, -0.3, 0])
    p1 = p0 + np.array([0.2, 0.02, 0])
    test_traj(p0, p1, 0.3, 0.1)


if __name__ == "__main__":
    multi_spline_figure(save_video=False)
