import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation

from mpc_debug import MpcDebug, LcmTrajectory

from pydairlib.common import plotting_utils, plot_styler


def make_solution_animation(mpc_debug):
    ps = plot_styler.PlotStyler()
    soln = mpc_debug.mpc_trajs['solution']

    def _animate_callback(i):
        ax = plt.gca()
        ax.clear()
        t = mpc_debug.t_mpc[i]
        plot_com_traj_solution_overhead(
            soln.xxs[t],
            soln.pps[t],
            mpc_debug.fsm[t],
            mpc_debug.x0[t],
            mpc_debug.realsense_origin_in_world[t]
        )

    ani = animation.FuncAnimation(
        ps.fig, _animate_callback, frames=len(mpc_debug.t_mpc), interval=1)

    plt.show()


def plot_com_traj_solution_overhead(xx, pp, fsm, x0, p0):
    nm = len(pp)
    nk = len(xx[0])
    traj_colors = ['blue', 'orange']
    xy_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][k][:2] + pp[n][:2], axis=-1) for k in range(nk)]
            ) for n in range(nm)
        ]
    )
    # Plot the CoM traj
    ax = plt.gca()
    ax.set_xlim([-xy_traj[1, 0] - 0.5, -xy_traj[1, 0] + 0.5])
    ax.set_ylim([xy_traj[0, 0] - 0.1, xy_traj[0, 0] + 1.0])

    ax.plot(-xy_traj[1], xy_traj[0], color=traj_colors[fsm], linewidth=2)
    # Plot lines from the footstep to the CoM
    for i in range(nm):
        for k in range(nk):
            xy = xx[i][k].ravel()[:2] + pp[i][:2]
            line_x = [-xy[1], -pp[i][1]]
            line_y = [xy[0], pp[i][0]]
            plt.plot(line_x, line_y, color='black', linewidth=1)

    x = x0[:3] + p0
    ax.plot(-x[1], x[0], color=traj_colors[fsm], marker='X', markersize=20)
    ax.plot(-p0[1], p0[0], color='black', marker='X', markersize=20)


def mpc_processing_callback(data, mpc_channel, footstep_channel, fsm_channel):
    mpc_debug = MpcDebug()
    for msg in data[mpc_channel]:
        mpc_debug.append(msg)
    mpc_debug.to_numpy()

    footsteps = []
    # for msg in data[footstep_channel]:
    #     footsteps.append(msg.target)

    return mpc_debug, footsteps


def plot_mpc_state_traj(xx, ps=None):
    ps = plot_styler.PlotStyler() if ps is None else ps

    nk = len(xx[0])
    xx_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][k], axis=-1) for k in range(nk)]
            ) for n in range(len(xx))
        ]
    )
    data = {
        't': np.arange(0, nk * len(xx), dtype=float),
        'xx': xx_traj.T
    }
    labels = ['x', 'y', 'L_x', 'L_y']
    plotting_utils.make_plot_of_entire_series(
        data,
        't',
        {'xx': labels},
        {'xlabel': 'Knot Point',
         'ylabel': 'ALIP State',
         'title': 'MPC State Trajectory'},
        ps)
    return ps


def plot_mpc_initial_state(mpc_debug, ps=None):
    ps = plot_styler.PlotStyler() if ps is None else ps

    xx = np.zeros((len(mpc_debug.t_mpc), 4))
    for i, t in enumerate(mpc_debug.t_mpc):
        xx[i] = mpc_debug.x0[t]
    labels = ['x', 'y', 'L_x', 'L_y']
    data = {
        't_mpc': mpc_debug.t_mpc,
        'xx': xx
    }
    plotting_utils.make_plot_of_entire_series(
        data, 't_mpc', {'xx': labels},
        {'xlabel': 'Time (s)', 'ylabel': 'x0', 'title': 'MPC Initial States'},
        ps
    )
    return ps


def plot_mpc_state_prediction(mpc_debug, ps=None):
    ps = plot_mpc_initial_state(mpc_debug, ps)
    xx = np.zeros((len(mpc_debug.t_mpc), 4))
    for i, t in enumerate(mpc_debug.t_mpc):
        xx[i] = mpc_debug.mpc_trajs["solution"].xxs[t][0][-1]
    labels = ['x-predicted', 'y-predicted', 'L_x-predicted', 'L_y-predicted']
    data = {
        't_mpc': mpc_debug.t_mpc,
        'xx': xx
    }
    plotting_utils.make_plot_of_entire_series(
        data, 't_mpc', {'xx': labels},
        {'xlabel': 'Time (s)', 'ylabel': 'x0', 'title': 'MPC Initial States'},
        ps
    )


def plot_mpc_loop_time(mpc_debug):
    plt.figure()
    looptimes = np.ediff1d(mpc_debug.t_mpc)
    plt.plot( mpc_debug.t_mpc[:-1], looptimes)
    plt.title('Mpc loop times')
    plt.ylabel('Loop time (s)')


def plot_mpc_solve_time(mpc_debug):
    plt.figure()
    plt.plot(mpc_debug.t_mpc, mpc_debug.solve_time)
    plt.title('MPC solve time')
    plt.ylabel('Solve time (s)')


def plot_foothold_origin(mpc_debug, ps=None):
    ps = plot_styler.PlotStyler() if ps is None else ps



def plot_mpc_timing(mpc_debug):
    plt.figure()
    looptimes = np.ediff1d(mpc_debug.t_mpc)
    solve_times = mpc_debug.solve_time[:-1]
    plt.plot(mpc_debug.t_mpc[:-1], looptimes - solve_times)
    plt.plot(mpc_debug.t_mpc[:-1], looptimes)
    plt.plot(mpc_debug.t_mpc, mpc_debug.solve_time)
    plt.title("MPC loop time vs solve time")
    plt.ylabel('t(s)')
    plt.legend(['Loop time - solve time', 'loop time', 'solve time'])


def plot_foot_targets(mpc_debug, i, ps=None):
    ps = plot_styler.PlotStyler() if ps is None else ps
    foot_targets = np.hstack(
        [
            np.expand_dims(mpc_debug.mpc_trajs["solution"].pps[t][i], axis=-1)
            for t in mpc_debug.t_mpc
        ]
    )
    labels = {
        'title': 'MPC Next foot position solution',
        'xlabel': 't (s)',
        'ylabel': 'Position P (m)'
    }
    plotting_utils.make_plot_of_entire_series(
        {'t': mpc_debug.t_mpc, 'p': foot_targets.T},
        't',
        {'p': ['x', 'y', 'z']},
        labels, ps
    )
    return ps

