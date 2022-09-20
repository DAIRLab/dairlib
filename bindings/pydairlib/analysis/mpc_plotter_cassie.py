import sys
import lcm
import numpy as np
from matplotlib import pyplot as plt


from mpc_debug import MpcDebug, LcmTrajectory
from process_lcm_log import get_log_data
from mpc_plot_config import MpcPlotConfig
from log_plotter_cassie import plotter_main

# lcmtypes
import dairlib

mpc_channels = {
    "ALIP_MINLP_DEBUG": dairlib.lcmt_mpc_debug,
    "ALIP_COM_TRAJ": dairlib.lcmt_saved_traj
}


def plot_com_traj_solutions(lcm_traj_list, dims, slc=None):
    plt.figure()
    if slc is None:
        slc = slice(len(lcm_traj_list))

    for traj in lcm_traj_list[slc]:
        t, y = traj.trajectories["com_traj"].get_traj_as_first_order_hold(200)
        plt.plot(t, y[:, dims])

def plot_state_traj_over_time(xx):
    nk = len(xx[0])
    xx_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][k], axis=-1) for k in range(nk)]
            ) for n in range(len(xx))
        ]
    )
    plt.plot(xx_traj.T)
    plt.legend(['x', 'y', 'Lx', 'Ly'])


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
    plt.plot(-xy_traj[1], xy_traj[0], color=traj_colors[fsm], linewidth=2)
    # Plot lines from the footstep to the CoM
    for i in range(nm):
        for k in range(nk):
            xy = xx[i][k].ravel()[:2] + pp[i][:2]
            line_x = [-xy[1], -pp[i][1]]
            line_y = [xy[0], pp[i][0]]
            plt.plot(line_x, line_y, color='black', linewidth=1)

    x = x0[:3] + p0
    plt.plot(-x[1], x[0], color=traj_colors[fsm], marker='X', markersize=20)
    plt.plot(-p0[1], p0[0], color='black', marker='X', markersize=20)


def plot_foot_targets(mpc_debug, i):
    plt.figure()
    foot_targets = np.hstack(
        [np.expand_dims(mpc_debug.pps[t][i], axis=-1) for t in mpc_debug.t_mpc]
    )
    plt.plot(mpc_debug.t_mpc, foot_targets.T)


def mpc_processing_callback(data, mpc_channel, com_traj_channel):
    dbg = MpcDebug()
    for msg in data[mpc_channel]:
        dbg.append(msg)
    dbg.to_numpy()
    com_trajs = []
    for msg in data[com_traj_channel]:
        com_trajs.append(LcmTrajectory(msg))
    return dbg, com_trajs


def main():
    plot_config = \
        MpcPlotConfig(
            'bindings/pydairlib/analysis/plot_configs/mpc_plot_config.yaml')

    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    plotter_main(plot_config.cassie_plot_config, log)
    mpc_debug, com_trajs = get_log_data(
        log, mpc_channels,
        plot_config.start_time, plot_config.end_time,
        mpc_processing_callback, "ALIP_MINLP_DEBUG", "ALIP_COM_TRAJ"
    )
    plot_com_traj_solutions(com_trajs, [2])

    idx = 200
    t = mpc_debug.t_mpc[idx]

    # plot_com_traj_solution_overhead(
    #     mpc_debug.xxs[t],
    #     mpc_debug.pps[t],
    #     mpc_debug.fsm[t],
    #     mpc_debug.x0[t],
    #     mpc_debug.p0[t]
    # )
    plot_foot_targets(mpc_debug, 1)
    plt.show()


if __name__ == "__main__":
    main()
