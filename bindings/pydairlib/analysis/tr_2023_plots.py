import sys
import lcm
import numpy as np
from matplotlib import pyplot as plt

from process_lcm_log import get_log_data
import mpc_plotting_utils as mpc_plots

from pydairlib.common.plot_styler import PlotStyler

# lcmtypes
import dairlib


import io
import os
from yaml import load, dump


try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


mpc_channels = {
    "ALIP_MINLP_DEBUG": dairlib.lcmt_mpc_debug,
    "FOOTSTEP_TARGET": dairlib.lcmt_footstep_target
}


def init_ps():
    ps = PlotStyler()
    ps.set_default_styling()


def collect_solve_time_vs_num_footholds(mpc_debug, data=None):
    data = {} if data is None else data
    for i, t in enumerate(mpc_debug.t_mpc):
        if mpc_debug.nfootholds[t] not in data:
            data[mpc_debug.nfootholds[t]] = [mpc_debug.solve_time[i]]
        else:
            data[mpc_debug.nfootholds[t]].append(mpc_debug.solve_time[i])
    return data


def collect_solve_time_vs_constraint_activation(mpc_debug, data=None):
    data = { True: [], False: [] } if data is None else data
    for i, t in enumerate(mpc_debug.t_mpc):
        data[mpc_debug.constraint_activation[t]].append(mpc_debug.solve_time[i])
    return data


def plot_solve_time_vs_nfootholds(logs):
    data = {}
    for log in logs.values():
        data = collect_solve_time_vs_num_footholds(log["mpc_debug"], data)
    for key in data:
        data[key] = np.array(data[key])

    ns = list(data.keys())
    t_min = [1000 * np.min(data[k]) for k in data]
    t_avg = [1000 * np.mean(data[k]) for k in data]
    t_max = [1000 * np.max(data[k]) for k in data]
    t_95 = [1000 * np.percentile(data[k], 90) for k in data]

    t_min = [t for _, t in sorted(zip(ns, t_min))]
    t_max = [t for _, t in sorted(zip(ns, t_max))]
    t_avg = [t for _, t in sorted(zip(ns, t_avg))]
    t_95 = [t for _, t in sorted(zip(ns, t_95))]
    ns = sorted(ns)

    ps = PlotStyler()
    ps.plot(ns, t_min)
    ps.plot(ns, t_max)
    ps.plot(ns, t_avg)
    ps.plot(
        ns, t_95, xlabel='Number of Footholds', ylabel='MPC Solve Time (ms)',
        title='MPC Solve Time vs. Number of Footholds'
    )
    ps.add_legend(['min', 'max', 'mean', '90th Percentile'])


def plot_solve_time_vs_constraint_activation(logs):
    data = { False: [], True: [] }

    for log in logs.values():
        data = collect_solve_time_vs_constraint_activation(log["mpc_debug"], data)
    for key in data:
        data[key] = 1000 * np.array(data[key])

    ps = PlotStyler()
    plt.boxplot(data.values())
    plt.gca().set_xticklabels(
        ['Foothold Constraint Slack', 'Foothold Constraint Active']
    )
    plt.ylabel('MPC Solve Time (ms)')
    plt.title('MPC Solve Time vs. Foothold Constraint Activation')


def main():
    init_ps()

    dataset_config = load(
        io.open('bindings/pydairlib/analysis/plot_configs/tr_plots.yaml', 'r'),
        Loader=Loader
    )
    log_parent_folder = sys.argv[1]
    logs = {}
    for key in dataset_config.keys():
        logs_key = {}
        for log in dataset_config[key]:
            log_key = f'{log["date"]}-lcmlog-mpc-{log["lognum"]}'
            fname = os.path.join(
                log_parent_folder,
                f'{log["date"]}/lcmlog-mpc-{log["lognum"]}'
            )
            lcmlog = lcm.EventLog(fname, "r")
            mpc_debug, _ = get_log_data(
                lcmlog,
                mpc_channels,
                log["start_time"],
                log["end_time"],
                mpc_plots.mpc_processing_callback,
                "ALIP_MINLP_DEBUG", "FOOTSTEP_TARGET", "FSM"
            )
            logs_key[log_key] = {"mpc_debug" : mpc_debug}
        logs[key] = logs_key

    plot_solve_time_vs_nfootholds(logs["solve_time_vs_num_footholds"])
    plot_solve_time_vs_constraint_activation(logs["solve_time_vs_constraint_activation"])
    plt.show()


if __name__ == "__main__":
    main()
