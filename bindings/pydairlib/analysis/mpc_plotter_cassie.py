import sys
import lcm
import numpy as np
from matplotlib import pyplot as plt

from process_lcm_log import get_log_data
from mpc_plot_config import MpcPlotConfig
from log_plotter_cassie import plotter_main

import mpc_plotting_utils as mpc_plots

from pydairlib.common.plot_styler import PlotStyler

# lcmtypes
import dairlib

mpc_channels = {
    "ALIP_MINLP_DEBUG": dairlib.lcmt_mpc_debug,
    "FOOTSTEP_TARGET": dairlib.lcmt_footstep_target
}


def init_ps():
    ps = PlotStyler()
    ps.set_default_styling()


def main():
    # init_ps()

    plot_config = MpcPlotConfig(
            'bindings/pydairlib/analysis/plot_configs/mpc_plot_config.yaml')

    filename = sys.argv[1]
    filenum = filename.split('/')[-1].split('-')[-1]
    if sys.argv[2] == 'hardware':
        filename_mpc = filename.replace(f'lcmlog-{filenum}', f'lcmlog-mpc-{filenum}')
    else:
        filename_mpc = filename
    log = lcm.EventLog(filename, "r")
    log_mpc = lcm.EventLog(filename_mpc, "r")
    plotter_main(plot_config.cassie_plot_config, log)

    mpc_debug, footstep_targets = get_log_data(
        log_mpc,
        mpc_channels,
        plot_config.start_time,
        plot_config.end_time,
        mpc_plots.mpc_processing_callback,
        "ALIP_MINLP_DEBUG", "FOOTSTEP_TARGET", "FSM"
    )

    if plot_config.make_animation:
        mpc_plots.make_solution_animation(mpc_debug)
        quit()

    _ = mpc_plots.plot_mpc_timing(mpc_debug)
    
    ps = mpc_plots.plot_mpc_state_traj(
        mpc_debug.mpc_trajs["solution"].xxs[mpc_debug.t_mpc[5]])

    _ = mpc_plots.plot_mpc_state_prediction(mpc_debug)

    _ = mpc_plots.plot_foot_targets(mpc_debug, 1)

    plt.show()


if __name__ == "__main__":
    main()
