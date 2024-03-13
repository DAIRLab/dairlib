import sys
import lcm
import numpy as np
from matplotlib import pyplot as plt

from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.analysis.log_plotter_cassie import plotter_main

import pydairlib.analysis.mpfc_plotting_utils as mpfc_plots

from pydairlib.common.plot_styler import PlotStyler

# lcmtypes
import dairlib

mpfc_channels = {
    "ALIP__S2S_MPFC_DEBUG": dairlib.lcmt_alip_s2s_mpfc_debug,
    "NETWORK_CASSIE_CONTACT_DISPATCHER": dairlib.lcmt_contact
}


def init_ps():
    ps = PlotStyler()
    ps.set_default_styling()


def main():
    # init_ps()
    filename_mpc = sys.argv[1]
    log_mpc = lcm.EventLog(filename_mpc, "r")

    mpfc_debug_data = get_log_data(
        log_mpc, mpfc_channels, 0, -1, mpfc_plots.mpfc_debug_callback,
        "ALIP__S2S_MPFC_DEBUG"
    )

    contact_data = get_log_data(
        log_mpc, mpfc_channels, 0, -1, mpfc_plots.contact_callback,
        "NETWORK_CASSIE_CONTACT_DISPATCHER"
    )

    _ = mpfc_plots.plot_footstep_sol_in_stance_frame(mpfc_debug_data)
    _ = mpfc_plots.plot_initial_state(mpfc_debug_data)
    _ = mpfc_plots.plot_timing_solution(mpfc_debug_data)
    _ = mpfc_plots.plot_contact(contact_data, mpfc_debug_data)
    plt.show()


if __name__ == "__main__":
    main()
