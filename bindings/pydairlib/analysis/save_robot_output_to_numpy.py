import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.analysis.cassie_plot_config import CassiePlotConfig
import pydairlib.analysis.cassie_plotting_utils as cassie_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.common.plot_styler import PlotStyler


def parse(log, end_time, outfile):
    use_floating_base = True
    use_springs = True

    channel_x = "CASSIE_STATE_DISPATCHER"
    channel_u = "CASSIE_INPUT"
    channel_osc = "OSC_DEBUG_WALKING"

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=True)
    controller_plant, _ = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    default_channels = cassie_plots.cassie_default_channels
    robot_output, robot_input, osc_debug, imu_accel = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     0,
                     end_time,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)

    np.savez_compressed(
        outfile,
        robot_output=robot_output,
        position_names=pos_names,
        velocity_names=vel_names,
        actuator_names=act_names
    )


def main():
    logfile = sys.argv[1]
    end_time = float(sys.argv[2])
    outfile = sys.argv[3]

    log = lcm.EventLog(logfile, "r")
    parse(log, end_time, outfile)


if __name__ == '__main__':
    main()