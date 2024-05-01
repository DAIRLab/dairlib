import sys
import lcm
import matplotlib as mpl
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
from pydairlib.analysis.process_lcm_log import get_log_data
from pydairlib.analysis.cassie_plot_config import CassiePlotConfig
import pydairlib.analysis.cassie_plotting_utils as cassie_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.common.plot_styler import PlotStyler


def plotter_main(plot_config, log1, log2):
    """
        workhorse function to construct plots given an lcmlog and a plot config.
        Note that to allow for more modularity, this function
        doesn't call plt.show()
    """

    use_floating_base = plot_config.use_floating_base
    use_springs = plot_config.use_springs

    channel_x = "CASSIE_STATE_DISPATCHER"
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=True)
    controller_plant, _ = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)
    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    default_channels = cassie_plots.cassie_default_channels
    if plot_config.use_archived_lcmtypes:
        default_channels = cassie_plots.cassie_default_channels_archive
    robot_output, robot_input, osc_debug, imu_accel = \
        get_log_data(log1,  # log
                     default_channels,  # lcm channels
                     2, 4,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)

    _, robot_input2, osc_debug2, _ = \
        get_log_data(log2,  # log
                     default_channels,  # lcm channels
                     2, 4,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)

    ps = PlotStyler()
    ps.set_default_styling()

    osc_debug['t_osc'] -= osc_debug['t_osc'][0]
    osc_debug2['t_osc'] -= osc_debug2['t_osc'][0]

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)
    print('Log start time: ', robot_output['t_x'][0])

    plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)
    plot.plot(osc_debug2['t_osc'], osc_debug2['qp_solve_time'])
    plot.add_legend(['FCCQP', 'OSQP'])

    mbp_plots.add_fsm_to_plot(
        plot,
        osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names
    )

    ps = PlotStyler()
    u1 = robot_input['u'][:, 6:8]
    u2 = robot_input2['u'][:, 6:8]
    tu1 = robot_input['t_u'] - robot_input['t_u'][0]
    tu2 = robot_input2['t_u'] - robot_input2['t_u'][0]
    plt.gca().set_prop_cycle(plt.cycler(color=[ps.red, ps.blue]))
    ps.plot(tu1, u1, linestyle='--')
    ps.plot(tu2, u2, linestyle='-')
    ps.add_legend([
        'Knee Torque (left) - FCCQP',
        'Knee Torque (right) - FCCQP',
        'Knee Torque (left) - OSQP',
        'Knee Torque (right) - OSQP'])

    mbp_plots.add_fsm_to_plot(
        ps,
        osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names
    )


def main():
    config_folder = 'bindings/pydairlib/analysis/plot_configs/'
    config_file = config_folder + 'cassie_default_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    filename1 = "/media/brian/tb2/cassie_backup/logs/cassie_hardware/2024/03_29_24/lcmlog-00"
    filename2 = ("/media/brian/tb2/cassie_backup/logs/cassie_hardware/2024"
                 "/03_29_24/lcmlog-01")
    log = lcm.EventLog(filename1, "r")
    log2 = lcm.EventLog(filename2, "r")
    plotter_main(plot_config, log, log2)

    plt.show()


if __name__ == '__main__':
    main()
