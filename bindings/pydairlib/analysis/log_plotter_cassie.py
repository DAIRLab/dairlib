import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

import dairlib
from process_lcm_log import get_log_data
import cassie_plotting_utils as cassie_plots


def main():

    # Global settings for plotting
    use_floating_base = True
    use_springs = True

    # TODO: get these channels automatically from reading the log
    channel_x = 'CASSIE_STATE_SIMULATION'
    channel_u = 'CASSIE_INPUT'
    channel_osc = 'OSC_DEBUG_STANDING'

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)

    pos_map, vel_map, act_map = cassie_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = cassie_plots.make_mbp_name_vectors(plant)


    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input, osc_debug = \
        get_log_data(log,                                               # log
                     cassie_plots.cassie_default_channels,              # lcm channels
                     cassie_plots.load_default_channels,                # processing callback
                     plant, channel_x, channel_u, channel_osc)          # processing callback arguments

    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)

    ''' Plot Positions '''
    # Plot floating base positions if applicable
    if use_floating_base:
        cassie_plots.plot_floating_base_positions(
            robot_output, pos_names, t_x_slice)

    # Plot joint positions
    cassie_plots.plot_joint_positions(robot_output, pos_names, t_x_slice,
                                      floating_base=use_floating_base)
    # Plot specific positions
    cassie_plots.plot_positions_by_name(robot_output,
                                        ['knee_left', 'knee_right'],
                                        t_x_slice, pos_map)

    ''' Plot Velocities '''
    # Plot floating base velocities if applicable
    if use_floating_base:
        cassie_plots.plot_floating_base_velocities(
            robot_output, vel_names, t_x_slice)

    # Plot all joint velocities
    cassie_plots.plot_joint_velocities(robot_output, vel_names, t_x_slice,
                                       floating_base=use_floating_base)
    # Plot specific velocities
    cassie_plots.plot_velocities_by_name(robot_output, ['base_vz'], t_x_slice, vel_map)

    ''' Plot Efforts '''
    cassie_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
    cassie_plots.plot_measured_efforts_by_name(robot_output,
                                               ['knee_left_motor'],
                                               t_x_slice, act_map)
    plt.show()


if __name__ == '__main__':
    main()
