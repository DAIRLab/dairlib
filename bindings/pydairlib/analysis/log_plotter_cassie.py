import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

import dairlib
from pydairlib.multibody import createStateNameVectorFromMap
from process_lcm_log import get_log_data, passthrough_callback
import cassie_plotting_utils


def main():
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")

    plant, context = cassie_plotting_utils.make_plant_and_context(
        floating_base=True, springs=True)

    x_names = createStateNameVectorFromMap(plant)
    q_names = x_names[:plant.num_positions()]
    v_names = x_names[plant.num_positions():]

    robot_output, robot_input, osc_debug = \
        get_log_data(log,
                     cassie_plotting_utils.cassie_default_channels,
                     cassie_plotting_utils.load_default_channels,
                     plant,
                     'CASSIE_STATE_SIMULATION',
                     'CASSIE_INPUT',
                     'OSC_DEBUG_STANDING')

    t_x_slice = slice(robot_output['t_x'].size)

    plot_floating_base_positions(robot_output, q_names, t_x_slice)
    plot_joint_positions(robot_output, q_names, t_x_slice, floating_base=True)

    plt.show()


def plot_floating_base_positions(robot_output, q_names, time_slice):
    cassie_plotting_utils.make_plot(
        robot_output,                       # data dict
        't_x',                              # time channel
        time_slice,
        ['q'],                              # key to plot
        {'q': slice(0, 7)},                 # slice of 'q' to plot
        {'q': q_names[:7]},                 # legend entries
        {'xlabel': 'Time',
         'ylabel': 'Position',
         'title': 'Floating Base Positions'})


def plot_joint_positions(robot_output, q_names, time_slice, floating_base=True):
    q_slice = slice(7, len(q_names)) if floating_base else slice(len(q_names))
    cassie_plotting_utils.make_plot(
        robot_output,                       # data dict
        't_x',                              # time channel
        time_slice,
        ['q'],                              # key to plot
        {'q': q_slice},                     # slice of 'q' to plot
        {'q': q_names[q_slice]},            # legend entries
        {'xlabel': 'Time',
         'ylabel': 'Joint Angle (rad)',
         'title': 'Joint Positions'})

if __name__ == '__main__':
    main()

