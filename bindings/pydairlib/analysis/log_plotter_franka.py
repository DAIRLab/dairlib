import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

from bindings.pydairlib.analysis.franka_plot_config import FrankaPlotConfig
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
from cassie_plot_config import CassiePlotConfig
import franka_plotting_utils as franka_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.common import plot_styler


def main():
    config_file = 'bindings/pydairlib/analysis/plot_configs/franka_translation_plot.yaml'
    plot_config = FrankaPlotConfig(config_file)

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc

    if plot_config.use_default_styling:
        plot_styler.PlotStyler.set_default_styling()

    ''' Get the plant '''
    franka_plant, franka_context, tray_plant, tray_context = franka_plots.make_plant_and_context()

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(franka_plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(franka_plant)


    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    default_channels = franka_plots.franka_default_channels
    robot_output, robot_input, osc_debug = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     franka_plant, channel_x, channel_u, channel_osc)  # processing callback arguments

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))

    # Plot joint positions
    if plot_config.plot_joint_positions:
        plot = mbp_plots.plot_joint_positions(robot_output, pos_names,
                                              0, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(robot_output, plot_config.pos_names,
                                                t_x_slice, pos_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # Plot all joint velocities
    if plot_config.plot_joint_velocities:
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names,
                                               0,
                                               t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # Plot specific velocities
    if plot_config.vel_names:
        plot = mbp_plots.plot_velocities_by_name(robot_output, plot_config.vel_names,
                                                 t_x_slice, vel_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    ''' Plot Efforts '''
    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_commanded_efforts:
        plot = mbp_plots.plot_commanded_efforts(robot_input, act_names, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.act_names:
        plot = mbp_plots.plot_measured_efforts_by_name(robot_output,
                                                plot_config.act_names,
                                                t_x_slice, act_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


    ''' Plot OSC '''
    if plot_config.plot_tracking_costs:
        plot = mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
        plt.ylim([0, 1e3])
    if plot_config.plot_qp_costs:
        plot = mbp_plots.plot_qp_costs(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_qp_solutions:
        plot = mbp_plots.plot_lambda_c_sol(osc_debug, t_osc_slice, slice(0, 12))
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
        plot = mbp_plots.plot_lambda_h_sol(osc_debug, t_osc_slice, slice(0, 6))
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


    if plot_config.tracking_datas_to_plot:
        for traj_name, config in plot_config.tracking_datas_to_plot.items():
            for deriv in config['derivs']:
                for dim in config['dims']:
                    plot = mbp_plots.plot_osc_tracking_data(osc_debug, traj_name, dim,
                                                            deriv, t_osc_slice)
                    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
                    # plot.save_fig(traj_name + '_' + deriv + '_' + str(dim)  + '.png')
                    tracking_data = osc_debug['osc_debug_tracking_datas'][traj_name]
                    # print(tracking_data.name + str(dim))
                    # print('mean error: ' + str(np.nanmax(np.sqrt(tracking_data.error_y[:, dim].flatten()**2))))
                    # print('mean error at end of tracking: ' + str(np.mean(np.sqrt(tracking_data.error_y[:, dim][np.append(np.isnan(tracking_data.error_y[:, dim][1:]), False)]**2))))
                # plot = mbp_plots.plot_osc_tracking_data_in_space(osc_debug, traj_name, config['dims'], deriv, t_osc_slice)
                # plot.save_fig('swing_foot_target_trajectory.png')

    if plot_config.plot_qp_solve_time:
        plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_active_tracking_datas:
        plot = mbp_plots.plot_active_tracking_datas(osc_debug, t_osc_slice, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
        # plot.save_fig('active_tracking_datas.png')
    plt.show()


if __name__ == '__main__':
    main()
