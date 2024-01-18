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

from pydrake.all import JointIndex, JointActuatorIndex


def main():
    # config_file = ('bindings/pydairlib/analysis/plot_configs'
    #                '/franka_translation_plot.yaml')
    config_file = ('bindings/pydairlib/analysis/plot_configs'
                   '/franka_sim_plot_config.yaml')
    plot_config = FrankaPlotConfig(config_file)

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc
    channel_c3 = plot_config.channel_c3
    channel_c3_target = plot_config.channel_c3_target
    channel_c3_actual = plot_config.channel_c3_actual
    channel_tray = plot_config.channel_tray

    if plot_config.plot_style == "paper":
        plot_styler.PlotStyler.set_default_styling()
    elif plot_config.plot_style == "compact":
        plot_styler.PlotStyler.set_compact_styling()

    ''' Get the plant '''
    franka_plant, franka_context, tray_plant, tray_context = (
        franka_plots.make_plant_and_context())

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(franka_plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(
        franka_plant)
    tray_pos_map, tray_vel_map, _ = mbp_plots.make_name_to_mbp_maps(tray_plant)

    tray_pos_names, tray_vel_names, _ = mbp_plots.make_mbp_name_vectors(
        tray_plant)

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
                     franka_plant, channel_x, channel_u,
                     channel_osc)  # processing callback arguments

    # processing callback arguments
    if plot_config.plot_c3_debug:
        c3_output, c3_tracking_target, c3_tracking_actual = get_log_data(log, default_channels, plot_config.start_time,
                                 plot_config.duration, mbp_plots.load_c3_debug,
                                 channel_c3, channel_c3_target, channel_c3_actual)
        print('Average C3 frequency: ', 1 / np.mean(np.diff(c3_output['t'])))

    # processing callback arguments
    if plot_config.plot_object_state:
        object_state = get_log_data(log, default_channels,
                                    plot_config.start_time,
                                    plot_config.duration,
                                    mbp_plots.load_object_state,
                                    channel_tray)
        t_object_slice = slice(object_state['t'].size)

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))


    (franka_joint_position_limit_range, franka_joint_velocity_limit_range,
     franka_joint_actuator_limit_range) = mbp_plots.generate_joint_limits(
        franka_plant)

    # Plot joint positions
    if plot_config.plot_joint_positions:
        plot = mbp_plots.plot_joint_positions(robot_output, pos_names, 0,
                                              t_x_slice)
        plt.ylim(franka_joint_position_limit_range)
    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(robot_output,
                                                plot_config.pos_names,
                                                t_x_slice, pos_map)

    # import pdb; pdb.set_trace()
    if plot_config.plot_c3_debug:
        # t_c3_slice = slice(c3_output['t'].size)
        # mbp_plots.plot_c3_inputs(c3_output, t_c3_slice, 0)

        t_c3_slice = slice(c3_output['t'].size)
        plot = mbp_plots.plot_c3_inputs(c3_output, t_c3_slice, 1)
        plot.axes[0].axhline(y=8.06, color='r', linestyle='-')
        plot.axes[0].axhline(y=-8.06, color='r', linestyle='-')




        # t_c3_slice = slice(c3_output['t'].size)
        # mbp_plots.plot_c3_inputs(c3_output, t_c3_slice, 2)

    if plot_config.plot_c3_tracking:
        plot = plot_styler.PlotStyler(nrows=2)
        plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 0:3], subplot_index = 0)
        plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 0:3], subplot_index = 0)
        plot.add_legend(['robot_des_x', 'robot_des_y', 'robot_des_z', 'robot_x', 'robot_y', 'robot_z'], subplot_index = 0)
        plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 7:10], subplot_index = 1)
        plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 7:10], subplot_index = 1)
        plot.add_legend(['tray_des_x', 'tray_des_y', 'tray_des_z', 'tray_x', 'tray_y', 'tray_z'], subplot_index = 1)
    # plt.plot(c3_output['t'], c3_output['x'][:, 0, :])

    # Plot all joint velocities
    if plot_config.plot_joint_velocities:
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names, 0,
                                               t_x_slice)
        plt.ylim(franka_joint_velocity_limit_range)
        plt.axhline(franka_joint_velocity_limit_range[0], linestyle='--')
        plt.axhline(franka_joint_velocity_limit_range[1], linestyle='--')

    # Plot specific velocities
    if plot_config.vel_names:
        plot = mbp_plots.plot_velocities_by_name(robot_output,
                                                 plot_config.vel_names,
                                                 t_x_slice, vel_map)

    if plot_config.plot_end_effector:
        end_effector_plotter = plot_styler.PlotStyler(nrows=2)
        mbp_plots.plot_points_positions(robot_output, t_x_slice, franka_plant,
                                        franka_context, ['plate'],
                                        {'plate': np.zeros(3)},
                                        {'plate': [0, 1, 2]},
                                        ps=end_effector_plotter,
                                        subplot_index=0)

        mbp_plots.plot_points_velocities(robot_output, t_x_slice, franka_plant,
                                         franka_context, ['plate'],
                                         {'plate': np.zeros(3)},
                                         {'plate': [0, 1, 2]},
                                         ps=end_effector_plotter,
                                         subplot_index=1)

    ''' Plot Efforts '''
    effort_plotter = plot_styler.PlotStyler(nrows=2)
    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(robot_output, act_names,
                                               t_x_slice, effort_plotter,
                                               subplot_index=0)
        plot.fig.axes[0].set_ylim(franka_joint_actuator_limit_range)

    if plot_config.plot_commanded_efforts:
        plot = mbp_plots.plot_commanded_efforts(robot_input, act_names,
                                                t_osc_slice, effort_plotter,
                                                subplot_index=1)
        plot.fig.axes[1].set_ylim(franka_joint_actuator_limit_range)

    if plot_config.act_names:
        plot = mbp_plots.plot_measured_efforts_by_name(robot_output,
                                                       plot_config.act_names,
                                                       t_x_slice, act_map)

    ''' Plot OSC '''
    if plot_config.plot_tracking_costs:
        plot = mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)
        plt.ylim([0, 1e3])

    if plot_config.plot_qp_costs:
        plot = mbp_plots.plot_qp_costs(osc_debug, t_osc_slice)

    if plot_config.plot_qp_solutions:
        plot = mbp_plots.plot_ddq_sol(osc_debug, t_osc_slice, pos_names,
                                      slice(0, 7))
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names, 0,
                                               t_x_slice)
        plot = mbp_plots.plot_lambda_c_sol(osc_debug, t_osc_slice, slice(0, 3))
        # plot = mbp_plots.plot_lambda_h_sol(osc_debug, t_osc_slice, slice(0,
        # 6))

    if plot_config.tracking_datas_to_plot:
        for traj_name, config in plot_config.tracking_datas_to_plot.items():
            for deriv in config['derivs']:
                for dim in config['dims']:
                    plot = mbp_plots.plot_osc_tracking_data(osc_debug,
                                                            traj_name, dim,
                                                            deriv, t_osc_slice)
                    tracking_data = osc_debug['osc_debug_tracking_datas'][
                        traj_name]

    if plot_config.plot_qp_solve_time:
        plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)

    if plot_config.plot_active_tracking_datas:
        plot = mbp_plots.plot_active_tracking_datas(osc_debug, t_osc_slice,
                                                    osc_debug['t_osc'],
                                                    osc_debug['fsm'],
                                                    plot_config.fsm_state_names)
        # plot.save_fig('active_tracking_datas.png')

    if plot_config.plot_object_state:
        plot = mbp_plots.plot_positions_by_name(object_state,
                                                tray_pos_names[4:],
                                                t_object_slice, tray_pos_map)
        # plot.save_fig(('/').join(filename.split('/')[-2:]) + '/object_position')
    if plot_config.plot_object_state:
        plot = mbp_plots.plot_positions_by_name(object_state,
                                                tray_pos_names[:4],
                                                t_object_slice, tray_pos_map)

    plt.show()


if __name__ == '__main__':
    main()
