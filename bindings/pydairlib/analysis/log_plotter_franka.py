import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np
import pdb

from bindings.pydairlib.analysis.franka_plot_config import FrankaPlotConfig
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
from bindings.pydairlib.lcm.process_lcm_log import print_log_summary
import franka_plotting_utils as franka_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.common import plot_styler

from pydrake.all import JointIndex, JointActuatorIndex

from matplotlib.patches import Patch

def main():
    # Parse the command line arguments for the log file
    filename = sys.argv[1]
    print(f'FILENAME {filename}')

    if filename.split('/')[-1].startswith('hwlog'):
    # if file name starts with hwlog, use the hardware plot config
        print('Using hardware plot config')
        config_file = ('bindings/pydairlib/analysis/plot_configs'
                       '/franka_hardware_plot_config.yaml')
    else:
        # otherwise, use the simulation plot config
        print('Using simulation plot config')
        config_file = ('bindings/pydairlib/analysis/plot_configs'
                       '/franka_sim_plot_config.yaml')
    
    plot_config = FrankaPlotConfig(config_file)

    franka_state_channel = plot_config.franka_state_channel
    osc_channel = plot_config.osc_channel
    osc_debug_channel = plot_config.osc_debug_channel
    c3_debug_output_curr_channel = plot_config.c3_debug_output_curr_channel
    c3_debug_output_best_channel = plot_config.c3_debug_output_best_channel
    c3_target_state_channel = plot_config.c3_target_state_channel
    c3_actual_state_channel = plot_config.c3_actual_state_channel
    object_state_channel = plot_config.object_state_channel

    if plot_config.plot_style == "paper":
        plot_styler.PlotStyler.set_paper_styling()
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
    log = lcm.EventLog(filename, "r")
    print("PRINTING CHANNEL NAMES")
    print_log_summary(filename, log)
    default_channels = franka_plots.franka_default_channels
    print(default_channels)
    robot_output, robot_input, osc_debug = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     franka_plant, franka_state_channel, osc_channel,
                     osc_debug_channel)  # processing callback arguments

    # processing callback arguments
    if plot_config.plot_c3_debug or plot_config.plot_errors_with_mode_highlights:
        c3_output_curr, c3_output_best, c3_tracking_target, c3_tracking_actual=\
            get_log_data(log, 
                         default_channels, plot_config.start_time,
                         plot_config.duration, mbp_plots.load_c3_debug,
                         c3_debug_output_curr_channel,
                         c3_debug_output_best_channel,
                         c3_target_state_channel,
                         c3_actual_state_channel)
        solve_times = np.diff(c3_output_curr['t'], prepend=[c3_output_curr['t'][0]])
        print('Average C3 frequency: ', 1 / np.mean(np.diff(c3_output_curr['t'])))

    # processing callback arguments
    if plot_config.plot_lcs_debug:
        lcs_curr, lcs_best, c3_target, c3_actual, c3_forces_curr, c3_debug_curr = \
            get_log_data(
                log, default_channels,
                plot_config.start_time,
                plot_config.duration,
                mbp_plots.load_lcs_debug,
                plot_config.dynamically_feasible_curr_plan_channel,
                plot_config.dynamically_feasible_best_plan_channel,
                c3_target_state_channel,
                c3_actual_state_channel,
                plot_config.c3_force_curr_channel,
                plot_config.c3_debug_output_curr_channel)
        t_slice = slice(lcs_curr['t'].size)

    if plot_config.plot_object_state:
        object_state = get_log_data(log, default_channels,
                                    plot_config.start_time,
                                    plot_config.duration,
                                    mbp_plots.load_object_state,
                                    object_state_channel)
        t_object_slice = slice(object_state['t'].size)

    if plot_config.plot_sample_costs:
        time_sample_costs_dict = get_log_data(log, default_channels,
                                    plot_config.start_time,
                                    plot_config.duration,
                                    mbp_plots.load_sample_costs,
                                    plot_config.sample_costs_channel)
    
    if plot_config.plot_curr_and_best_sample_costs:
        time_curr_and_best_costs_dict = get_log_data(log, default_channels,
                                    plot_config.start_time,
                                    plot_config.duration,
                                    mbp_plots.load_curr_and_best_costs,
                                    plot_config.curr_and_best_costs_channel)
    
    if plot_config.plot_is_c3_mode:
        time_is_c3_mode_dict = get_log_data(log, default_channels,
                                    plot_config.start_time,
                                    plot_config.duration,
                                    mbp_plots.load_is_c3_mode,
                                    plot_config.is_c3_mode_channel)

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    # print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))


    (franka_joint_position_limit_range, franka_joint_velocity_limit_range,
     franka_joint_actuator_limit_range) = mbp_plots.generate_joint_limits(
        franka_plant)
    
    if plot_config.plot_lcs_debug:
        plot = mbp_plots.plot_lcs_debug(
            lcs_curr, lcs_best, c3_target, c3_actual, c3_forces_curr,
            c3_debug_curr, t_slice)
    
    if plot_config.plot_sample_costs:
        t_sample_costs_slice = slice(time_sample_costs_dict['t'].size)
        if (plot_config.plot_is_c3_mode):
            print(f'First: {time_is_c3_mode_dict["is_c3_mode"].shape}')
            plot = mbp_plots.plot_sample_costs(
                time_sample_costs_dict,
                t_sample_costs_slice,
                time_is_c3_mode_dict)
        else:
            plot = mbp_plots.plot_sample_costs(
                time_sample_costs_dict,
                t_sample_costs_slice)

    if plot_config.plot_curr_and_best_sample_costs:
        t_curr_and_best_costs_slice = slice(time_curr_and_best_costs_dict['t'].size)
        if (plot_config.plot_is_c3_mode):
            print(f'Second: {time_is_c3_mode_dict["is_c3_mode"].shape}')
            plot = mbp_plots.plot_curr_and_best_costs(
                time_curr_and_best_costs_dict,
                t_curr_and_best_costs_slice,
                time_is_c3_mode_dict)
        else:
            plot = mbp_plots.plot_curr_and_best_costs(
                time_curr_and_best_costs_dict,
                t_curr_and_best_costs_slice)

    if plot_config.plot_errors_with_mode_highlights:
        print(f'Third: {time_is_c3_mode_dict["is_c3_mode"].shape}')
        t_is_c3_mode_slice = slice(time_is_c3_mode_dict['t'].size)
        plot = mbp_plots.plot_object_position_error(
            c3_tracking_actual, c3_tracking_target, time_is_c3_mode_dict,
            t_is_c3_mode_slice, orientation_too=True)

    if plot_config.plot_is_c3_mode:
        # plots the c3 vs repositioning in separate plot
        t_is_c3_mode_slice = slice(time_is_c3_mode_dict['t'].size)
        print(f'Fourth: {time_is_c3_mode_dict["is_c3_mode"].shape}')
        plot = mbp_plots.plot_is_c3_mode(time_is_c3_mode_dict, t_is_c3_mode_slice)


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
        # Plot debug infor for c3 solves curr and best
        # t_c3_slice = slice(c3_output['t'].size)
        # mbp_plots.plot_c3_inputs(c3_output, t_c3_slice, 0)

        t_c3_slice_curr = slice(c3_output_curr['t'].size)
        plot_curr = mbp_plots.plot_c3_inputs(c3_output_curr, t_c3_slice_curr, 1)
        plot_curr.axes[0].axhline(y=8.06, color='r', linestyle='-')
        plot_curr.axes[0].axhline(y=-8.06, color='r', linestyle='-')
        # set plot title
        plot_curr.axes[0].set_title('C3 PLAN CURR')
        # plot.save_fig('c3_inputs_' + filename.split('/')[-1])

        t_c3_slice_best = slice(c3_output_best['t'].size)
        plot_best = mbp_plots.plot_c3_inputs(c3_output_best, t_c3_slice_best, 1)
        plot_best.axes[0].axhline(y=8.06, color='r', linestyle='-')
        plot_best.axes[0].axhline(y=-8.06, color='r', linestyle='-')
        # set plot title
        plot_best.axes[0].set_title('C3 PLAN BEST')




        # t_c3_slice = slice(c3_output_curr['t'].size)
        # mbp_plots.plot_c3_inputs(c3_output_curr, t_c3_slice, 2)

    if plot_config.plot_c3_tracking:
        # plot = plot_styler.PlotStyler(nrows=2)
        plot = plot_styler.PlotStyler()

        # plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 0:3], subplot_index = 0)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 0:3], subplot_index = 0)
        # plot.add_legend(['robot_des_x', 'robot_des_y', 'robot_des_z', 'robot_x', 'robot_y', 'robot_z'], subplot_index = 0)
        # plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 7:10], subplot_index = 1)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 7:10], subplot_index = 1)
        # plot.add_legend(['tray_des_x', 'tray_des_y', 'tray_des_z', 'tray_x', 'tray_y', 'tray_z'], subplot_index = 1)

        # plot target
        # plot.plot(c3_tracking_actual['t'], c3_tracking_target['x'][:, 7:10], subplot_index = 0, ylabel='target position (m)', xlabel='time (s)', grid=False)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 7:10], subplot_index = 0, ylabel='target position (m)', xlabel='time (s)', grid=False)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 0:3], subplot_index = 0, ylabel='target position (m)', xlabel='time (s)', grid=False)

        # plots between end effector y and tray y
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 8:9], subplot_index = 0, ylabel='y position (m)', xlabel='time (s)', grid=False)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 1:2], subplot_index = 0, ylabel='y position (m)', xlabel='time (s)', grid=False)

        # plots y - z trajectories
        # plot.axes[0].scatter(c3_tracking_target['x'][0, 7], c3_tracking_target['x'][0, 9], marker='s')
        # plot.plot(c3_tracking_actual['x'][:, 7:8], c3_tracking_actual['x'][:, 9:10], subplot_index = 0, xlabel='X Position (m)', ylabel='Z Position (m)', grid=False)
        # plot.plot(c3_tracking_actual['x'][:, 0:1], c3_tracking_actual['x'][:, 2:3], subplot_index = 0, xlabel='X Position (m)', ylabel='Z Position (m)', grid=False)
        
        # plot.plot(c3_tracking_actual['t'], c3_tracking_target['x'][:, 7:10], subplot_index = 0, xlabel='Time (t)', grid=False)
        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 7:10], subplot_index = 0, xlabel='Time (t)', grid=False)
        plot.plot(c3_tracking_actual['t'], c3_tracking_target['x'][:, 7:10] - c3_tracking_actual['x'][:, 7:10], subplot_index = 0, xlabel='Time (t)', ylabel='Position Tracking Error (m)', grid=False)
        # plot.plot(c3_tracking_actual['x'][:, 7], c3_tracking_actual['x'][:, 8], subplot_index = 0, xlabel='Time (t)', grid=False)

        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 8:9], subplot_index = 0, xlabel='y position (m)', ylabel='z position (m)', grid=False)
        # plot.plot(c3_tracking_target['x'][:, 1:2], c3_tracking_target['x'][:, 2:3], subplot_index = 0)
        # plot.plot(c3_tracking_actual['x'][:, 1:2], c3_tracking_actual['x'][:, 2:3], subplot_index = 0, xlabel='y position (m)', ylabel='z position (m)', grid=False)

        # plot.plot(c3_tracking_actual['t'], c3_tracking_actual['x'][:, 7:8], subplot_index = 1)
        # plot.plot(c3_tracking_actual['x'][:, 7:8], c3_tracking_actual['x'][:, 7:8], subplot_index = 1)

        # plot.add_legend(['robot_des_x', 'robot_des_y', 'robot_des_z', 'robot_x', 'robot_y', 'robot_z'], subplot_index = 0)

        # plot.axes[0].set_xlim([0.4, 0.8])
        # plot.axes[0].set_ylim([0.35, 0.65])

        # plot.add_legend(['Tray Path', 'End Effector Path'])
        # plot.add_legend(['Target', 'End Effector Path'])
        plot.add_legend(['object tracking error x', 'object tracking error y', 'object tracking error z'])
        # plot.add_legend(['tray', 'end effector'])
        # plot.save_fig('c3_actual_xz_plot')


        # plot.save_fig('figure_8_tracking_over_time')
        # plot.save_fig('figure_8_tracking')
        # plot.save_fig('c3_gaiting')
        # plot.save_fig('c3_actual_trajectory_time')

        # plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 0:1], subplot_index = 0)
        # plot.plot(c3_tracking_target['t'], c3_tracking_target['x'][:, 7:8], subplot_index = 1)
        # plot.axes[0].set_ylim([0.4, 0.7])
        # plot.add_legend(['tray_des_x', 'tray_des_y', 'tray_des_z', 'tray_x', 'tray_y', 'tray_z'], subplot_index = 1)

    # plot = plot_styler.PlotStyler(nrows=2)
    # plot.plot(c3_tracking_target['t'], solve_times, subplot_index = 0)
    # plot.plot(c3_tracking_target['t'], c3_tracking_actual['x'][:, 0:1], subplot_index = 0)
    # plot.plot(c3_tracking_target['t'], c3_tracking_actual['x'][:, 7:8], subplot_index = 0)
    # plot.plot(c3_tracking_target['t'], solve_times, subplot_index = 1)
    # plot.plot(c3_tracking_target['t'], c3_tracking_actual['x'][:, 2:3], subplot_index = 1)
    # plot.plot(c3_tracking_target['t'], c3_tracking_actual['x'][:, 9:10], subplot_index = 1)

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
                                        franka_context, ['end_effector_tip'],
                                        {'end_effector_tip': np.zeros(3)},
                                        {'end_effector_tip': [0, 1, 2]},
                                        ps=end_effector_plotter,
                                        subplot_index=0)

        mbp_plots.plot_points_velocities(robot_output, t_x_slice, franka_plant,
                                         franka_context, ['end_effector_tip'],
                                         {'end_effector_tip': np.zeros(3)},
                                         {'end_effector_tip': [0, 1, 2]},
                                         ps=end_effector_plotter,
                                         subplot_index=1)

    ''' Plot Efforts '''
    if plot_config.plot_measured_efforts or plot_config.plot_commanded_efforts:
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
        ps = plot_styler.PlotStyler(nrows=2)
        mbp_plots.plot_positions_by_name(object_state,
                                                tray_pos_names[4:],
                                                t_object_slice, tray_pos_map, ps = ps, subplot_index = 0)
        mbp_plots.plot_positions_by_name(object_state,
                                                tray_pos_names[:4],
                                                t_object_slice, tray_pos_map, ps = ps, subplot_index = 1)
        # plot.save_fig(('/').join(filenme.split('/')[-2:]) + '/object_position')

    plt.show()


if __name__ == '__main__':
    main()
