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
    config_file = 'bindings/pydairlib/analysis/plot_configs/franka_translation_plot.yaml'
    plot_config = FrankaPlotConfig(config_file)

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc

    if plot_config.plot_style == "paper":
        plot_styler.PlotStyler.set_default_styling()
    elif plot_config.plot_style == "compact":
        plot_styler.PlotStyler.set_compact_styling()

    ''' Get the plant '''
    franka_plant, franka_context, tray_plant, tray_context = franka_plots.make_plant_and_context()

    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(franka_plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(
        franka_plant)

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

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))

    franka_joint_limits_lower = np.zeros(franka_plant.num_positions())
    franka_joint_limits_upper = np.zeros(franka_plant.num_positions())
    franka_joint_velocity_limits_lower = np.zeros(franka_plant.num_positions())
    franka_joint_velocity_limits_upper = np.zeros(franka_plant.num_positions())
    franka_joint_actuator_limits_lower = np.zeros(franka_plant.num_positions())
    franka_joint_actuator_limits_upper = np.zeros(franka_plant.num_positions())
    for i in range(franka_plant.num_positions()):
        franka_joint_limits_upper[i] = franka_plant.get_joint(JointIndex(i)).position_upper_limits()[0]
        franka_joint_limits_lower[i] = franka_plant.get_joint(JointIndex(i)).position_lower_limits()[0]
        franka_joint_velocity_limits_lower[i] = franka_plant.get_joint(JointIndex(i)).velocity_upper_limits()[0]
        franka_joint_velocity_limits_upper[i] = franka_plant.get_joint(JointIndex(i)).velocity_lower_limits()[0]
        franka_joint_actuator_limits_lower[i] = -franka_plant.get_joint_actuator(JointActuatorIndex(i)).effort_limit()
        franka_joint_actuator_limits_upper[i] = franka_plant.get_joint_actuator(JointActuatorIndex(i)).effort_limit()
    franka_joint_limit_range = [np.min(franka_joint_limits_lower), np.max(franka_joint_limits_upper)]
    franka_joint_velocity_limit_range = [np.min(franka_joint_velocity_limits_lower), np.max(franka_joint_velocity_limits_upper)]
    franka_joint_actuator_limit_range = [np.min(franka_joint_actuator_limits_lower), np.max(franka_joint_actuator_limits_upper)]

    # Plot joint positions
    if plot_config.plot_joint_positions:
        plot = mbp_plots.plot_joint_positions(robot_output, pos_names,
                                              0, t_x_slice)
        plt.ylim(franka_joint_limit_range)

    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(robot_output,
                                                plot_config.pos_names,
                                                t_x_slice, pos_map)

    # Plot all joint velocities
    if plot_config.plot_joint_velocities:
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names,
                                               0,
                                               t_x_slice)
        plt.ylim(franka_joint_velocity_limit_range)


# Plot specific velocities
    if plot_config.vel_names:
        plot = mbp_plots.plot_velocities_by_name(robot_output,
                                                 plot_config.vel_names,
                                                 t_x_slice, vel_map)

    if plot_config.plot_end_effector:
        mbp_plots.plot_points_positions(robot_output, t_x_slice, franka_plant,
                                        franka_context, ['paddle'],
                                        {'paddle': np.zeros(3)},
                                        {'paddle': [0, 1, 2]})

    mbp_plots.plot_points_velocities(robot_output, t_x_slice, franka_plant,
                                     franka_context, ['paddle'],
                                     {'paddle': np.zeros(3)},
                                     {'paddle': [0, 1, 2]})
    # q = np.load(
    #     '/home/yangwill/Documents/research/projects/franka/leon_data/test_2023-05-19-14-10-43_q.npy')
    # v = np.load(
    #     '/home/yangwill/Documents/research/projects/franka/leon_data/test_2023-05-19-14-10-43_v.npy')
    # t = np.load(
    #     '/home/yangwill/Documents/research/projects/franka/leon_data/test_2023-05-19-14-10-43_t.npy')
    # pos = mbp_plots.make_point_positions_from_q(q,
    #                                       franka_plant, franka_context,
    #                                       franka_plant.GetBodyByName(
    #                                           'paddle').body_frame(),
    #                                       np.zeros(3))
    # vel = mbp_plots.make_point_velocities_from_qv(q,
    #                                               v,
    #                                               franka_plant, franka_context,
    #                                               franka_plant.GetBodyByName(
    #                                                   'paddle').body_frame(),
    #                                               np.zeros(3))
    # ps = plot_styler.PlotStyler()
    # ps.plot(t, pos, title='leon_experiment_positions')
    # ps = plot_styler.PlotStyler()
    # ps.plot(t, vel, title='leon_experiment_velocities')
    # ps = plot_styler.PlotStyler()

    # ps.plot(t[1:], 1 / np.diff(t) * np.diff(vel, axis=0)[:, 1],
    #         title='leon_experiment_accelerations', ylim=[-10, 10])

    ''' Plot Efforts '''
    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(robot_output, act_names,
                                               t_x_slice)
        plt.ylim(franka_joint_actuator_limit_range)

    if plot_config.plot_commanded_efforts:
        plot = mbp_plots.plot_commanded_efforts(robot_input, act_names,
                                                t_osc_slice)
        plt.ylim(franka_joint_actuator_limit_range)

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
        plot = mbp_plots.plot_lambda_c_sol(osc_debug, t_osc_slice, slice(0, 12))
        plot = mbp_plots.plot_lambda_h_sol(osc_debug, t_osc_slice, slice(0, 6))

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

    plt.show()


if __name__ == '__main__':
    main()
