import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
from process_lcm_log import get_log_data
from cassie_plot_config import CassiePlotConfig
import cassie_plotting_utils as cassie_plots
import mbp_plotting_utils as mbp_plots
from pydairlib.common.plot_styler import PlotStyler


def plotter_main(plot_config, log):
    """
        workhorse function to construct plots given an lcmlog and a plot config.
        Note that to allow for more modularity, this function
        doesn't call plt.show()
    """

    use_floating_base = plot_config.use_floating_base
    use_springs = plot_config.use_springs

    channel_x = plot_config.channel_x
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
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)

    if plot_config.plot_contact_forces:
        contact_output = get_log_data(log,  # log
            cassie_plots.cassie_contact_channels,  # lcm channels
            plot_config.start_time,
            plot_config.duration,
            mbp_plots.load_force_channels,  # processing callback
            'CASSIE_CONTACT_DRAKE')  # processing callback arguments

    ps = PlotStyler()
    ps.set_default_styling()

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)
    print('Log start time: ', robot_output['t_x'][0])

    ''' Plot Positions '''
    # Plot floating base positions if applicable
    if use_floating_base and plot_config.plot_floating_base_positions:
        plot = mbp_plots.plot_floating_base_positions(
            robot_output, pos_names, 7, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names)

    # Plot joint positions
    if plot_config.plot_joint_positions:
        mbp_plots.plot_joint_positions(robot_output, pos_names,
            7 if use_floating_base else 0, t_x_slice)
    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(
            robot_output, plot_config.pos_names, t_x_slice, pos_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names)

    ''' Plot Velocities '''
    # Plot floating base velocities if applicable
    if use_floating_base and plot_config.plot_floating_base_velocities:
        plot = mbp_plots.plot_floating_base_velocities(
            robot_output, vel_names, 6, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names)

    if use_floating_base and plot_config.plot_floating_base_velocity_body_frame:
        plot = mbp_plots.plot_floating_base_body_frame_velocities(
            robot_output, t_x_slice, plant, context, "pelvis")
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names)

    # Plot all joint velocities
    if plot_config.plot_joint_positions:
        mbp_plots.plot_joint_velocities(robot_output, vel_names,
            6 if use_floating_base else 0,
            t_x_slice)
    # Plot specific velocities
    if plot_config.vel_names:
        mbp_plots.plot_velocities_by_name(robot_output, plot_config.vel_names,
            t_x_slice, vel_map)

    ''' Plot Efforts '''
    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_commanded_efforts:
        plot = mbp_plots.plot_commanded_efforts(robot_input, act_names, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.act_names:
        plot = mbp_plots.plot_measured_efforts_by_name(robot_output,
            plot_config.act_names,
            t_x_slice, act_map)
        mbp_plots.add_fsm_to_plot(plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    ''' Plot OSC '''
    if plot_config.plot_qp_costs:
        mbp_plots.plot_qp_costs(osc_debug, t_osc_slice)
    if plot_config.plot_tracking_costs:
        mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)

    if plot_config.tracking_datas_to_plot:
        for traj_name, config in plot_config.tracking_datas_to_plot.items():
            for deriv in config['derivs']:
                for dim in config['dims']:
                    plot = mbp_plots.plot_osc_tracking_data(osc_debug, traj_name, dim,
                        deriv, t_osc_slice)
                    mbp_plots.add_fsm_to_plot(plot,
                        osc_debug['t_osc'],
                        osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_qp_solve_time:
        mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)

    ''' Plot Foot Positions '''
    if plot_config.foot_positions_to_plot:
        _, pts_map = cassie_plots.get_toe_frames_and_points(plant)
        foot_frames = []
        dims = {}
        pts = {}
        for pos in plot_config.foot_positions_to_plot:
            foot_frames.append('toe_' + pos)
            dims['toe_' + pos] = plot_config.foot_xyz_to_plot[pos]
            pts['toe_' + pos] = pts_map[plot_config.pt_on_foot_to_plot]

        plot = mbp_plots.plot_points_positions(
            robot_output, t_x_slice, plant, context, foot_frames, pts, dims)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names)

    ''' Plot IMU acceleration '''
    if True:
        ps = mbp_plots.plot_imu_accel(imu_accel)
        mbp_plots.add_fsm_to_plot(ps, osc_debug['t_osc'],
            osc_debug['fsm'], plot_config.fsm_state_names)


def main():
    config_folder = 'bindings/pydairlib/analysis/plot_configs/'
    config_file = config_folder + 'cassie_default_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    plotter_main(plot_config, log)

    plt.show()


if __name__ == '__main__':
    main()
