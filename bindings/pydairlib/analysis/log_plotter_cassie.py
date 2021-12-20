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
import mpc_debug as mpc


def main():
    config_file = \
        'bindings/pydairlib/analysis/plot_configs/cassie_default_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    use_floating_base = plot_config.use_floating_base
    use_springs = plot_config.use_springs

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc
    channel_mpc = plot_config.channel_mpc

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)
    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)
    I_srbd = np.diag([0.91, 0.55, 0.89])

    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input, osc_debug = \
        get_log_data(log,                                       # log
                     cassie_plots.cassie_default_channels,      # lcm channels
                     mbp_plots.load_default_channels,           # processing callback
                     plant, channel_x, channel_u, channel_osc)  # processing callback arguments
    mpc_data = get_log_data(log,
                            {channel_mpc: dairlib.lcmt_saved_traj},
                            mpc.process_mpc_channel, channel_mpc)

    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    ''' Plot Positions '''
    # Plot floating base positions if applicable
    if use_floating_base and plot_config.plot_floating_base_positions:
        mbp_plots.plot_floating_base_positions(
            robot_output, pos_names, 7, t_x_slice)

    # Plot joint positions
    if plot_config.plot_joint_positions:
        mbp_plots.plot_joint_positions(robot_output, pos_names,
                                       7 if use_floating_base else 0, t_x_slice)
    # Plot specific positions
    if plot_config.pos_names:
        mbp_plots.plot_positions_by_name(robot_output, plot_config.pos_names,
                                         t_x_slice, pos_map)

    ''' Plot Velocities '''
    # Plot floating base velocities if applicable
    if use_floating_base and plot_config.plot_floating_base_velocities:
        mbp_plots.plot_floating_base_velocities(
            robot_output, vel_names, 6, t_x_slice)

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
        mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
    if plot_config.act_names:
        mbp_plots.plot_measured_efforts_by_name(robot_output,
                                                plot_config.act_names,
                                                t_x_slice, act_map)

    ''' Plot OSC '''
    if plot_config.plot_qp_costs:
        mbp_plots.plot_qp_costs(osc_debug, t_osc_slice)
    if plot_config.plot_tracking_costs:
        mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)
    if plot_config.plot_qp_solve_time:
        mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)

    if plot_config.tracking_datas_to_plot:
        for traj_name, config in plot_config.tracking_datas_to_plot.items():
            for deriv in config['derivs']:
                for dim in config['dims']:
                    mbp_plots.plot_osc_tracking_data(osc_debug, traj_name, dim,
                                                     deriv, t_osc_slice)

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

        mbp_plots.plot_points_positions(robot_output, t_x_slice, plant, context,
                                        foot_frames, pts, dims)

    ''' Plot MPC solutions '''
    for traj, config in plot_config.mpc_trajs_to_plot.items():
        for dim in config['dims']:
            mpc.plot_mpc_traj(mpc_data, traj, dim)

    ''' Custom Plots '''
    if plot_config.plot_momentum:
        mbp_plots.plot_angular_momentum(robot_output, t_x_slice, plant, context,
                                        [0, 1, 2])

        mbp_plots.plot_angular_momentum_srbd(
            osc_debug['osc_debug_tracking_datas']['orientation_traj'].ydot,
            osc_debug['osc_debug_tracking_datas']['orientation_traj'].t,
            t_osc_slice,
            I_srbd, [0, 1, 2])

        mbp_plots.plot_planned_linear_momentum(
            osc_debug['osc_debug_tracking_datas']['com_traj'].ydot_des,
            osc_debug['osc_debug_tracking_datas']['com_traj'].t,
            t_osc_slice,
            30.0218, [0, 1, 2])

        mbp_plots.plot_linear_momentum(robot_output, t_x_slice, plant, context,
                                       [0, 1, 2])

    plt.show()


if __name__ == '__main__':
    main()
