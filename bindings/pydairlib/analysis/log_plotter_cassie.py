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

    print(act_names)
    print(pos_names)

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

    if plot_config.plot_battery_voltage:
        cassie_output = get_log_data(
            log,
            {plot_config.channel_cassie_out: dairlib.lcmt_cassie_out},
            plot_config.start_time,
            plot_config.duration,
            cassie_plots.load_cassie_out,
            plot_config.channel_cassie_out
        )

    ps = PlotStyler()
    ps.set_default_styling()

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)
    print('Log start time: ', robot_output['t_x'][0])
    # print(osc_debug['t_osc'].shape)
    # print(osc_debug['fsm'][:1000])
    # print(plot_config.fsm_state_names)

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
        plot = mbp_plots.plot_joint_velocities(
            robot_output, vel_names, 6 if use_floating_base else 0,
            t_x_slice)
        mbp_plots.add_fsm_to_plot(
            plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names
        )

    # Plot specific velocities
    if plot_config.vel_names:
        plot = mbp_plots.plot_velocities_by_name(
            robot_output, plot_config.vel_names, t_x_slice, vel_map
        )
        mbp_plots.add_fsm_to_plot(
            plot, osc_debug['t_osc'], osc_debug['fsm'],
            plot_config.fsm_state_names
        )

    ''' Plot Efforts '''
    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(robot_output, act_names, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

        plot = mbp_plots.plot_motor_power(robot_output, act_names, plant, t_x_slice)
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

    if plot_config.plot_battery_voltage:
        plot = cassie_plots.plot_battery_voltage(cassie_output)
        mbp_plots.add_fsm_to_plot(
            plot, osc_debug['t_osc'],
            osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_qp_solve_time:
        plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(
            plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names
        )

    ''' Plot Lambda'''
    if plot_config.plot_contact_forces:
        plot = mbp_plots.plot_lambda_c_sol(osc_debug, t_osc_slice, slice(0, 12, 1))
        mbp_plots.add_fsm_to_plot(
            plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names
        )

    if plot_config.act_names_to_validate:
        plot = mbp_plots.plot_measured_vs_commanded_efforts_by_name(
            robot_output, robot_input, plot_config.act_names_to_validate,
            act_map)
        mbp_plots.add_fsm_to_plot(
            plot,
            osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

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
    plot_imu = False
    if plot_imu:
        ps = mbp_plots.plot_imu_accel(imu_accel)
        mbp_plots.add_fsm_to_plot(ps, osc_debug['t_osc'],
            osc_debug['fsm'], plot_config.fsm_state_names)
        
    ''' Plot OOD window '''
    plot_ood = True
    if plot_ood:
        _, pts_map = cassie_plots.get_toe_frames_and_points(plant)
        foot_frames = []
        dims = {}
        pts = {}
        for pos in plot_config.foot_positions_to_plot:
            foot_frames.append('toe_' + pos)
            dims['toe_' + pos] = plot_config.foot_xyz_to_plot[pos]
            pts['toe_' + pos] = pts_map[plot_config.pt_on_foot_to_plot]
            
        ## Joint Position
        # ood_times = [53.013523, 53.313552, 53.613605, 53.913651, 54.215039, 54.517727, 54.821266, 55.128796, 55.436281, 55.743893, 56.052443, 56.361979, 56.670529, 56.978997, 57.288124, 57.597144, 57.90569, 58.214254, 58.523291, 58.832327, 59.140873, 59.449879, 59.758465, 60.067515, 60.37555, 60.682022, 60.987134, 61.290687, 61.591707, 61.891766, 62.191792, 62.491769, 62.791848, 63.091842, 63.391902]
        
        ## Ydot
        # ood_times = [24.326492, 24.927484, 25.227616, 26.427785, 26.728238, 27.628449, 30.929371, 34.832973, 36.672225, 37.907401, 42.812086, 44.612334, 44.912323, 45.212417, 45.51247, 47.612759, 48.512884, 49.71305, 50.613175, 52.713475, 55.128796, 55.743893, 56.670529, 57.597144, 60.37555, 60.987134]
        # plot = mbp_plots.plot_points_positions(
        #     robot_output, t_x_slice, plant, context, foot_frames, pts, dims)
        # plot = mbp_plots.plot_velocities_by_name(
        #     robot_output, plot_config.vel_names, t_x_slice, vel_map
        # )
        # mbp_plots.add_OOD_to_plot(osc_debug['t_osc'], osc_debug['fsm'],
        #     plot_config.fsm_state_names, ood_times1, ood_times2)

def save_main(plot_config, log):
    """
        Construct plots given an lcmlog and a plot config.
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

    # print(act_names)
    # print(pos_names)

    default_channels = cassie_plots.cassie_default_channels
    print(default_channels)
    robot_output, robot_input, osc_debug, imu_accel = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)
    if plot_config.plot_battery_voltage:
        cassie_output = get_log_data(
            log,
            {plot_config.channel_cassie_out: dairlib.lcmt_cassie_out},
            plot_config.start_time,
            plot_config.duration,
            cassie_plots.load_cassie_out,
            plot_config.channel_cassie_out
        )

    print('Finished processing log')
    # t_x_slice = slice(robot_output['t_x'].size)
    # t_osc_slice = slice(osc_debug['t_osc'].size)
    print('Log start time: ', robot_output['t_x'][0])
    actuator_index = 0
    actuator_data = robot_output['u'][:, actuator_index]

    for i in range(len(actuator_data)):
        if np.all(actuator_data[i:] == 0):
            end_index = i
            break
        else:
            end_index = len(actuator_data) - 1
    print(len(actuator_data))
    #print(end_index)
    print(robot_output['t_x'][end_index])
    end_index = end_index - 10000
    #print(end_index)
    robot_t_x = robot_output['t_x']
    start_time = robot_t_x[0]
    target_start_time = start_time + 2.5
    start_index = np.argmin(np.abs(robot_t_x - target_start_time))
    end_time = robot_output['t_x'][end_index]
    print(end_time)

    # Save commanded_efforts
    commanded_efforts = robot_input['u'][start_index:end_index]
    np.save('commanded_efforts.npy', commanded_efforts)

    # Save measured_efforts
    measured_efforts = robot_output['u'][start_index:end_index]
    np.save('measured_efforts.npy', measured_efforts)

    # Save joint_pos
    joint_pos = robot_output['q'][start_index:end_index]
    np.save('joint_pos.npy', joint_pos)

    # Save joint_vel
    joint_vel = robot_output['v'][start_index:end_index]
    np.save('joint_vel.npy', joint_vel)

    t_x = robot_output['t_x'][start_index:end_index]
    np.save('t_x', t_x)

    # Save FSM info
    np.save('osc_fsm.npy', np.vstack((osc_debug['fsm'], osc_debug['t_osc'])))

    ########################################################################
    
    tracking_data = osc_debug['osc_debug_tracking_datas']['swing_ft_traj']
    osc_time = tracking_data.t
    start_time = osc_time[0]
    target_start_time = start_time + 2.5
    valid_indices = ~np.isnan(osc_time)
    start_index = np.argmin(np.abs(osc_time[valid_indices] - target_start_time))
    end_index = np.argmin(np.abs(osc_time[valid_indices] - end_time))

    data_x = {}
    data_y = {}
    data_z = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_x['y'] = tracking_data.y[start_index:end_index, dim]
    data_x['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_x['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_x['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_x['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
    print(data_x['ydot_des'].shape)
    print(data_x['ydot'].shape)
    print(data_x['error_ydot'].shape)
    # data_x['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_x['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_x['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_x['t'] = tracking_data.t[start_index:end_index]
    #########################
    dim = 1
    data_y['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_y['y'] = tracking_data.y[start_index:end_index, dim]
    data_y['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_y['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_y['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_y['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_y['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_y['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_y['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_y['t'] = tracking_data.t[start_index:end_index]
    ###########################
    dim = 2
    data_z['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_z['y'] = tracking_data.y[start_index:end_index, dim]
    data_z['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_z['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t[start_index:end_index]
    
    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('swing_traj_x.npy', data_x_array)
    data_y_array = np.column_stack(list(data_y.values()))
    np.save('swing_traj_y.npy', data_y_array)
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('swing_traj_z.npy', data_z_array)

    #########################################################################################################
    
    tracking_data = osc_debug['osc_debug_tracking_datas']['alip_com_traj']
    osc_time = tracking_data.t
    start_time = osc_time[0]
    target_start_time = start_time + 2.5
    valid_indices = ~np.isnan(osc_time)
    start_index = np.argmin(np.abs(osc_time[valid_indices] - target_start_time))
    end_index = np.argmin(np.abs(osc_time[valid_indices] - end_time))

    data_z = {}

    dim = 2
    data_z['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_z['y'] = tracking_data.y[start_index:end_index, dim]
    data_z['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_z['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t[start_index:end_index]
    
    # Save data
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('alip_com_traj_z.npy', data_z_array)

    #########################################################################################################

    tracking_data = osc_debug['osc_debug_tracking_datas']['swing_hip_yaw_traj']
    osc_time = tracking_data.t
    start_time = osc_time[0]
    target_start_time = start_time + 2.5
    valid_indices = ~np.isnan(osc_time)
    start_index = np.argmin(np.abs(osc_time[valid_indices] - target_start_time))
    end_index = np.argmin(np.abs(osc_time[valid_indices] - end_time))

    data_x = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_x['y'] = tracking_data.y[start_index:end_index, dim]
    data_x['error_y'] = tracking_data.error_y[start_index:end_index, dim]
    data_x['t'] = tracking_data.t[start_index:end_index]

    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('swing_hip_yaw_traj_x.npy', data_x_array)

    #########################################################################################################

    tracking_data = osc_debug['osc_debug_tracking_datas']['pelvis_balance_traj']
    osc_time = tracking_data.t
    start_time = osc_time[0]
    target_start_time = start_time + 2.5
    valid_indices = ~np.isnan(osc_time)
    start_index = np.argmin(np.abs(osc_time[valid_indices] - target_start_time))
    end_index = np.argmin(np.abs(osc_time[valid_indices] - end_time))

    data_x = {}
    data_y = {}
    data_z = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_x['y'] = tracking_data.y[start_index:end_index, dim]
    data_x['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_x['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_x['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_x['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_x['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_x['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_x['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_x['t'] = tracking_data.t[start_index:end_index]
    #########################
    dim = 1
    data_y['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_y['y'] = tracking_data.y[start_index:end_index, dim]
    data_y['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_y['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_y['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_y['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_y['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_y['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_y['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_y['t'] = tracking_data.t[start_index:end_index]
    ###########################
    dim = 2
    data_z['y_des'] = tracking_data.y_des[start_index:end_index, dim]
    data_z['y'] = tracking_data.y[start_index:end_index, dim]
    data_z['error_y'] = tracking_data.error_y[start_index:end_index, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[start_index:end_index, dim]
    data_z['ydot'] = tracking_data.ydot[start_index:end_index, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[start_index:end_index, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t[start_index:end_index]
    
    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('pelvis_balance_traj_x.npy', data_x_array)
    data_y_array = np.column_stack(list(data_y.values()))
    np.save('pelvis_balance_traj_y.npy', data_y_array)
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('pelvis_balance_traj_z.npy', data_z_array)

def save_sim_main(plot_config, log):

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

    # print(act_names)
    # print(pos_names)

    default_channels = cassie_plots.cassie_default_channels
    print(default_channels)
    robot_output, robot_input, osc_debug, imu_accel = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, controller_plant, channel_x, channel_u, channel_osc)
    
    if plot_config.plot_battery_voltage:
        cassie_output = get_log_data(
            log,
            {plot_config.channel_cassie_out: dairlib.lcmt_cassie_out},
            plot_config.start_time,
            plot_config.duration,
            cassie_plots.load_cassie_out,
            plot_config.channel_cassie_out
        )

    print('Finished processing log')
    # t_x_slice = slice(robot_output['t_x'].size)
    # t_osc_slice = slice(osc_debug['t_osc'].size)
    print('Log start time: ', robot_output['t_x'][0])
    actuator_index = 0
    actuator_data = robot_output['u'][:, actuator_index]

    for i in range(len(actuator_data)):
        if np.all(actuator_data[i:] == 0):
            end_index = i
            break
        else:
            end_index = len(actuator_data) - 1
    print(len(actuator_data))
    print(end_index)

    # Save commanded_efforts
    commanded_efforts = robot_input['u']
    np.save('commanded_efforts.npy', commanded_efforts)

    # Save measured_efforts
    measured_efforts = robot_output['u']
    np.save('measured_efforts.npy', measured_efforts)

    # Save joint_pos
    joint_pos = robot_output['q']
    np.save('joint_pos.npy', joint_pos)

    # Save joint_vel
    joint_vel = robot_output['v']
    np.save('joint_vel.npy', joint_vel)

    t_x = robot_output['t_x']
    np.save('t_x', t_x)
    
    # Save FSM info
    np.save('osc_fsm.npy', np.vstack((osc_debug['fsm'], osc_debug['t_osc'])))
    ########################################################################
    
    tracking_data = osc_debug['osc_debug_tracking_datas']['swing_ft_traj']

    data_x = {}
    data_y = {}
    data_z = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[:, dim]
    data_x['y'] = tracking_data.y[:, dim]
    data_x['error_y'] = tracking_data.error_y[:, dim]
   
    data_x['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_x['ydot'] = tracking_data.ydot[:, dim]
    data_x['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_x['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_x['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_x['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_x['t'] = tracking_data.t
    #########################
    dim = 1
    data_y['y_des'] = tracking_data.y_des[:, dim]
    data_y['y'] = tracking_data.y[:, dim]
    data_y['error_y'] = tracking_data.error_y[:, dim]
   
    data_y['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_y['ydot'] = tracking_data.ydot[:, dim]
    data_y['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_y['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_y['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_y['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_y['t'] = tracking_data.t
    ###########################
    dim = 2
    data_z['y_des'] = tracking_data.y_des[:, dim]
    data_z['y'] = tracking_data.y[:, dim]
    data_z['error_y'] = tracking_data.error_y[:, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_z['ydot'] = tracking_data.ydot[:, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t
    
    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('swing_traj_x.npy', data_x_array)
    data_y_array = np.column_stack(list(data_y.values()))
    np.save('swing_traj_y.npy', data_y_array)
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('swing_traj_z.npy', data_z_array)

    #########################################################################################################

    tracking_data = osc_debug['osc_debug_tracking_datas']['alip_com_traj']

    data_z = {}

    dim = 2
    data_z['y_des'] = tracking_data.y_des[:, dim]
    data_z['y'] = tracking_data.y[:, dim]
    data_z['error_y'] = tracking_data.error_y[:, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_z['ydot'] = tracking_data.ydot[:, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t
    
    # Save data
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('alip_com_traj_z.npy', data_z_array)

    #########################################################################################################

    tracking_data = osc_debug['osc_debug_tracking_datas']['swing_hip_yaw_traj']

    data_x = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[:, dim]
    data_x['y'] = tracking_data.y[:, dim]
    data_x['error_y'] = tracking_data.error_y[:, dim]
    data_x['t'] = tracking_data.t

    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('swing_hip_yaw_traj_x.npy', data_x_array)

    #########################################################################################################

    tracking_data = osc_debug['osc_debug_tracking_datas']['pelvis_balance_traj']

    data_x = {}
    data_y = {}
    data_z = {}

    dim = 0
    data_x['y_des'] = tracking_data.y_des[:, dim]
    data_x['y'] = tracking_data.y[:, dim]
    data_x['error_y'] = tracking_data.error_y[:, dim]
   
    data_x['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_x['ydot'] = tracking_data.ydot[:, dim]
    data_x['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_x['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_x['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_x['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_x['t'] = tracking_data.t
    #########################
    dim = 1
    data_y['y_des'] = tracking_data.y_des[:, dim]
    data_y['y'] = tracking_data.y[:, dim]
    data_y['error_y'] = tracking_data.error_y[:, dim]
   
    data_y['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_y['ydot'] = tracking_data.ydot[:, dim]
    data_y['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_y['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_y['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_y['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_y['t'] = tracking_data.t
    ###########################
    dim = 2
    data_z['y_des'] = tracking_data.y_des[:, dim]
    data_z['y'] = tracking_data.y[:, dim]
    data_z['error_y'] = tracking_data.error_y[:, dim]
   
    data_z['ydot_des'] = tracking_data.ydot_des[:, dim]
    data_z['ydot'] = tracking_data.ydot[:, dim]
    data_z['error_ydot'] = tracking_data.error_ydot[:, dim]
   
    # data_z['yddot_des'] = tracking_data.yddot_des[start_index:end_index, dim]
    # data_z['yddot_command'] = tracking_data.yddot_command[start_index:end_index, dim]
    # data_z['yddot_command_sol'] = tracking_data.yddot_command_sol[start_index:end_index, dim]
    data_z['t'] = tracking_data.t
    
    # Save data
    data_x_array = np.column_stack(list(data_x.values()))
    np.save('pelvis_balance_traj_x.npy', data_x_array)
    data_y_array = np.column_stack(list(data_y.values()))
    np.save('pelvis_balance_traj_y.npy', data_y_array)
    data_z_array = np.column_stack(list(data_z.values()))
    np.save('pelvis_balance_traj_z.npy', data_z_array)

def main():
    config_folder = 'bindings/pydairlib/analysis/plot_configs/'
    config_file = config_folder + 'cassie_default_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")

    # plotter_main(plot_config, log)
    # plt.show()

    # save_main(plot_config, log)

    save_sim_main(plot_config, log)


if __name__ == '__main__':
    main()