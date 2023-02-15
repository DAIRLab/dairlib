import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

from bindings.pydairlib.lcm.process_lcm_log import get_log_data
from cassie_plot_config import CassiePlotConfig
import cassie_plotting_utils as cassie_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from bindings.pydairlib.common import plot_styler

def main():
    config_file = 'bindings/pydairlib/analysis/plot_configs/cassie_running_plot.yaml'
    # config_file = 'bindings/pydairlib/analysis/plot_configs/cassie_kcmpc_plot.yaml'
    # config_file = 'bindings/pydairlib/analysis/plot_configs/cassie_standing_plot.yaml'
    # config_file = 'bindings/pydairlib/analysis/plot_configs/cassie_default_plot.yaml'
    # config_file = 'bindings/pydairlib/analysis/plot_configs/cassie_jumping_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    use_floating_base = plot_config.use_floating_base
    use_springs = plot_config.use_springs

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)
    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    default_channels = cassie_plots.cassie_default_channels
    if plot_config.use_archived_lcmtypes:
        default_channels = cassie_plots.cassie_default_channels_archive
    robot_output, robot_input, osc_debug = \
        get_log_data(log,  # log
                     default_channels,  # lcm channels
                     plot_config.start_time,
                     plot_config.duration,
                     mbp_plots.load_default_channels,  # processing callback
                     plant, channel_x, channel_u, channel_osc)  # processing callback arguments

    if plot_config.plot_contact_forces:
        contact_output = get_log_data(log,  # log
                                      cassie_plots.cassie_contact_channels,  # lcm channels
                                      plot_config.start_time,
                                      plot_config.duration,
                                      mbp_plots.load_force_channels,  # processing callback
                                      'CASSIE_CONTACT_DRAKE')  # processing callback arguments

    print('Finished processing log - making plots')
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)

    print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))

    ''' Plot Positions '''
    # Plot floating base positions if applicable
    if use_floating_base and plot_config.plot_floating_base_positions:
        plot = mbp_plots.plot_floating_base_positions(
            robot_output, pos_names, 7, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


    # Plot joint positions
    if plot_config.plot_joint_positions:
        plot = mbp_plots.plot_joint_positions(robot_output, pos_names,
                                       7 if use_floating_base else 0, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(robot_output, plot_config.pos_names,
                                         t_x_slice, pos_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


    ''' Plot Velocities '''
    # Plot floating base velocities if applicable
    if use_floating_base and plot_config.plot_floating_base_velocities:
        plot = mbp_plots.plot_floating_base_velocities(
            robot_output, vel_names, 6, t_x_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_floating_base_velocity_body_frame:
        plot = mbp_plots.plot_floating_base_body_frame_velocities(
            robot_output, t_x_slice, plant, context, "pelvis")
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # Plot all joint velocities
    if plot_config.plot_joint_velocities:
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names,
                                               6 if use_floating_base else 0,
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
        mbp_plots.plot_measured_efforts_by_name(robot_output,
                                                plot_config.act_names,
                                                t_x_slice, act_map)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


    ''' Plot OSC '''
    if plot_config.plot_tracking_costs:
        plot = mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
        # plt.ylim([0, 1e4])
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
                    plot.save_fig(traj_name + '_' + deriv + '_' + str(dim)  + '.png')

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

        plot = mbp_plots.plot_points_positions(robot_output, t_x_slice, plant, context,
                                               foot_frames, pts, dims)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    if plot_config.plot_qp_solve_time:
        plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)
        mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # if plot_config.plot_active_tracking_datas:
    #     plot = mbp_plots.plot_active_tracking_datas(osc_debug, t_osc_slice, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    #     plot.save_fig('active_tracking_datas.png')

    # import pdb;pdb.set_trace()

    fig = ComputeAndPlotCentroidalAngularMomentum(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context)
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["x", "y", "z"])

    # fig = PlotCommandedAndActualTorques(robot_output, robot_input, osc_debug)
    # plot = plot_styler.PlotStyler(fig)
    # mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    #

    ComputeAndPlotACoM(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug, osc_debug['t_osc'], plot_config)
    ComputeAndPlotACoMRate(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug, osc_debug['t_osc'], plant, plot_config)

    plt.show()


# def PlotCommandedAndActualTorques(robot_output, robot_input, osc_debug):
#     fig = plt.figure("torques")
#     plt.plot(robot_output['t_x'], robot_output['u'])
#     plt.gca().set_prop_cycle(None)
#     plt.plot(robot_input['t_u'], robot_input['u'])
#     return fig

def ComputeAndPlotACoMRate(x, t_x, osc_debug, t_osc_debug, plant, plot_config):
    ### ACoM rate (angle axis)
    acom_omega = np.zeros((t_osc_debug.size, 3))
    acom_omega = osc_debug['osc_debug_tracking_datas']['acom_traj'].ydot

    ### pelvis rate (angle axis)
    nq = plant.num_positions()
    pelvis_omega = x[:,nq:nq+3]

    ### Plot rate
    fig = plt.figure("Angular Center of Mass Rate")
    plt.plot(t_osc_debug, acom_omega[:,[0,1,2]], "--")
    plt.gca().set_prop_cycle(None)
    plt.plot(t_x, pelvis_omega[:,[0,1,2]])
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["ACoM x", "ACoM y", "ACoM z", "pelvis x", "pelvis y", "pelvis z"])



def ComputeAndPlotACoM(x, t_x, osc_debug, t_osc_debug, plot_config):
    from scipy.spatial.transform import Rotation as R

    ### Compute ACoM RPY
    acom_quaternion = np.zeros((t_osc_debug.size, 4))
    acom_quaternion = osc_debug['osc_debug_tracking_datas']['acom_traj'].y
    acom_rpy = np.zeros((t_osc_debug.size, 3))
    for i in range(t_osc_debug.size):
        y = acom_quaternion[i]
        r = R.from_quat([y[1], y[2], y[3], y[0]])
        acom_rpy[i] = r.as_euler('xyz', degrees=False)

    ### Compute pelvis RPY
    pelvis_quaternion = x[:,0:4]
    pelvis_rpy = np.zeros((t_x.size, 3))
    for i in range(t_x.size):
        y = pelvis_quaternion[i]
        r = R.from_quat([y[1], y[2], y[3], y[0]])
        pelvis_rpy[i] = r.as_euler('xyz', degrees=False)

    ### Plot RPY
    fig = plt.figure("Angular Center of Mass")
    plt.plot(t_osc_debug, acom_rpy[:,[0,1,2]], "--")
    plt.gca().set_prop_cycle(None)
    plt.plot(t_x, pelvis_rpy[:,[0,1,2]])
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["ACoM x", "ACoM y", "ACoM z", "pelvis x", "pelvis y", "pelvis z"])



def ComputeAndPlotCentroidalAngularMomentum(x, t_x, t_osc_debug, fsm, plant, context):
    ### Total centroidal angular momentum
    centroidal_angular_momentum = np.zeros((t_x.size, 3))
    for i in range(t_x.size):
        # import pdb;pdb.set_trace()
        plant.SetPositionsAndVelocities(context, x[i])
        com = plant.CalcCenterOfMassPositionInWorld(context)

        h_WC_eval = plant.CalcSpatialMomentumInWorldAboutPoint(context, com)
        centroidal_angular_momentum[i] = h_WC_eval.rotational()

    fig = plt.figure("Centroidal angular momentum")
    plt.plot(t_x, centroidal_angular_momentum[:,[0,1,2]])
    # plt.plot(t_osc_debug, 0.1 * fsm)
    # plt.legend(["x", "y", "z", "fsm"])
    plt.legend(["x", "y", "z"])
    # plt.legend(["x", "z", "fsm"])

    return fig

    ### Individual momentum (reference: CalcSpatialMomentumInWorldAboutPoint)
    # body_indices = plant.GetBodyIndices(model_instance)
    # dictionary_centroidal_angular_momentum_per_body = {}
    # for body_idx in body_indices:
    #     # No contribution from the world body.
    #     if body_idx == 0:
    #         continue
    #     # Ensure MultibodyPlant method contains a valid body_index.
    #     if int(body_idx) >= plant.num_bodies():
    #         raise ValueError("wrong index. Bug somewhere")
    #
    #     body = plant.get_body(body_idx)
    #     print(body.name())
    #
    #     angular_momentum_per_body = np.zeros((t_x.size, 3))
    #     for i in range(t_x.size):
    #         plant.SetPositionsAndVelocities(context, x[i])
    #         com = plant.CalcCenterOfMassPositionInWorld(context)
    #
    #         body_pose = plant.EvalBodyPoseInWorld(context, body)
    #
    #         R_AE = body_pose.rotation()
    #         M_BBo_W = body.default_spatial_inertia().ReExpress(R_AE)
    #         V_WBo_W = plant.EvalBodySpatialVelocityInWorld(context, body)
    #         L_WBo_W = M_BBo_W * V_WBo_W
    #
    #         # SpatialMomentumInWorldAboutWo
    #         p_WoBo_W = body_pose.translation()
    #         L_WS_W = L_WBo_W.Shift(-p_WoBo_W)
    #
    #         # SpatialMomentumInWorldAboutCOM
    #         L_WS_W = L_WS_W.Shift(com)
    #
    #         angular_momentum_per_body[i] = L_WS_W.rotational()
    #     dictionary_centroidal_angular_momentum_per_body[body.name()] = angular_momentum_per_body
    #
    # dim = 0
    # plt.figure("Centroidal angular momentum per body")
    # plt.plot(t_osc_debug, 0.1 * fsm, 'k')
    # legend_list = ["fsm"]
    # i = 0
    # linestyle = '-'
    # for key in dictionary_centroidal_angular_momentum_per_body:
    #     plt.plot(t_x, dictionary_centroidal_angular_momentum_per_body[key][:,dim], linestyle)
    #     legend_list += [key]
    #     if i == 9:
    #         linestyle = '--'
    #     if i == 19:
    #         linestyle = '-.'
    #     i+=1
    # plt.legend(legend_list)

if __name__ == '__main__':
    main()
