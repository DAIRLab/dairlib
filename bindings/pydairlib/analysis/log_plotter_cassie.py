import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

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
    model_instance = plant.GetModelInstanceByName("cassie")
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
        # See `process_contact_channel` in mbp_plotting_utils.py for details
        # As of 20230217:
        #   Both force and position are ewrt world frame.
        #   The order is [left_back , left_front, right_back, right_front], under the assumption that the front contact's x position is larger than the rear's!
        #   The reason is that we compare the world position of the contact point to determine which contact is front. This issue can be fixed if we transform it into body frame.
        PlotContactForce(contact_output['t_lambda'], contact_output['lambda_c'], contact_output['p_lambda_c'], osc_debug, plot_config)
        ComputeAndPlotContactMoment(contact_output['t_lambda'], contact_output['lambda_c'], contact_output['p_lambda_c'], osc_debug, plot_config)


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

    # fig = PlotCommandedAndActualTorques(robot_output, robot_input, osc_debug)
    # plot = plot_styler.PlotStyler(fig)
    # mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    #


    # ComputeAndPlotCentroidalAngularMomentum(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context,osc_debug,plot_config)

    selected_bodies = ["pelvis","yaw_left","yaw_right","hip_left","hip_right"]
    angular_momentum_about_local_com_of_selected_bodies, _ = ComputeAndPlotAngularMomentumAboutTheSelectedBodiesCoM(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context,osc_debug,plot_config, model_instance, selected_bodies)
    dictionary_centroidal_angular_momentum_per_body, dictionary_centroidal_angular_momentum_per_body_from_rotation = ComputeAndPlotCentroidalAngularMomentumOfEachBody(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context,osc_debug,plot_config, model_instance)
    ComputeAndPlotCentroidalAngularMomentumForPaper(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context,osc_debug,plot_config, dictionary_centroidal_angular_momentum_per_body, dictionary_centroidal_angular_momentum_per_body_from_rotation, angular_momentum_about_local_com_of_selected_bodies)
    # 0.72-(-0.36)
    # 0.94-(-0.53)


    ComputeAndPlotACoM(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug, osc_debug['t_osc'], plot_config)
    ComputeAndPlotACoMRate(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug, osc_debug['t_osc'], plant, plot_config, True)
    #
    # PlotTorqueSquare(robot_output, robot_input, osc_debug, plot_config)


    # InvestigatePelvisMotionRtCoM(np.hstack([robot_output['q'], robot_output['v']]), robot_output['t_x'], osc_debug['t_osc'], osc_debug['fsm'], plant, context,osc_debug,plot_config, model_instance)


    plt.show()


# def PlotCommandedAndActualTorques(robot_output, robot_input, osc_debug):
#     fig = plt.figure("torques")
#     plt.plot(robot_output['t_x'], robot_output['u'])
#     plt.gca().set_prop_cycle(None)
#     plt.plot(robot_input['t_u'], robot_input['u'])
#     return fig



def PlotTorqueSquare(robot_output, robot_input, osc_debug, plot_config):
    fig = plt.figure("torques")
    torque_square = np.zeros(robot_input['t_u'].size)
    for i in range(torque_square.size):
        torque_square[i] = np.inner(robot_input['u'][i], robot_input['u'][i])
    plt.plot(robot_input['t_u'], torque_square)
    # plt.plot(robot_input['t_u'], robot_input['u'])
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)


# cutoff_freq is in Hz
def ApplyLowPassFilter(x, t, cutoff_freq, start_from_zero=False):
    x = np.copy(x)
    if start_from_zero:
        x[0] = np.zeros(x[0].size)

    dt = np.diff(t)
    x_filtered = x[0]
    for i in range(len(dt)):
        alpha = 2 * np.pi * dt[i] * cutoff_freq / (
            2 * np.pi * dt[i] * cutoff_freq + 1)
        x_filtered = alpha * x[i + 1] + (1 - alpha) * x_filtered
        x[i + 1] = x_filtered
    return x

def PlotContactForce(t, forces, positions, osc_debug, plot_config):
    fig = plt.figure("Contact forces")
    plt.title("Contact forces")
    plt.xlabel("time")
    plt.ylabel("Contact forces (N)")
    plt.plot(t, forces[0,:,:])
    plt.plot(t, forces[1,:,:])
    plt.gca().set_prop_cycle(None)
    plt.plot(t, forces[2,:,:], "--")
    plt.plot(t, forces[3,:,:], "--")
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["left rear x", "left rear y", "left rear z", "left front x", "left front y", "left front z", "right rear x", "right rear y", "right rear z", "right front x", "right front y", "right front z"])

def ComputeAndPlotContactMoment(t, forces, positions, osc_debug, plot_config):
    moments = np.zeros((2, t.size, 3))

    # left foot
    for i in range(t.size):
        cop = (positions[0,i,:] * forces[0,i,2] + positions[1,i,:] * forces[1,i,2]) / (forces[0,i,2] + forces[1,i,2])
        moments[0,i,:] = np.cross(positions[0,i,:] - cop, forces[0,i,:]) + np.cross(positions[1,i,:] - cop, forces[1,i,:])
    # right foot
    for i in range(t.size):
        cop = (positions[2,i,:] * forces[2,i,2] + positions[3,i,:] * forces[3,i,2]) / (forces[2,i,2] + forces[3,i,2])
        moments[1,i,:] = np.cross(positions[2,i,:] - cop, forces[2,i,:]) + np.cross(positions[3,i,:] - cop, forces[3,i,:])

    fig = plt.figure("Contact moment")
    plt.title("Contact moment")
    plt.xlabel("time")
    plt.ylabel("Contact moment (Nm)")
    plt.plot(t, moments[0,:,:])
    plt.gca().set_prop_cycle(None)
    plt.plot(t, moments[1,:,:], "--")
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["left x", "left y", "left z", "right x", "right y", "right z"])



def ComputeAndPlotACoMRate(x, t_x, osc_debug, t_osc_debug, plant, plot_config, express_xy_in_local=False):
    pelvis_quaternion = osc_debug['osc_debug_tracking_datas']['pelvis_rot_traj'].y

    ### ACoM rate (angle axis)
    acom_omega = np.zeros((t_osc_debug.size, 3))
    acom_omega = osc_debug['osc_debug_tracking_datas']['acom_traj'].ydot

    ### pelvis rate (angle axis)
    #nq = plant.num_positions()
    #pelvis_omega = x[:,nq:nq+3]
    pelvis_omega = osc_debug['osc_debug_tracking_datas']['pelvis_rot_traj'].ydot

    ### Express the x y component in pelvis local frame
    if express_xy_in_local:
        for i in range(t_osc_debug.size):
            y = pelvis_quaternion[i]
            r = R.from_quat([y[1], y[2], y[3], y[0]])

            acom_omega[i,0:2] = (r.as_matrix().T @ acom_omega[i])[0:2]
            pelvis_omega[i,0:2] = (r.as_matrix().T @ pelvis_omega[i])[0:2]

    ### Average yaw velocity
    filterred_acom_yaw_rate = ApplyLowPassFilter(acom_omega[:,2], t_osc_debug, 0.1, True)
    filterred_pelvis_yaw_rate = ApplyLowPassFilter(pelvis_omega[:,2], t_osc_debug, 0.1, True)

    ### Plot rate
    fig = plt.figure("Angular Velocity")
    plt.title("Angular Velocity (ACoM vs floating base)")
    plt.xlabel("time")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.plot(t_osc_debug, acom_omega[:,[0,1,2]], "--")
    plt.gca().set_prop_cycle(None)
    plt.plot(t_osc_debug, pelvis_omega[:,[0,1,2]])
    plt.plot(t_osc_debug, filterred_acom_yaw_rate, "k--")
    plt.plot(t_osc_debug, filterred_pelvis_yaw_rate, "k")
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["ACoM x", "ACoM y", "ACoM z", "pelvis x", "pelvis y", "pelvis z", "ave yaw rate"])



def ComputeAndPlotACoM(x, t_x, osc_debug, t_osc_debug, plot_config):
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
    plt.title("ACoM vs floating base (RPY)")
    plt.xlabel("time")
    plt.ylabel("Angles (rad)")
    plt.plot(t_osc_debug, acom_rpy[:,[0,1,2]], "--")
    plt.gca().set_prop_cycle(None)
    plt.plot(t_x, pelvis_rpy[:,[0,1,2]])
    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["ACoM roll", "ACoM pitch", "ACoM yaw", "pelvis roll", "pelvis pitch", "pelvis yaw"])



def ComputeAndPlotCentroidalAngularMomentumForPaper(x, t_x, t_osc_debug, fsm, plant, context,osc_debug,plot_config,
        dictionary_centroidal_angular_momentum_per_body, dictionary_centroidal_angular_momentum_per_body_from_rotation, angular_momentum_about_local_com_of_selected_bodies):
    ### Total centroidal angular momentum
    centroidal_angular_momentum = np.zeros((t_x.size, 3))
    for i in range(t_x.size):
        # import pdb;pdb.set_trace()
        plant.SetPositionsAndVelocities(context, x[i])
        com = plant.CalcCenterOfMassPositionInWorld(context)

        h_WC_eval = plant.CalcSpatialMomentumInWorldAboutPoint(context, com)
        centroidal_angular_momentum[i] = h_WC_eval.rotational()

    # Testing -- testing if the CAM per body calculation is correct (Ans: it is)
    # cam_sum = np.zeros((t_x.size,3))
    # for key in dictionary_centroidal_angular_momentum_per_body:
    #     cam_sum = cam_sum + dictionary_centroidal_angular_momentum_per_body[key]

    # Testing -- compute the CAM of the whole pelvis body including the yaw and hip
    whole_pelvis_body_dict = ["pelvis","yaw_left","yaw_right","hip_left","hip_right"]
    centroidal_angular_momentum_whole_pelvis_body = np.zeros((t_x.size, 3))
    for key in whole_pelvis_body_dict:
        centroidal_angular_momentum_whole_pelvis_body = centroidal_angular_momentum_whole_pelvis_body + dictionary_centroidal_angular_momentum_per_body[key]

    fig = plt.figure("Centroidal angular momentum about z axis")
    plt.title("Centroidal angular momentum about z axis")
    plt.xlabel("time (s)")
    plt.ylabel("($kg \cdot m^2 / s$)")
    plt.xlim([6.24,7.44])
    plt.ylim([-0.64,1.03])
    plt.plot(t_x, centroidal_angular_momentum[:,[2]])
    # plt.plot(t_x, dictionary_centroidal_angular_momentum_per_body["pelvis"][:,[2]])
    # plt.plot(t_x, dictionary_centroidal_angular_momentum_per_body_from_rotation["pelvis"][:,[2]])
    plt.plot(t_x, angular_momentum_about_local_com_of_selected_bodies[:,[2]])
    # plt.plot(t_x, centroidal_angular_momentum_whole_pelvis_body[:,[0]])
    # plt.plot(t_x, centroidal_angular_momentum_whole_pelvis_body[:,[1]])
    # plt.plot(t_x, centroidal_angular_momentum_whole_pelvis_body[:,[2]])
    # plt.plot(t_x, cam_sum[:,[2]])
    # plt.plot(t_osc_debug, 0.1 * fsm)
    # plt.legend(["x", "y", "z", "fsm"])

    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)

    # plt.legend(["total", "pelvis (rt com)", "pelvis"])
    # plt.legend(["total", "x", "y", "z"])
    # plt.legend(["total", "whole pelvis (rt com)"])
    plt.legend(["total", "whole pelvis"])
    # plt.legend(["total", "pelvis"])


def ComputeAndPlotCentroidalAngularMomentum(x, t_x, t_osc_debug, fsm, plant, context,osc_debug,plot_config):
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

    plot = plot_styler.PlotStyler(fig)
    mbp_plots.add_fsm_to_plot(plot, osc_debug['t_osc'], osc_debug['fsm'], plot_config.fsm_state_names)
    plt.legend(["x", "y", "z"])


def ComputeAndPlotCentroidalAngularMomentumOfEachBody(x, t_x, t_osc_debug, fsm, plant, context,osc_debug,plot_config, model_instance):
    ## Individual momentum (reference: CalcSpatialMomentumInWorldAboutPoint)
    body_indices = plant.GetBodyIndices(model_instance)
    dictionary_centroidal_angular_momentum_per_body = {}
    dictionary_centroidal_angular_momentum_per_body_from_rotation = {}
    for body_idx in body_indices:
        # No contribution from the world body.
        if body_idx == 0:
            continue
        # Ensure MultibodyPlant method contains a valid body_index.
        if int(body_idx) >= plant.num_bodies():
            raise ValueError("wrong index. Bug somewhere")

        body = plant.get_body(body_idx)
        print(body.name())

        angular_momentum_per_body = np.zeros((t_x.size, 3))
        angular_momentum_per_body_from_rotation = np.zeros((t_x.size, 3))
        for i in range(t_x.size):
            plant.SetPositionsAndVelocities(context, x[i])
            com = plant.CalcCenterOfMassPositionInWorld(context)

            body_pose = plant.EvalBodyPoseInWorld(context, body)

            R_AE = body_pose.rotation()
            M_BBo_W = body.default_spatial_inertia().ReExpress(R_AE)
            V_WBo_W = plant.EvalBodySpatialVelocityInWorld(context, body)
            L_WBo_W = M_BBo_W * V_WBo_W

            # Testing
            p_BoBc_W = R_AE.matrix() @ body.default_com()
            L_BcBc_W = L_WBo_W.Shift(p_BoBc_W)

            # if t_x[i] > 6.7:
            #     import pdb;pdb.set_trace()

            # SpatialMomentumInWorldAboutWo
            p_WoBo_W = body_pose.translation()
            L_WS_W = L_WBo_W.Shift(-p_WoBo_W)

            # SpatialMomentumInWorldAboutCOM
            L_WS_W = L_WS_W.Shift(com)

            angular_momentum_per_body[i] = L_WS_W.rotational()
            # angular_momentum_per_body_from_rotation[i] = L_WBo_W.rotational()
            angular_momentum_per_body_from_rotation[i] = L_BcBc_W.rotational()
            # import pdb;pdb.set_trace()
        dictionary_centroidal_angular_momentum_per_body[body.name()] = angular_momentum_per_body
        dictionary_centroidal_angular_momentum_per_body_from_rotation[body.name()] = angular_momentum_per_body_from_rotation

    dim = 2   # 0 corresponds to x
    plt.figure("Centroidal angular momentum per body")
    plt.plot(t_osc_debug, 0.1 * fsm, 'k')
    legend_list = ["fsm"]
    i = 0
    linestyle = '-'
    for key in dictionary_centroidal_angular_momentum_per_body:
        plt.plot(t_x, dictionary_centroidal_angular_momentum_per_body[key][:,dim], linestyle)
        legend_list += [key]
        if i == 9:
            linestyle = '--'
        if i == 19:
            linestyle = '-.'
        i+=1
    plt.legend(legend_list)

    return dictionary_centroidal_angular_momentum_per_body, dictionary_centroidal_angular_momentum_per_body_from_rotation


def ComputeAndPlotAngularMomentumAboutTheSelectedBodiesCoM(x, t_x, t_osc_debug, fsm, plant, context,osc_debug,plot_config, model_instance, body_names):
    ### Get CoMs
    CoMs_per_body = {}
    mass_per_body = {}
    for name in body_names:
        body = plant.GetBodyByName(name)

        coms_per_body = np.zeros((t_x.size, 3))
        for i in range(t_x.size):
            plant.SetPositionsAndVelocities(context, x[i])

            body_pose = plant.EvalBodyPoseInWorld(context, body)
            R_AE = body_pose.rotation()
            p_WoBo_W = body_pose.translation()
            p_BoBc_W = R_AE.matrix() @ body.default_com()

            coms_per_body[i] = p_WoBo_W + p_BoBc_W

        CoMs_per_body[body.name()] = coms_per_body
        mass_per_body[body.name()] = body.default_mass()

    M = 0
    for name in body_names:
        body = plant.GetBodyByName(name)
        M += mass_per_body[body.name()]
    CoMs = np.zeros((t_x.size, 3))
    for name in body_names:
        body = plant.GetBodyByName(name)
        CoMs += mass_per_body[body.name()] * CoMs_per_body[body.name()]
    CoMs /= M

    ### Get AM
    angular_momentum_about_local_com_of_selected_bodies = np.zeros((t_x.size, 3))
    angular_momentum_about_local_com_of_selected_bodies_per_body = {}
    for name in body_names:
        body = plant.GetBodyByName(name)

        angular_momentum_about_local_com_per_body = np.zeros((t_x.size, 3))
        for i in range(t_x.size):
            plant.SetPositionsAndVelocities(context, x[i])

            body_pose = plant.EvalBodyPoseInWorld(context, body)

            R_AE = body_pose.rotation()
            M_BBo_W = body.default_spatial_inertia().ReExpress(R_AE)
            V_WBo_W = plant.EvalBodySpatialVelocityInWorld(context, body)
            L_WBo_W = M_BBo_W * V_WBo_W

            # Testing
            # p_BoBc_W = R_AE.matrix() @ body.default_com()
            # L_BcBc_W = L_WBo_W.Shift(p_BoBc_W)

            # if t_x[i] > 6.7:
            #     import pdb;pdb.set_trace()

            # SpatialMomentumInWorldAboutWo
            p_WoBo_W = body_pose.translation()
            L_WS_W = L_WBo_W.Shift(-p_WoBo_W)

            # SpatialMomentumInWorldAboutCOM
            L_WS_W = L_WS_W.Shift(CoMs[i])

            angular_momentum_about_local_com_per_body[i] = L_WS_W.rotational()
            # import pdb;pdb.set_trace()
        angular_momentum_about_local_com_of_selected_bodies += angular_momentum_about_local_com_per_body
        angular_momentum_about_local_com_of_selected_bodies_per_body[body.name()] = angular_momentum_about_local_com_per_body

    dim = 2   # 0 corresponds to x
    plt.figure("angular momentum per body about the COM of selected bodies")
    plt.plot(t_osc_debug, 0.1 * fsm, 'k')
    legend_list = ["fsm"]
    i = 0
    linestyle = '-'
    for key in angular_momentum_about_local_com_of_selected_bodies_per_body:
        plt.plot(t_x, angular_momentum_about_local_com_of_selected_bodies_per_body[key][:,dim], linestyle)
        legend_list += [key]
        if i == 9:
            linestyle = '--'
        if i == 19:
            linestyle = '-.'
        i+=1
    plt.plot(t_x, angular_momentum_about_local_com_of_selected_bodies[:,dim], linestyle)
    legend_list += ["all"]
    plt.legend(legend_list)

    return angular_momentum_about_local_com_of_selected_bodies, angular_momentum_about_local_com_of_selected_bodies_per_body




def InvestigatePelvisMotionRtCoM(x, t_x, t_osc_debug, fsm, plant, context,osc_debug,plot_config, model_instance):
    ## Individual momentum (reference: CalcSpatialMomentumInWorldAboutPoint)
    body_indices = plant.GetBodyIndices(model_instance)

    body = plant.GetBodyByName("pelvis")
    angular_momentum_per_body = np.zeros((t_x.size, 3))
    angular_momentum_per_body_from_rotation = np.zeros((t_x.size, 3))
    coms = np.zeros((t_x.size, 3))
    pelvis_rotations = np.zeros((t_x.size, 3, 3))
    pelvis_positions = np.zeros((t_x.size, 3))
    com_pos_max = -100000 * np.ones([2,1])
    com_pos_min = 100000 * np.ones([2,1])
    for i in range(t_x.size):
        plant.SetPositionsAndVelocities(context, x[i])
        com = plant.CalcCenterOfMassPositionInWorld(context)

        body_pose = plant.EvalBodyPoseInWorld(context, body)

        R_AE = body_pose.rotation()
        M_BBo_W = body.default_spatial_inertia().ReExpress(R_AE)
        V_WBo_W = plant.EvalBodySpatialVelocityInWorld(context, body)
        L_WBo_W = M_BBo_W * V_WBo_W

        # SpatialMomentumInWorldAboutWo
        p_WoBo_W = body_pose.translation()
        L_WS_W = L_WBo_W.Shift(-p_WoBo_W)

        # SpatialMomentumInWorldAboutCOM
        L_WS_W = L_WS_W.Shift(com)

        angular_momentum_per_body[i] = L_WS_W.rotational()
        angular_momentum_per_body_from_rotation[i] = L_WBo_W.rotational()

        coms[i] = com
        pelvis_rotations[i] = R_AE.matrix()
        pelvis_positions[i] = p_WoBo_W

        com_pos_max = np.max(np.hstack([com[0:2].reshape(2,1),com_pos_max]), 1).reshape(2,1)
        com_pos_min = np.min(np.hstack([com[0:2].reshape(2,1),com_pos_min]), 1).reshape(2,1)
    x_limit = [com_pos_min[0] - 1, com_pos_max[0]+1]
    y_limit = [com_pos_min[1] - 1, com_pos_max[1]+1]

    # Animate
    first_time = True
    for i in range(0, t_x.size, 10):
        if first_time:
            plt.figure("Feet and CoM positions", figsize=(6.4, 4.8))
            first_time = False
        plt.title("current time = %.3f" % t_x[i])

        plt.plot(coms[i,0],coms[i,1], 'ko')

        # Draw pelvis orientation
        rot = pelvis_rotations[i]
        # rot = pelvis_rotations[i].T
        pelvis_x_axis = rot[0:2,0]
        pelvis_y_axis = rot[0:2,1]
        # import pdb;pdb.set_trace()
        plt.arrow(x=pelvis_positions[i,0], y=pelvis_positions[i,1], dx=pelvis_x_axis[0]/5, dy=pelvis_x_axis[1]/5, width=.01)
        plt.arrow(x=pelvis_positions[i,0], y=pelvis_positions[i,1], dx=pelvis_y_axis[0]/5, dy=pelvis_y_axis[1]/5, width=.01)

        # Labels and stuffs
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        if len(x_limit) > 0:
            plt.xlim(x_limit)
            plt.ylim(y_limit)
        # else:
        #     lb = np.min(np.hstack([global_feet_pos, global_com_pos]), 1)
        #     ub = np.min(np.hstack([global_feet_pos, global_com_pos]), 1)
        #     plt.xlim([lb[0] - 1, lb[1] + 1])
        #     plt.ylim([ub[0] - 1, ub[1] + 1])
        # Manually overwrite
        # plt.xlim([-0.2, 3])
        # plt.ylim([-1, 1])

        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')


        plt.draw()
        plt.pause(0.01)
        # plt.pause(0.1)
        # plt.pause(0.3)
        plt.clf()


if __name__ == '__main__':
    main()
