import sys

import dairlib
import drake
import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
from scipy import integrate
import pydairlib.lcm_trajectory
import process_lcm_log
import pydairlib.multibody_utils
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial



def get_index_at_time(times, t):
    return np.argwhere(times - t > 0)[0][0]

def print_osc_debug(t_idx, length, osc_debug):
    print('y = ' + str(osc_debug.y[t_idx:t_idx + length, :]))
    print('y_des = ' + str(osc_debug.y_des[t_idx:t_idx + length, :]))
    print('error_y = ' + str(osc_debug.error_y[t_idx:t_idx + length, :]))
    print('dy = ' + str(osc_debug.dy[t_idx:t_idx + length, :]))
    print('dy_des = ' + str(osc_debug.dy_des[t_idx:t_idx + length, :]))
    print('error_dy = ' + str(osc_debug.error_dy[t_idx:t_idx + length, :]))
    print('ddy_des = ' + str(osc_debug.ddy_des[t_idx:t_idx + length, :]))
    print('ddy_command = ' + str(
        osc_debug.ddy_command[t_idx:t_idx + length, :]))
    print('ddy_command_sol = ' + str(
        osc_debug.ddy_command_sol[t_idx:t_idx + length, :]))


def calcNetImpulse(plant, context, t_contact_info, contact_info, t_state, q, v):
    n_dim = 3
    impact_duration = 0.05
    net_impulse = np.zeros((n_dim,1))

    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)

    # Get the index for when the first grf is non-zero
    t_start_idx = np.min(np.argwhere(contact_info > 0), 0)[1]
    # Get the index for impact_duration (s) after the first non-zero grf
    t_end_idx = get_index_at_time(t_contact_info, t_state[t_start_idx] + impact_duration)
    x = np.hstack((q, v))

    l_contact_frame = plant.GetBodyByName("toe_left").body_frame()
    r_contact_frame = plant.GetBodyByName("toe_right").body_frame()
    world = plant.world_frame()
    front_contact_disp = np.array((-0.0457, 0.112, 0))
    rear_contact_disp = np.array((0.088, 0, 0))

    t_slice = slice(t_start_idx, t_end_idx)
    impulses = np.zeros((contact_info.shape[0], n_dim))
    for i in range(contact_info.shape[0]):
        for j in range(n_dim):
            impulses[i, j] = np.trapz(contact_info[i, t_slice, j],
                                   t_contact_info[t_slice])

    impulse_from_contact = np.zeros((t_end_idx - t_start_idx, v.shape[1]))

    for i in range(t_start_idx, t_end_idx):
        plant.SetPositionsAndVelocities(context, x[i])
        J_l_r = plant.CalcJacobianTranslationalVelocity(context,
                    JacobianWrtVariable.kV, l_contact_frame, rear_contact_disp,
                    world, world)
        J_l_f = plant.CalcJacobianTranslationalVelocity(context,
                    JacobianWrtVariable.kV, l_contact_frame, front_contact_disp,
                    world, world)
        J_r_r = plant.CalcJacobianTranslationalVelocity(context,
                    JacobianWrtVariable.kV, r_contact_frame, rear_contact_disp,
                    world, world)
        J_r_f = plant.CalcJacobianTranslationalVelocity(context,
                    JacobianWrtVariable.kV, r_contact_frame, front_contact_disp,
                    world, world)

        # import pdb; pdb.set_trace()
        impulse_from_contact[i - t_start_idx] = J_l_r.T @ contact_info[0, i]
        impulse_from_contact[i - t_start_idx] = J_l_f.T @ contact_info[1, i]
        impulse_from_contact[i - t_start_idx] = J_r_r.T @ contact_info[2, i]
        impulse_from_contact[i - t_start_idx] = J_r_f.T @ contact_info[3, i]

    #Assuming_the position change is negligible
    # import pdb; pdb.set_trace()
    net_impulse_from_contact = np.zeros(v.shape[1])
    for j in range(v.shape[1]):
        net_impulse_from_contact[j] = np.trapz(impulse_from_contact[:,j],
                                             t_contact_info[
            t_slice])
    delta_v = M_inv @ net_impulse_from_contact
    print("Interval between: ", t_state[t_start_idx], t_state[t_end_idx])
    print(impulses)
    print(delta_v)
    return net_impulse


def plot_nominal_input_traj(u_traj_nominal, state_traj_mode0):
    start_time = u_traj_nominal.start_time()
    end_time = u_traj_nominal.end_time()
    fig = plt.figure('target input trajectory: ' + filename)
    points = []
    times = []
    input_slice = slice(4,8)
    for i in range(1000):
        t = start_time + (end_time - start_time) * i / 1000
        times.append(t)
        points.append(u_traj_nominal.value(t))
    points = np.array(points)
    plt.plot(times, points[:, input_slice, 0])
    plt.legend(state_traj_mode0.datatypes[4 + 74: 8 + 74])


def main():
    global x_traj_nominal
    # Set default directory for saving figures
    matplotlib.rcParams["savefig.directory"] = \
        "/home/yangwill/Documents/research/projects/cassie/jumping/analysis" \
        "/figures/"

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
        "/cassie_v2.urdf")
    plant.mutable_gravity_field().set_gravity_vector(
        -9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    # relevant MBP parameters
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = plant.num_positions() + plant.num_velocities()
    nu = plant.num_actuators()

    pos_map = pydairlib.multibody_utils.makeNameToPositionsMap(plant)
    vel_map = pydairlib.multibody_utils.makeNameToVelocitiesMap(plant)
    act_map = pydairlib.multibody_utils.makeNameToActuatorsMap(plant)

    state_names_w_spr = [[] for i in range(len(pos_map) + len(vel_map))]
    for name in pos_map:
        state_names_w_spr[pos_map[name]] = name
        print(name)
    for name in vel_map:
        state_names_w_spr[nq + vel_map[name]] = name
    # import pdb; pdb.set_trace()

    l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
    r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
    world = plant.world_frame()
    no_offset = np.zeros(3)
    context = plant.CreateDefaultContext()



    loadedStateTraj = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrackingDataTraj = pydairlib.lcm_trajectory.LcmTrajectory()
    # loadedStateTraj.loadFromFile(
    #     "/home/yangwill/Documents/research/projects/cassie/jumping"
    #     "/saved_trajs/target_trajs/jumping_0.2")
    loadedStateTraj.loadFromFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping"
        "/saved_trajs/target_trajs/April_22_jumping_0.2")
        # "/saved_trajs/target_trajs/April_19_jumping_0.2")
    loadedTrackingDataTraj.loadFromFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping"
        "/saved_trajs/target_trajs/April_19_jumping_0.2_processed")
        # "/saved_trajs/target_trajs/jumping_0.2_processed")

    state_traj_mode0 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u0")
    state_traj_mode1 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u1")
    state_traj_mode2 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u2")

    # Useful for optimal lambdas
    decision_vars = loadedStateTraj.getTrajectory("cassie_jumping_decision_vars")


    lcm_l_foot_traj = loadedTrackingDataTraj.getTrajectory(
        "left_foot_trajectory")
    lcm_r_foot_traj = loadedTrackingDataTraj.getTrajectory(
        "right_foot_trajectory")
    lcm_com_traj = loadedTrackingDataTraj.getTrajectory(
        "center_of_mass_trajectory")


    x_points_nominal = np.hstack((state_traj_mode0.datapoints,
                                  state_traj_mode1.datapoints,
                                  state_traj_mode2.datapoints))
    t_nominal = np.hstack((state_traj_mode0.time_vector,
                           state_traj_mode1.time_vector,
                           state_traj_mode2.time_vector))

    nq_fb = 7
    nv_fb = 6
    x_traj_nominal = PiecewisePolynomial.CubicHermite(t_nominal,
                                                      x_points_nominal[:37,
                                                      :],
                                                      x_points_nominal[37:74,
                                                      :])
    u_traj_nominal = PiecewisePolynomial.FirstOrderHold(t_nominal,
                                                        x_points_nominal[-10:])
    l_foot_traj = PiecewisePolynomial.CubicHermite(lcm_l_foot_traj.time_vector,
                                                   lcm_l_foot_traj.datapoints[
                                                   0:3, :],
                                                   lcm_l_foot_traj.datapoints[
                                                   3:6, :])
    r_foot_traj = PiecewisePolynomial.CubicHermite(lcm_r_foot_traj.time_vector,
                                                   lcm_r_foot_traj.datapoints[
                                                   0:3, :],
                                                   lcm_r_foot_traj.datapoints[
                                                   3:6, :])
    com_traj = PiecewisePolynomial.CubicHermite(lcm_com_traj.time_vector,
                                                lcm_com_traj.datapoints[
                                                   0:3, :],
                                                lcm_com_traj.datapoints[
                                                   3:6, :])
    # import pdb; pdb.set_trace()

    # l_foot_traj = PiecewisePolynomial.FirstOrderHold(lcm_l_foot_traj.time_vector,
    #                                                lcm_l_foot_traj.datapoints[
    #                                                0:3, :])
    # r_foot_traj = PiecewisePolynomial.FirstOrderHold(lcm_r_foot_traj.time_vector,
    #                                                lcm_r_foot_traj.datapoints[
    #                                                0:3, :])
    # com_traj = PiecewisePolynomial.FirstOrderHold(lcm_com_traj.time_vector,
    #                                             lcm_com_traj.datapoints[
    #                                                0:3, :])

    # Note this plots relative feet trajs as the target trajectory is relative
    # To get the nominal feet trajs in world coordinates, calculate it from
    # the full state traj


    # generate_state_maps()
    integrate_q_v(x_traj_nominal)
    evaluate_constraints(x_traj_nominal, u_traj_nominal)

    state_names_wo_spr = state_traj_mode0.datatypes

    if len(sys.argv) < 2:
        sys.stderr.write("Provide log file as command line argument!")
        sys.stderr.write("Must be an absolute path")
        sys.exit(1)

    global filename
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    log_samples = int(log.size() / 100)
    # print(log.size() / 100)

    contact_info, contact_info_locs, control_inputs, estop_signal, osc_debug, \
    q, switch_signal, t_contact_info, t_controller_switch, t_osc, t_osc_debug, \
    t_state, v = process_lcm_log.process_log(log, pos_map, vel_map)

    # init_x = np.hstack((q[0,:], v[0,:]))
    # plant.SetPositionsAndVelocities(context, init_x)
    # M = plant.CalcMassMatrixViaInverseDynamics(context)
    # matlab_data = dict(M=M)
    # print(q[0, :])
    # print(plant.)
    # sio.savemat('/home/yangwill/Documents/research/projects/cassie/jumping'
    #             '/logs/M_drake', matlab_data)
    # print(plant.CalcMassMatrixViaInverseDynamics(context))

    # plot_nominal_com_traj(com_traj, lcm_com_traj)
    plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj)
    # plot_nominal_input_traj(u_traj_nominal, state_traj_mode0)

    calcNetImpulse(plant, context, t_contact_info, contact_info, t_state, q, v)

    start_time = 0.0
    end_time = 0.15
    t_start_idx = get_index_at_time(t_state, start_time)
    t_end_idx = get_index_at_time(t_state, end_time)
    t_state_slice = slice(t_start_idx, t_end_idx)

    # plot_simulation_state(q, v, t_state, t_state_slice, state_names_w_spr)
    plot_nominal_state(x_traj_nominal, state_names_wo_spr)

    # For printing out osc_values at a specific time interval
    t_osc_start_idx = get_index_at_time(t_osc_debug, start_time)
    t_osc_end_idx = get_index_at_time(t_osc_debug, end_time)
    t_osc_slice = slice(t_osc_start_idx, t_osc_end_idx)
    print("Num osc samples", t_osc_end_idx - t_osc_start_idx)

    plot_osc_control_inputs(control_inputs, state_traj_mode0, t_osc,
                            t_osc_end_idx, t_osc_start_idx)
    #
    # plot_nominal_control_inputs(nu, state_traj_mode0, t_nominal, x_points_nominal)

    # plot_ground_reaction_forces(contact_info, t_state, t_state_slice)




    # plt.plot(t_osc_debug[t_osc_start_idx:t_osc_end_idx], osc_debug[
    #                                                          0].is_active[
    #                                                      t_osc_start_idx:t_osc_end_idx])
    # Will always be active, because we don't log the osc debug unless its
    # active

    front_contact_disp = np.array((-0.0457, 0.112, 0))
    rear_contact_disp = np.array((0.088, 0, 0))

    plot_feet_from_optimization(x_traj_nominal,
                                front_contact_disp, world, "left_", "_front")

    # Foot plotting
    # plot_feet_simulation(context, l_toe_frame, r_toe_frame, world, no_offset,
    #                      plant, v, q, t_state, t_state_slice)
    if False:
        plot_feet_simulation(plant, context, q, v, l_toe_frame, front_contact_disp,
                             world, t_state, t_state_slice, "left_", "_front")
        plot_feet_simulation(plant, context, q, v, r_toe_frame, front_contact_disp,
                             world, t_state, t_state_slice, "right_", "_front")
        plot_feet_simulation(plant, context, q, v, l_toe_frame, rear_contact_disp,
                             world, t_state, t_state_slice, "left_", "_rear")
        plot_feet_simulation(plant, context, q, v, r_toe_frame, rear_contact_disp,
                             world, t_state, t_state_slice, "right_", "_rear")

    plot = True
    if plot:
        fig = plt.figure("osc_output: " + filename)
        plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].ydot_des[t_osc_slice], \
        label="0")
        plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].ydot[t_osc_slice],
                 label="0")
        plt.legend()
    plt.show()

def evaluate_constraints(x_traj_nominal, u_traj_nominal):
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
        "/cassie_fixed_springs.urdf")
    plant.mutable_gravity_field().set_gravity_vector(
        -9.81 * np.array([0, 0, 1]))
    plant.Finalize()
    context = plant.CreateDefaultContext()

    thigh_left = plant.GetBodyByName("thigh_left").body_frame()
    heel_spring_left = plant.GetBodyByName("heel_spring_left").body_frame()
    thigh_right = plant.GetBodyByName("thigh_right").body_frame()
    heel_spring_right = plant.GetBodyByName("heel_spring_right").body_frame()
    world = plant.world_frame()
    rod_length = 0.5012
    pt_on_heel_spring = np.array((.11877, -.01, 0.0))
    pt_on_thigh_left = np.array((0.0, 0.0, 0.045))
    pt_on_thigh_right = np.array((0.0, 0.0, -0.045))

    breaks = x_traj_nominal.get_segment_times()
    t0 = breaks[8]
    t1 = breaks[9]
    tc = 0.5 * ( breaks[8] + breaks[9] )

    xdot_traj_nominal = x_traj_nominal.derivative(1)

    x0 = x_traj_nominal.value(t0)
    x1 = x_traj_nominal.value(t1)
    xc = x_traj_nominal.value(tc)
    u0 = u_traj_nominal.value(t0)
    u1 = u_traj_nominal.value(t1)
    uc = u_traj_nominal.value(tc)
    xdot_0 = xdot_traj_nominal.value(t0)
    xdot_1 = xdot_traj_nominal.value(t1)
    xdot_c = xdot_traj_nominal.value(tc)

    # no contact forces besides loop closure in this mode
    plant.SetPositionsAndVelocities(context, x0)
    M0 = plant.CalcMassMatrixViaInverseDynamics(context)
    B = plant.MakeActuationMatrix()
    c0 = plant.CalcBiasTerm(context)
    g0 = plant.CalcGravityGeneralizedForces(context)
    J0_thigh_l = plant.CalcJacobianTranslationalVelocity(context,
                                                 JacobianWrtVariable.kV, thigh_left, pt_on_thigh_left,
                                                 world, world)
    J0_thigh_r = plant.CalcJacobianTranslationalVelocity(context,
                                                 JacobianWrtVariable.kV, thigh_right, pt_on_thigh_right,
                                                 world, world)
    J0_heel_l = plant.CalcJacobianTranslationalVelocity(context,
                                                 JacobianWrtVariable.kV, heel_spring_left, pt_on_heel_spring,
                                                 world, world)
    J0_heel_r = plant.CalcJacobianTranslationalVelocity(context,
                                                 JacobianWrtVariable.kV, heel_spring_right, pt_on_heel_spring,
                                                 world, world)
    r_heel_spring_l = plant.CalcPointsPositions(context, heel_spring_left,
                                                pt_on_heel_spring, world)
    r_heel_spring_r = plant.CalcPointsPositions(context, heel_spring_right,
                                                pt_on_heel_spring, world)
    r_thigh_l = plant.CalcPointsPositions(context, thigh_left,
                                                pt_on_thigh_left, world)
    r_thigh_r = plant.CalcPointsPositions(context, thigh_right,
                                         pt_on_thigh_right, world)
    plant.SetPositionsAndVelocities(context, xc)
    Mc = plant.CalcMassMatrixViaInverseDynamics(context)
    cc = plant.CalcBiasTerm(context)
    gc = plant.CalcGravityGeneralizedForces(context)
    Jc_thigh_l = plant.CalcJacobianTranslationalVelocity(context,
                                                         JacobianWrtVariable.kV, thigh_left, pt_on_thigh_left,
                                                         world, world)
    Jc_thigh_r = plant.CalcJacobianTranslationalVelocity(context,
                                                         JacobianWrtVariable.kV, thigh_right, pt_on_thigh_right,
                                                         world, world)
    Jc_heel_l = plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV, heel_spring_left, pt_on_heel_spring,
                                                        world, world)
    Jc_heel_r = plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV, heel_spring_right, pt_on_heel_spring,
                                                        world, world)
    rc_heel_spring_l = plant.CalcPointsPositions(context, heel_spring_left,
                                                pt_on_heel_spring, world)
    rc_heel_spring_r = plant.CalcPointsPositions(context, heel_spring_right,
                                                pt_on_heel_spring, world)
    rc_thigh_l = plant.CalcPointsPositions(context, thigh_left,
                                          pt_on_thigh_left, world)
    rc_thigh_r = plant.CalcPointsPositions(context, thigh_right,
                                          pt_on_thigh_right, world)
    plant.SetPositionsAndVelocities(context, x1)
    M1 = plant.CalcMassMatrixViaInverseDynamics(context)
    c1 = plant.CalcBiasTerm(context)
    g1 = plant.CalcGravityGeneralizedForces(context)

    c0 = np.reshape(c0, (18, 1))
    c1 = np.reshape(c1, (18, 1))
    cc = np.reshape(cc, (18, 1))
    g0 = np.reshape(g0, (18, 1))
    g1 = np.reshape(g1, (18, 1))
    gc = np.reshape(gc, (18, 1))

    nv = plant.num_velocities()
    qddot_0 = xdot_0[-nv:]
    qddot_1 = xdot_1[-nv:]
    qddot_c = xdot_c[-nv:]

    Jc0 = np.zeros((2, nv))
    Jc0[0, :] = 2 * (r_thigh_l - r_heel_spring_l).T @ (J0_thigh_l - J0_heel_l)
    Jc0[1, :] = 2 * (r_thigh_r - r_heel_spring_r).T @ (J0_thigh_r - J0_heel_r)
    Jcc = np.zeros((2, nv))
    Jcc[0, :] = 2 * (rc_thigh_l - rc_heel_spring_l).T @ (Jc_thigh_l -
                                                         Jc_heel_l)
    Jcc[1, :] = 2 * (rc_thigh_r - rc_heel_spring_r).T @ (Jc_thigh_r -
                                                         Jc_heel_r)

    lambda_0 = np.array((193.992, 179.513))
    lambda_0 = np.reshape(lambda_0, (2, 1))
    lambda_c = np.array((125.558, 131.192))
    lambda_c = np.reshape(lambda_c, (2, 1))

    constraint0 = M0 @ qddot_0 - (B @ u0 - c0 + g0 + Jc0.T @ lambda_0)
    constraint1 = M1 @ qddot_1 - (B @ u1 - c1 + g1)
    constraintc = Mc @ qddot_c - (B @ uc - cc + gc + Jcc.T @ lambda_c)
    import pdb; pdb.set_trace()

def plot_ground_reaction_forces(contact_info, t_state, t_state_slice):
    fig = plt.figure('contact data: ' + filename)
    # plt.plot(t_contact_info, contact_info[0, :, 2] + contact_info[1, :, 2],
    plt.plot(t_state[t_state_slice], contact_info[0, t_state_slice, 2],
             label='$\lambda_n left_r$')
    plt.plot(t_state[t_state_slice], contact_info[1, t_state_slice, 2],
             label='$\lambda_n left_f$')
    plt.plot(t_state[t_state_slice], contact_info[2, t_state_slice, 2],
             label='$\lambda_n right_r$')
    plt.plot(t_state[t_state_slice], contact_info[3, t_state_slice, 2],
             label='$\lambda_n right_f$')
    plt.legend()


def plot_nominal_state(x_traj_nominal, state_names_wo_spr):
    x_nominal = []
    xdot_nominal = []
    # v_nominal = []
    # xdot_traj_nominal = x_traj_nominal.derivative(1)
    pos_slice = slice(11, 15)
    vel_slice = slice(19 + 10, 19 + 14)
    t_nominal = np.linspace(x_traj_nominal.start_time(),
                            x_traj_nominal.end_time(), 3000)
    # t_slice = slice(0, )
    for t in (t_nominal):
        x_nominal.append(x_traj_nominal.value(t))
        # xdot_nominal.append(xdot_traj_nominal.value(t))
    # fig = plt.figure('Nominal Traj')
    fig = plt.figure('Nominal state: ' + filename)
    x_nominal = np.array(x_nominal)
    xdot_nominal = np.array(xdot_nominal)
    # v_nominal = np.array(v_nominal)
    plt.plot(t_nominal, x_nominal[:, pos_slice, 0])
    # plt.plot(t_nominal, x_nominal[:, vel_slice, 0])
    # plt.plot(t_nominal, x_nominal[:, 17, 0])
    plt.plot(t_nominal, x_nominal[:, vel_slice, 0])
    # plt.plot(t_nominal, xdot_nominal[:, vel_slice, 0])
    # plt.plot(t_state[t_state_slice], x_nominal[:, pos_slice, 0])
    plt.legend(state_names_wo_spr[pos_slice])
    # plt.legend(state_names_wo_spr[pos_slice])
    # plt.legend(state_names_wo_spr[15])
    plt.plot(x_traj_nominal.get_segment_times(), np.zeros(len(x_traj_nominal.get_segment_times())), 'k*')

def plot_nominal_control_inputs(nu, state_traj_mode0, t_nominal,
                                x_points_nominal):
    fig = plt.figure('target controller inputs: ' + filename)
    input_traj = PiecewisePolynomial.FirstOrderHold(t_nominal,
                                                    x_points_nominal[-10:])
    inputs = np.zeros((1000, nu))
    times = np.zeros(1000)
    for i in range(1000):
        timestep = t_nominal[-1] / 1000 * i
        inputs[i] = input_traj.value(timestep).ravel()
        times[i] = timestep
    plt.plot(times, inputs)
    plt.ylim(-100, 300)
    plt.legend(state_traj_mode0.datatypes[-10:])

def plot_osc_control_inputs(control_inputs, state_traj_mode0, t_osc,
                            t_osc_end_idx, t_osc_start_idx):
    fig = plt.figure('controller inputs: ' + filename)
    osc_indices = slice(t_osc_start_idx, t_osc_end_idx)
    actuator_indices = slice(4, 8)
    plt.plot(t_osc[osc_indices], control_inputs[osc_indices, actuator_indices])
    plt.ylim(-300, 300)
    plt.legend((state_traj_mode0.datatypes[-10:])[actuator_indices])


def integrand(t, y):
    return np.reshape(x_traj_nominal.value(t)[-15:], (15,))

def integrate_q_v(x_traj_nominal):

    times = x_traj_nominal.get_segment_times()
    for i in range(len(times) -1):
        t0 = times[i]
        t1 = times[i+1]

        x0 = x_traj_nominal.value(t0)
        x1 = x_traj_nominal.value(t1)

        q0 = x0[:19]
        v0 = x0[-18:]
        q1 = x1[:19]
        v1 = x1[-18:]

        q0_no_fb = q0[4:]
        q1_no_fb = q1[4:]
        v0_no_fb = v0[3:]
        tspan = [t0, t1]
        q0_no_fb = np.reshape(q0_no_fb, (15,))
        I = integrate.solve_ivp(integrand, tspan, q0_no_fb)
        print(q1_no_fb - np.reshape(I.y[:, -1], (15, 1)))
    # import pdb; pdb.set_trace()


def plot_feet_simulation(plant, context, q, v, toe_frame, contact_point, world,
                         t_state, t_state_slice, foot_type, contact_type):
    foot_state = np.zeros((6, t_state.size))
    # r_foot_state = np.zeros((6, t_state.size))
    for i in range(t_state.size):
        x = np.hstack((q[i, :], v[i, :]))
        plant.SetPositionsAndVelocities(context, x)
        foot_state[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                           contact_point, world)
        foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable.kV, toe_frame, contact_point,
            world,
            world) @ v[i, :]
        # r_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, r_toe_frame,
        #                                                    contact_point, world)
        # r_foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
        #     context, JacobianWrtVariable.kV, r_toe_frame, contact_point,
        #     world,
        #     world) @ v[i, :]
    # fig = plt.figure(foot_type + 'foot pos: ' + filename)
    fig = plt.figure('foot pos: ' + filename)
    state_indices = slice(0, 3)
    state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
    state_names = [foot_type + name for name in state_names]
    state_names = [name + contact_type for name in state_names]
    plt.plot(t_state[t_state_slice], foot_state.T[t_state_slice, state_indices],
             label=state_names[state_indices])
    # plt.plot(t_state[t_state_slice], r_foot_state.T[t_state_slice, state_indices],
    #          label=['xdot_r', 'ydot_r', 'zdot_r'])
    plt.legend()
    # plt.legend(['x pos', 'y pos', 'z pos'])
    # plt.legend(['x pos', 'y pos', 'z pos', 'x vel', 'y vel', 'z vel'])
    # plt.legend(['x pos', 'y pos', 'z pos', 'x vel', 'y vel', 'z vel'])

def plot_feet_from_optimization(x_traj,
                                contact_point, world, foot_type, contact_type):
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
        "/cassie_fixed_springs.urdf")
    plant.mutable_gravity_field().set_gravity_vector(
        -9.81 * np.array([0, 0, 1]))
    plant.Finalize()
    context = plant.CreateDefaultContext()

    toe_frame = plant.GetBodyByName("toe_left").body_frame()
    contact_points = np.zeros(3)
    front_contact_disp = np.array((-0.0457, 0.112, 0))

    n_points = 8000
    foot_state = np.zeros((6, n_points))
    times = np.linspace(x_traj.start_time(), x_traj.end_time(), n_points)
    for i in range(times.shape[0]):
        # x = np.hstack((q[i, :], v[i, :]))
        t = times[i]
        x = x_traj.value(t)
        # x[0:4] = np.array([[1], [0], [0], [0]])
        # x[19:22] = np.zeros((3,1))
        plant.SetPositionsAndVelocities(context, x)
        foot_state[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                         contact_points, world)
        foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable.kV, toe_frame, contact_points,
            world,
            world) @ x[-18:, 0]
        # r_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, r_toe_frame,
        #                                                    contact_point, world)
        # r_foot_state[3:6, i] = plant.CalcJacobianTranslationalVelocity(
        #     context, JacobianWrtVariable.kV, r_toe_frame, contact_point,
        #     world,
        #     world) @ v[i, :]
    # fig = plt.figure(foot_type + 'foot pos: ' + filename)
    fig = plt.figure('foot pos from optimization: ' + filename)
    state_indices = slice(0, 3)
    state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
    state_names = [foot_type + name for name in state_names]
    state_names = [name + contact_type for name in state_names]
    plt.plot(times, foot_state.T[:, (0, 3)],
             label=state_names[state_indices])
    plt.plot(x_traj.get_segment_times(), np.zeros(len(x_traj.get_segment_times(
    ))), 'k*')

    xdot_traj = x_traj.derivative(1)
    # plt.plot(t_state[t_state_slice], r_foot_state.T[t_state_slice, state_indices],
    #          label=['xdot_r', 'ydot_r', 'zdot_r'])
    plt.legend()
    # plt.legend(['x pos', 'y pos', 'z pos'])
    # plt.legend(['x pos', 'y pos', 'z pos', 'x vel', 'y vel', 'z vel'])
    # plt.legend(['x pos', 'y pos', 'z pos', 'x vel', 'y vel', 'z vel'])

def plot_simulation_state(q, v, t_state, t_state_slice, state_names):
    # fig = plt.figure('simulation positions')
    # state_indices = slice(0, q.shape[1])
    n_fb_pos = 7
    n_fb_vel = 6
    state_indices = slice(n_fb_pos, q.shape[1])
    # state_indices = slice(0, n_fb_pos)
    # plt.plot(t_state[t_state_slice], q[t_state_slice, state_indices])
    # plt.legend(state_names[state_indices])

    fig = plt.figure('simulation velocities: ' + filename)

    state_indices = slice(n_fb_vel, v.shape[1])
    plt.plot(t_state[t_state_slice], v[t_state_slice, state_indices])
    plt.legend(state_names[q.shape[1] + n_fb_vel:q.shape[1] + v.shape[1]])


def plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj):
    start_time = lcm_l_foot_traj.time_vector[0]
    end_time = lcm_l_foot_traj.time_vector[-1]
    fig = plt.figure('feet trajectories: ' + filename)
    l_foot_traj_dot = l_foot_traj.derivative(1)
    # r_foot_traj_dot = r_foot_traj.derivative(2)
    l_foot_points = []
    times = []
    n_points = 2000
    for i in range(n_points):
        t = start_time + (end_time - start_time) * i / n_points
        # plt.plot(t, l_foot_traj.value(t).T, 'b.')
        # plt.plot(t, r_foot_traj.value(t).T, 'r.')
        times.append(t)
        # l_foot_points.append(l_foot_traj_dot.value(t))
        l_foot_points.append(l_foot_traj.value(t))
    l_foot_points = np.array(l_foot_points)
    plt.plot(times, l_foot_points[:,:,0])
        # plt.plot(t, r_foot_traj_dot.value(t).T, 'r.')

def plot_nominal_com_traj(com_traj, lcm_com_traj):
    start_time = lcm_com_traj.time_vector[0]
    end_time = lcm_com_traj.time_vector[-1]
    fig = plt.figure('target com trajectory: ' + filename)
    com_dot = com_traj.derivative(1)
    com_ddot = com_traj.derivative(2)
    points = []
    dpoints = []
    ddpoints = []
    times = []
    n_points = 4500
    for i in range(n_points):
        t = start_time + (end_time - start_time) * i / n_points
        times.append(t)
        points.append(com_traj.value(t))
        dpoints.append(com_dot.value(t))
        ddpoints.append(com_ddot.value(t))
    points = np.array(points)
    dpoints = np.array(dpoints)
    ddpoints = np.array(ddpoints)
    plt.plot(times, points[:, :, 0])
    plt.plot(times, dpoints[:, :, 0])
    # plt.plot(times, ddpoints[:, 2, 0])
        # plt.plot(t, com_traj.value(t).T, 'b.')


if __name__ == "__main__":
    main()
