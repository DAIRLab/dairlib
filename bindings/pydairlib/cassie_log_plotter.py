import sys

import dairlib
import drake
import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
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

def main():
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
    loadedStateTraj.loadFromFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping"
        "/saved_trajs/target_trajs/jumping_0.2")
    loadedTrackingDataTraj.loadFromFile(
        "/home/yangwill/Documents/research/projects/cassie/jumping"
        "/saved_trajs/target_trajs/jumping_0.2_processed")

    state_traj_mode0 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u0")
    state_traj_mode1 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u1")
    state_traj_mode2 = loadedStateTraj.getTrajectory(
        "cassie_jumping_trajectory_x_u2")

    # Useful for optimal lambdas
    # decision_vars = loadedStateTraj.getTrajectory("cassie_jumping_decision_vars")

    # import pdb; pdb.set_trace()

    lcm_l_foot_traj = loadedTrackingDataTraj.getTrajectory(
        "left_foot_trajectory")
    lcm_r_foot_traj = loadedTrackingDataTraj.getTrajectory(
        "right_foot_trajectory")

    x_points_nominal = np.hstack((state_traj_mode0.datapoints,
                                  state_traj_mode1.datapoints,
                                  state_traj_mode2.datapoints))
    t_nominal = np.hstack((state_traj_mode0.time_vector,
                           state_traj_mode1.time_vector,
                           state_traj_mode2.time_vector))

    nq_fb = 7
    nv_fb = 6
    x_traj_nominal = PiecewisePolynomial.CubicHermite(t_nominal,
                                                      x_points_nominal[0 +
                                                                       nq_fb:19,
                                                      :],
                                                      x_points_nominal[19 +
                                                                       nv_fb:37,
                                                      :])
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
    # Note this plots relative feet trajs as the target trajectory is relative
    # To get the nominal feet trajs in world coordinates, calculate it from
    # the full state traj

    # plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj)

    # generate_state_maps()

    state_names_wo_spr = state_traj_mode0.datatypes

    if len(sys.argv) < 2:
        sys.stderr.write("Provide log file as command line argument!")
        sys.stderr.write("Must be an absolute path")
        sys.exit(1)

    log = lcm.EventLog(sys.argv[1], "r")
    log_samples = int(log.size() / 100)
    print(log.size() / 100)

    contact_info, contact_info_locs, control_inputs, estop_signal, osc_debug, \
    q, switch_signal, t_contact_info, t_controller_switch, t_osc, t_osc_debug, \
    t_state, v = process_lcm_log.process_log(log, pos_map, vel_map)

    start_time = 1.0
    end_time = 3.0
    t_start_idx = get_index_at_time(t_state, start_time)
    t_end_idx = get_index_at_time(t_state, end_time)
    t_state_slice = slice(t_start_idx, t_end_idx)

    plot_simulation_state(q, v, t_state, t_state_slice, state_names_w_spr)
    # plot_nominal_state(x_traj_nominal, t_state, t_state_slice,
    #                    state_names_wo_spr)

    # For printing out osc_values at a specific time interval
    t_osc_start_idx = get_index_at_time(t_osc_debug, start_time)
    t_osc_end_idx = get_index_at_time(t_osc_debug, end_time)
    t_osc_slice = slice(t_osc_start_idx, t_osc_end_idx)

    plot_osc_control_inputs(control_inputs, state_traj_mode0, t_osc,
                            t_osc_end_idx, t_osc_start_idx)
    #
    # plot_nominal_control_inputs(nu, state_traj_mode0, t_nominal, x_points_nominal)

    plot_ground_reaction_forces(contact_info, t_state, t_state_slice)




    # plt.plot(t_osc_debug[t_osc_start_idx:t_osc_end_idx], osc_debug[
    #                                                          0].is_active[
    #                                                      t_osc_start_idx:t_osc_end_idx])
    # Will always be active, because we don't log the osc debug unless its
    # active

    front_contact_disp = np.array((-0.0457, 0.112, 0))
    rear_contact_disp = np.array((0.088, 0, 0))

    # Foot plotting
    # plot_feet_simulation(context, l_toe_frame, r_toe_frame, world, no_offset,
    #                      plant, v, q, t_state, t_state_slice)
    plot_feet_simulation(plant, context, q, v, l_toe_frame, front_contact_disp,
                         world, t_state, t_state_slice, "left_", "_front")
    plot_feet_simulation(plant, context, q, v, r_toe_frame, front_contact_disp,
                         world, t_state, t_state_slice, "right_", "_front")
    plot_feet_simulation(plant, context, q, v, l_toe_frame, rear_contact_disp,
                         world, t_state, t_state_slice, "left_", "_rear")
    plot_feet_simulation(plant, context, q, v, r_toe_frame, rear_contact_disp,
                         world, t_state, t_state_slice, "right_", "_rear")

    plot = False
    if (plot):
        fig = plt.figure("osc_output")
        # plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].y[t_osc_slice], label="0")
        plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].y_des[t_osc_slice],
                 label="0")
        # plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].error_y[t_osc_slice],
        #          label="y_error")
        # plt.plot(t_osc_debug[t_osc_slice], osc_debug[0].error_dy[t_osc_slice],
        #          label="dy_error")
        # plt.plot(t_osc_debug[t_osc_slice], osc_debug[1].y[t_osc_slice], label="1")
        # plt.plot(t_osc_debug[t_osc_slice], osc_debug[2].y[t_osc_slice], label="2")
        plt.legend()

    plt.show()

def plot_ground_reaction_forces(contact_info, t_state, t_state_slice):
    fig = plt.figure('contact data')
    # plt.plot(t_contact_info, contact_info[0, :, 2] + contact_info[1, :, 2],
    plt.plot(t_state[t_state_slice], contact_info[0, t_state_slice, 2],
             label='$\lambda_n left_f$')
    plt.plot(t_state[t_state_slice], contact_info[1, t_state_slice, 2],
             label='$\lambda_n left_r$')
    plt.plot(t_state[t_state_slice], contact_info[2, t_state_slice, 2],
             label='$\lambda_n right_f$')
    plt.plot(t_state[t_state_slice], contact_info[3, t_state_slice, 2],
             label='$\lambda_n right_r$')
    plt.legend()


def plot_nominal_state(x_traj_nominal, t_state, t_state_slice,
                       state_names_wo_spr):
    q_nominal = []
    v_nominal = []
    v_traj_nominal = x_traj_nominal.derivative(1)
    t_offset = 1.0
    for t in (t_state[t_state_slice]):
        q_nominal.append(x_traj_nominal.value(t - t_offset))
        v_nominal.append(v_traj_nominal.value(t - t_offset))
    # fig = plt.figure('Nominal Traj')
    fig = plt.figure('simulation positions')
    q_nominal = np.array(q_nominal)
    v_nominal = np.array(v_nominal)
    plt.plot(t_state[t_state_slice], v_nominal[:, :, 0])
    # plt.plot(t_state[t_state_slice], q_nominal)
    # plt.legend(state_names_wo_spr[7:19])
    plt.legend(state_names_wo_spr[19 + 6: 37])

def plot_nominal_control_inputs(nu, state_traj_mode0, t_nominal,
                                x_points_nominal):
    fig = plt.figure('target controller inputs')
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
    fig = plt.figure('controller inputs')
    osc_indices = slice(t_osc_start_idx, t_osc_end_idx)
    plt.plot(t_osc[osc_indices], control_inputs[osc_indices])
    plt.ylim(-100, 300)
    plt.legend(state_traj_mode0.datatypes[-10:])

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
    fig = plt.figure('l foot pos')
    state_indices = slice(2, 3)
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


def plot_simulation_state(q, v, t_state, t_state_slice, state_names):
    fig = plt.figure('simulation positions')
    # state_indices = slice(0, q.shape[1])
    n_fb_states = 7
    state_indices = slice(n_fb_states, q.shape[1])
    # state_indices = slice(0, n_fb_states)
    plt.plot(t_state[t_state_slice], q[t_state_slice, state_indices])
    plt.legend(state_names[state_indices])

    # fig = plt.figure('simulation velocities')

    # state_indices = slice(0, v.shape[1])
    # plt.plot(t_state[t_state_slice], v[t_state_slice, state_indices])
    # plt.legend(state_names[q.shape[1]:q.shape[1] + v.shape[1]])


def plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj):
    start_time = lcm_l_foot_traj.time_vector[0]
    end_time = lcm_l_foot_traj.time_vector[-1]
    fig = plt.figure('feet trajectories')
    for i in range(1000):
        t = start_time + (end_time - start_time) * i / 1000
        plt.plot(t, l_foot_traj.value(t).T, 'b.')
        plt.plot(t, r_foot_traj.value(t).T, 'r.')


if __name__ == "__main__":
    main()
