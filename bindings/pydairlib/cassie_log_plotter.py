import sys
import lcm
import numpy as np
import matplotlib.pyplot as plt
import dairlib
import drake
import pydairlib.lcm_trajectory
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable

class lcmt_osc_tracking_data_t:
    def __init__(self):
        self.y_dim = 0
        self.name = ""
        self.is_active = []
        self.y = []
        self.y_des = []
        self.error_y = []
        self.dy = []
        self.dy_des = []
        self.error_dy = []
        self.ddy_des = []
        self.ddy_command = []
        self.ddy_command_sol = []
    def append(self, msg):
        self.is_active.append(msg.is_active)
        self.y.append(msg.y)
        self.y_des.append(msg.y_des)
        self.error_y.append(msg.error_y)
        self.dy.append(msg.dy)
        self.dy_des.append(msg.dy_des)
        self.error_dy.append(msg.error_dy)
        self.ddy_des.append(msg.ddy_des)
        self.ddy_command.append(msg.ddy_command)
        self.ddy_command_sol.append(msg.ddy_command_sol)
    def convertToNP(self):
        self.is_active = np.array(self.is_active)
        self.y = np.array(self.y)
        self.y_des = np.array(self.y_des)
        self.error_y = np.array(self.error_y)
        self.dy = np.array(self.dy)
        self.dy_des = np.array(self.dy_des)
        self.error_dy = np.array(self.error_dy)
        self.ddy_des = np.array(self.ddy_des)
        self.ddy_command = np.array(self.ddy_command)
        self.ddy_command_sol = np.array(self.ddy_command_sol)

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

def plot_nominal_traj(traj_mode0, traj_mode1, traj_mode2):
    fig = plt.figure('Nominal Traj')
    indices = slice(0,19)
    plt.plot(traj_mode0.time_vector, traj_mode0.datapoints.T[:,indices])
    plt.plot(traj_mode1.time_vector, traj_mode1.datapoints.T[:,indices])
    plt.plot(traj_mode2.time_vector, traj_mode2.datapoints.T[:,indices])
    print(traj_mode0.datatypes)
    plt.legend(traj_mode0.datatypes[indices])

def main():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
        "/cassie_v2.urdf")
    plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    # relevant MBP parameters
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = plant.num_positions() + plant.num_velocities()
    nu = plant.num_actuators()

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

    state_traj_mode0 = loadedStateTraj.getTrajectory("cassie_jumping_trajectory_x_u0")
    state_traj_mode1 = loadedStateTraj.getTrajectory("cassie_jumping_trajectory_x_u1")
    state_traj_mode2 = loadedStateTraj.getTrajectory("cassie_jumping_trajectory_x_u2")

    lcm_l_foot_traj = loadedTrackingDataTraj.getTrajectory("left_foot_trajectory")
    lcm_r_foot_traj = loadedTrackingDataTraj.getTrajectory("right_foot_trajectory")

    # plot_nominal_traj(state_traj_mode0, state_traj_mode1, state_traj_mode2)

    x_points_nominal = np.hstack((state_traj_mode0.datapoints,
                                  state_traj_mode1.datapoints,
                                  state_traj_mode2.datapoints))
    t_nominal = np.hstack((state_traj_mode0.time_vector,
                           state_traj_mode1.time_vector,
                           state_traj_mode2.time_vector))

    # x_traj_nominal = PiecewisePolynomial.Cubic(t_nominal,
    #                                            x_points_nominal[0:19,:],
    #                                            x_points_nominal[19:37,:])
    l_foot_traj = PiecewisePolynomial.Cubic(lcm_l_foot_traj.time_vector,
                                            lcm_l_foot_traj.datapoints[0:3,:],
                                            lcm_l_foot_traj.datapoints[3:6,:])
    r_foot_traj = PiecewisePolynomial.Cubic(lcm_r_foot_traj.time_vector,
                                            lcm_r_foot_traj.datapoints[0:3,:],
                                            lcm_r_foot_traj.datapoints[3:6,:])

    # plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj)

    generate_state_maps()

    state_names = state_traj_mode0.datatypes

    if len(sys.argv) < 2:
        sys.stderr.write("Provide log file as command line argument!")
        sys.stderr.write("Must be an absolute path")
        sys.exit(1)

    log = lcm.EventLog(sys.argv[1], "r")
    log_samples = int(log.size()/100)
    print(log.size()/100)

    t_state = []
    t_osc = []
    t_controller_switch = []
    t_osc_debug = []
    t_contact_info = []
    q = []
    v = []
    control_inputs = []
    estop_signal = []
    switch_signal = []
    osc_debug = [lcmt_osc_tracking_data_t() for i in range(4)]
    contact_info = [[], [], [], []]
    contact_info, control_inputs, estop_signal, log, osc_debug, \
    q, switch_signal, t_contact_info, \
    t_controller_switch, t_osc, t_osc_debug, t_state, v = process_log(contact_info,
                                                                      control_inputs,
                                                                      estop_signal, log,
                                                                      osc_debug,
                                                                      q,
                                                                      switch_signal,
                                                                      t_contact_info,
                                                                      t_controller_switch,
                                                                      t_osc, t_osc_debug,
                                                                      t_state, v)

    t_state_slice = plot_simulation_state(q, t_state, state_names)

    # For printing out osc_values at a specific time interval
    t_osc_start_idx = get_index_at_time(t_osc_debug, 1)
    t_osc_end_idx = get_index_at_time(t_osc_debug, 2)

    fig = plt.figure('controller inputs')
    osc_indices = slice(t_osc_start_idx, t_osc_end_idx)
    plt.plot(t_osc[osc_indices], control_inputs[osc_indices])
    plt.ylim(-100,300)
    plt.legend(state_traj_mode0.datatypes[-10:])

    fig = plt.figure('target controller inputs')
    input_traj = PiecewisePolynomial.FirstOrderHold(t_nominal,
                                                    x_points_nominal[-10:])
    inputs = np.zeros((1000, nu))
    times = np.zeros(1000)
    for i in range(1000):
        timestep = t_nominal[-1]/1000*i
        inputs[i] = input_traj.value(timestep).ravel()
        times[i] = timestep
    plt.plot(times, inputs)
    plt.ylim(-100,300)
    plt.legend(state_traj_mode0.datatypes[-10:])


    fig = plt.figure('contact data')
    plt.plot(t_contact_info, contact_info[0, :, 2] + contact_info[1, :, 2],
             label='$\lambda_n left$')
    plt.plot(t_contact_info, contact_info[2, :, 2] + contact_info[3, :, 2],
             label='$\lambda_n right$')
    plt.legend()

    # plt.plot(t_osc_debug[t_osc_start_idx:t_osc_end_idx], osc_debug[
    #                                                          0].is_active[
    #                                                      t_osc_start_idx:t_osc_end_idx])
    # Will always be active, because we don't log the osc debug unless its
    # active

    plot_feet_simulation(context, l_toe_frame, no_offset, plant, q, r_toe_frame,
                         t_state, t_state_slice, v, world)

    plt.show()
    # print_osc_debug(t_idx, 1, osc_debug[0])
    # print_osc_debug(t_idx+1, 1, osc_debug[0])

    plot = False
    if(plot):
        plt.plot(t_osc_debug, osc_debug.error_y[:,1], label="error_y")
        plt.plot(t_osc_debug, osc_debug.error_dy[:,1], label="error_dy")
        plt.plot(t_osc_debug, osc_debug.ddy_command[:,1], label="ddy_command")
        # plt.plot(t_osc_debug, osc_debug_2.error_y[:,1], label="rot_error_y")
        # plt.plot(t_osc_debug, osc_debug_2.error_dy[:,1], label="rot_error_dy")
        # plt.plot(t_osc_debug, osc_debug_2.ddy_command[:,1], label="rot_ddy_command")

        # plt.legend(("knee_left", "hip_pitch_left", "knee_left_dot", "hip_pitch_left_dot","knee_left_motor", "knee_right_motor", "hip_pitch_left_motor","knee_left_motor", "knee_right_motor"))
        # plt.legend()
        # plt.show()

    # pos_plot = plt.subplot(2,1,1)
    # vel_plot = plt.subplot(2,1,2)
    # pos_plot.plot(t[:i], q.transpose()[:i,pos_map["knee_left"]], 'r-')
    # pos_plot.plot(t[:i], q.transpose()[:i,pos_map["hip_pitch_left"]], 'b-')
    # vel_plot.plot(t[:i], v.transpose()[:i,vel_map["knee_left_dot"]], 'r--')
    # vel_plot.plot(t[:i], v.transpose()[:i,vel_map["hip_pitch_left_dot"]], 'b--')
    # vel_plot.plot(t_j[:j], u.transpose()[:j,effort_map["knee_left_motor"]], 'g-')
    # vel_plot.plot(t_j[:j], u.transpose()[:j,effort_map["hip_pitch_left_motor"]], 'c-')
    # plt.plot(t, v.transpose()[:,12])

    plt.show()


def plot_feet_simulation(context, l_toe_frame, no_offset, plant, q, r_toe_frame,
                         t_state, t_state_slice, v, world):
    l_foot_state = np.zeros((6, t_state.size))
    r_foot_state = np.zeros((6, t_state.size))
    for i in range(t_state.size):
        x = np.hstack((q[i, :], v[i, :]))
        plant.SetPositionsAndVelocities(context, x)
        l_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, l_toe_frame,
                                                           no_offset, world)
        r_foot_state[0:3, [i]] = plant.CalcPointsPositions(context, r_toe_frame,
                                                           no_offset, world)
    fig = plt.figure('l foot pos')
    plt.plot(t_state[t_state_slice], l_foot_state.T[t_state_slice, :])
    plt.plot(t_state[t_state_slice], r_foot_state.T[t_state_slice, :])
    plt.legend(['x pos', 'y pos', 'z pos'])


def plot_simulation_state(q, t_state, state_names):
    fig = plt.figure('simulation state')
    t_start_idx = get_index_at_time(t_state, 1)
    t_end_idx = get_index_at_time(t_state, 2)
    t_state_slice = slice(t_start_idx, t_end_idx)
    state_indices = slice(0, q.shape[1])
    plt.plot(t_state[t_state_slice], q[t_state_slice, state_indices])
    plt.legend(state_names[0:q.shape[1]])
    return t_state_slice


def generate_state_maps():
    pos_map = dict()
    vel_map = dict()
    mot_map = dict()
    pos_map = {
        "q_w": 0,
        "q_x": 1,
        "q_y": 2,
        "q_z": 3,
        "base_x": 4,
        "base_y": 5,
        "base_z": 6,
        "hip_roll_left": 7,
        "hip_roll_right": 8,
        "hip_yaw_left": 9,
        "hip_yaw_right": 10,
        "hip_pitch_left": 11,
        "hip_pitch_right": 12,
        "knee_left": 13,
        "knee_right": 14,
        "ankle_left": 15,
        "ankle_right": 16,
        "toe_left": 17,
        "toe_right": 18
    }
    vel_map = {
        "q_x_dot": 0,
        "q_y_dot": 1,
        "q_z_dot": 2,
        "base_x_dot": 3,
        "base_y_dot": 4,
        "base_z_dot": 5,
        "hip_roll_left_dot": 6,
        "hip_roll_right_dot": 7,
        "hip_yaw_left_dot": 8,
        "hip_yaw_right_dot": 9,
        "hip_pitch_left_dot": 10,
        "hip_pitch_right_dot": 11,
        "knee_left_dot": 12,
        "knee_right_dot": 13,
        "ankle_left_dot": 14,
        "ankle_right_dot": 15,
        "toe_left_dot": 16,
        "toe_right_dot": 17
    }
    effort_map = {
        "hip_roll_left_motor": 0,
        "hip_roll_right_motor": 1,
        "hip_yaw_left_motor": 2,
        "hip_yaw_right_motor": 3,
        "hip_pitch_left_motor": 4,
        "hip_pitch_right_motor": 5,
        "knee_left_motor": 6,
        "knee_right_motor": 7,
        "toe_left_motor": 8,
        "toe_right_motor": 9
    }


def plot_nominal_feet_traj(l_foot_traj, lcm_l_foot_traj, r_foot_traj):
    start_time = lcm_l_foot_traj.time_vector[0]
    end_time = lcm_l_foot_traj.time_vector[-1]
    fig = plt.figure('feet trajectories')
    for i in range(1000):
        t = start_time + (end_time - start_time) * i / 1000
        plt.plot(t, l_foot_traj.value(t).T, 'b.')
        plt.plot(t, r_foot_traj.value(t).T, 'r.')


def process_log(contact_info, control_inputs, estop_signal, log, osc_debug,
                q, switch_signal, t_contact_info,
                t_controller_switch, t_osc, t_osc_debug, t_state, v):
    for event in log:
        if event.channel == "CASSIE_STATE":
            msg = dairlib.lcmt_robot_output.decode(event.data)
            q.append(msg.position)
            v.append(msg.velocity)
            t_state.append(msg.utime / 1e6)
        if event.channel == "CASSIE_INPUT":
            msg = dairlib.lcmt_robot_input.decode(event.data)
            control_inputs.append(msg.efforts)
            t_osc.append(msg.utime / 1e6)
        if event.channel == "INPUT_SWITCH":
            msg = dairlib.lcmt_controller_switch.decode(event.data)
            switch_signal.append(msg.channel == "OSC_STANDING")
            t_controller_switch.append(msg.utime / 1e6)
        if event.channel == "OSC_DEBUG":
            msg = dairlib.lcmt_osc_output.decode(event.data)
            num_osc_tracking_data = len(msg.tracking_data)
            for i in range(num_osc_tracking_data):
                osc_debug[i].append(msg.tracking_data[0])
            t_osc_debug.append(msg.utime / 1e6)
        if event.channel == "CASSIE_CONTACT_RESULTS":
            msg = drake.lcmt_contact_results_for_viz.decode(event.data)
            t_contact_info.append(msg.timestamp / 1e6)
            num_left_contacts = 0
            num_right_contacts = 0
            for i in range(msg.num_point_pair_contacts):
                if msg.point_pair_contact_info[i].body2_name == "toe_left(2)":
                    contact_info[num_left_contacts].append(
                        msg.point_pair_contact_info[
                            i].contact_force)
                    num_left_contacts += 1
                elif msg.point_pair_contact_info[
                    i].body2_name == "toe_right(2)":
                    contact_info[2 + num_right_contacts].append(
                        msg.point_pair_contact_info[
                            i].contact_force)
                    num_right_contacts += 1
                else:
                    print("ERROR")
            while num_left_contacts != 2:
                contact_info[num_left_contacts].append((0.0, 0.0, 0.0))
                num_left_contacts += 1
            while num_right_contacts != 2:
                contact_info[2 + num_right_contacts].append((0.0, 0.0, 0.0))
                num_right_contacts += 1
    # Convert into numpy arrays
    t_state = np.array(t_state)
    t_osc = np.array(t_osc)
    t_controller_switch = np.array(t_controller_switch)
    t_contact_info = np.array(t_contact_info)
    t_osc_debug = np.array(t_osc_debug)
    q = np.array(q)
    v = np.array(v)
    control_inputs = np.array(control_inputs)
    estop_signal = np.array(estop_signal)
    switch_signal = np.array(switch_signal)
    contact_info = np.array(contact_info)
    for i in range(len(osc_debug)):
        osc_debug[i].convertToNP()
    # osc_debug.convertToNP()
    return contact_info, control_inputs, estop_signal, log, osc_debug, \
           q, switch_signal, t_contact_info, \
           t_controller_switch, t_osc, t_osc_debug, t_state, v


if __name__ == "__main__":
    main()
