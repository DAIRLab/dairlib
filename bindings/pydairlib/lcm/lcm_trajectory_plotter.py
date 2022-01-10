import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
import numpy as np
from pydrake.trajectories import PiecewisePolynomial


def main():
    loadedTrajs = lcm_trajectory.LcmTrajectory()
    # loadedTrajs.LoadFromFile(
    #     "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d_processed")
    # loadedTrajs.LoadFromFile(
    #     "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/walking_0.16.0_processed")
    loadedTrajs.LoadFromFile(
        "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/running_0.00_processed_rel")

    lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory0")
    lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory0")
    lcm_left_hip_traj = loadedTrajs.GetTrajectory("left_hip_trajectory0")
    lcm_right_hip_traj = loadedTrajs.GetTrajectory("right_hip_trajectory0")
    lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_trans_trajectory0")
    # lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_rot_trajectory0")

    x_slice = slice(0, 6)
    xdot_slice = slice(3, 9)
    left_foot_traj = PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector,
                                                      lcm_left_foot_traj.datapoints[x_slice],
                                                      lcm_left_foot_traj.datapoints[xdot_slice])
    right_foot_traj = PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector,
                                                       lcm_right_foot_traj.datapoints[x_slice],
                                                       lcm_right_foot_traj.datapoints[xdot_slice])
    left_hip_traj = PiecewisePolynomial.CubicHermite(lcm_left_hip_traj.time_vector,
                                                     lcm_left_hip_traj.datapoints[x_slice],
                                                     lcm_left_hip_traj.datapoints[xdot_slice])
    right_hip_traj = PiecewisePolynomial.CubicHermite(lcm_right_hip_traj.time_vector,
                                                      lcm_right_hip_traj.datapoints[x_slice],
                                                      lcm_right_hip_traj.datapoints[xdot_slice])
    pelvis_traj = PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[x_slice],
                                                   lcm_pelvis_traj.datapoints[xdot_slice])
    for mode in range(1, 6):
        lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory" + str(mode))
        lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory" + str(mode))
        lcm_left_hip_traj = loadedTrajs.GetTrajectory("left_hip_trajectory" + str(mode))
        lcm_right_hip_traj = loadedTrajs.GetTrajectory("right_hip_trajectory" + str(mode))
        lcm_pelvis_traj = loadedTrajs.GetTrajectory("pelvis_trans_trajectory" + str(mode))

        left_foot_traj.ConcatenateInTime(
            PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[x_slice],
                                             lcm_left_foot_traj.datapoints[xdot_slice]))
        right_foot_traj.ConcatenateInTime(
            PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[x_slice],
                                             lcm_right_foot_traj.datapoints[xdot_slice]))
        left_hip_traj.ConcatenateInTime(
            PiecewisePolynomial.CubicHermite(lcm_left_hip_traj.time_vector, lcm_left_hip_traj.datapoints[x_slice],
                                             lcm_left_hip_traj.datapoints[xdot_slice]))
        right_hip_traj.ConcatenateInTime(
            PiecewisePolynomial.CubicHermite(lcm_right_hip_traj.time_vector, lcm_right_hip_traj.datapoints[x_slice],
                                             lcm_right_hip_traj.datapoints[xdot_slice]))
        pelvis_traj.ConcatenateInTime(
            PiecewisePolynomial.CubicHermite(lcm_pelvis_traj.time_vector, lcm_pelvis_traj.datapoints[x_slice],
                                             lcm_pelvis_traj.datapoints[xdot_slice]))

    # plt.figure('accel')
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,2:3])
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,5:6])
    # plt.plot(times, accel[:, -1])
    # plt.figure("left_foot pos")
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,0:3])
    # plt.legend(['x','y','z'])
    # plt.figure("left_foot vel")
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,3:6])
    # plt.legend(['x','y','z'])
    # plt.figure("right_foot pos")
    # plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,0:3])
    # plt.legend(['x','y','z'])
    # plt.figure("right_foot vel")
    # plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,3:6])
    # plt.legend(['x','y','z'])

    # reconstruct_trajectory(left_foot_traj)
    reconstruct_trajectory_left_ft(left_foot_traj)

    # plot_trajectory(pelvis_traj, 'pelvis')
    # plot_trajectory(left_foot_traj - left_hip_traj, 'left_foot')
    # plot_trajectory(right_foot_traj, 'right_foot')
    plt.show()


def plot_trajectory(traj_of_interest, traj_name):
    times = np.arange(traj_of_interest.start_time(), traj_of_interest.end_time(), 5e-4)
    # times = np.arange(traj_of_interest.start_time(), 0.2, 0.001)
    y_dim = 3
    accel = np.zeros((times.shape[0], y_dim))
    pos = np.zeros((times.shape[0], y_dim))
    vel = np.zeros((times.shape[0], y_dim))
    for i in range(times.shape[0]):
        pos[i, :] = traj_of_interest.value(times[i])[:3, 0]
        vel[i, :] = traj_of_interest.EvalDerivative(times[i], 1)[:3, 0]
        accel[i, :] = traj_of_interest.EvalDerivative(times[i], 2)[-3:, 0]

    plt.figure(traj_name + "_pos")
    plt.plot(times, pos)
    plt.legend(['x', 'y', 'z', 'xdot', 'ydot', 'zdot'])
    plt.figure(traj_name + "xz")
    plt.plot(pos[:, 0], pos[:, 2])
    plt.legend(['xz'])
    plt.figure(traj_name + "yz")
    plt.plot(pos[:, 1], pos[:, 2])
    plt.legend(['yz'])
    plt.figure(traj_name + "_vel")
    plt.plot(times, vel)
    plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
    plt.figure(traj_name + "_acc")
    plt.plot(times, accel)
    plt.legend(['xdot', 'ydot', 'zdot', 'xddot', 'yddot', 'zddot'])
    # plt.legend(['pos','vel','accel'])


def reconstruct_trajectory(trajectory):
    T_waypoints = np.array([0.4, 0.8, 1.0])
    Y = np.zeros((3, 3))
    start_pos = np.array([0, 0, 0])
    end_pos = np.array([0.2, 0.1, 0])
    Y[0] = start_pos
    Y[1] = start_pos + 0.85 * (end_pos - start_pos)
    Y[1, 2] += 0.05
    Y[2] = end_pos
    Y = Y.T
    print(Y)

    Ydot_start = np.zeros(3)
    Ydot_end = np.zeros(3)

    # traj = PiecewisePolynomial.CubicShapePreserving(T_waypoints, Y, True)
    # traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(T_waypoints, Y, Ydot_start, Ydot_end)
    traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(T_waypoints, Y, Ydot_start, Ydot_end)

    plot_trajectory(traj, "reconstructed_trajectory")
    return traj


def reconstruct_trajectory_left_ft(trajectory):
    ref = reconstruct_trajectory(trajectory)

    T_waypoints_0 = np.array([0.0, 0.3, 0.5])
    T_waypoints_midpoint = np.array([0.5, 0.9])
    T_waypoints_1 = np.array([0.9, 1.0])
    Y = np.zeros((3, 3))

    start_pos = ref.value(0.5)[:, 0]
    mid_pos = ref.value(0.8)[:, 0]
    # end_pos = np.array([0.2, 0.1, 0])
    end_pos = ref.value(1.0)[:, 0]
    Y[0] = start_pos
    Y[1] = mid_pos
    Y[2] = end_pos
    Y = Y.T
    print(Y)

    segment_0_start = ref.value(0.5)[:, 0]
    segment_0_midpoint = ref.value(0.8)[:, 0]
    segment_0_end = ref.value(1.0)[:, 0]
    d_segment_0_start = ref.EvalDerivative(0.5, 1)[:, 0]
    d_segment_0_midpoint = ref.EvalDerivative(0.8, 1)[:, 0]
    d_segment_0_end = ref.EvalDerivative(1.0, 1)[:, 0]
    segment_1_start = ref.value(0.4)[:, 0]
    segment_1_end = ref.value(0.5)[:, 0]
    d_segment_1_start = ref.EvalDerivative(0.4, 1)[:, 0]
    d_segment_1_end = ref.EvalDerivative(0.5, 1)[:, 0]

    Ydot_start = np.zeros(3)
    Ydot_end = np.zeros(3)

    print(np.array([end_pos, segment_1_start]))
    # traj = PiecewisePolynomial.CubicShapePreserving(T_waypoints, Y, True)
    traj_0 = PiecewisePolynomial.CubicHermite(T_waypoints_0,
                                              np.array([segment_0_start, segment_0_midpoint, segment_0_end]).T,
                                              np.array([d_segment_0_start, d_segment_0_midpoint, d_segment_0_end]).T)
    traj_midpoint = PiecewisePolynomial.ZeroOrderHold(T_waypoints_midpoint, np.array([end_pos, end_pos]).T)
    traj_1 = PiecewisePolynomial.CubicHermite(T_waypoints_1, np.array([segment_1_start, segment_1_end]).T,
                                              np.array([d_segment_1_start, d_segment_1_end]).T)

    traj_0.ConcatenateInTime(traj_midpoint)
    traj_0.ConcatenateInTime(traj_1)
    plot_trajectory(traj_0, "reconstructed_trajectory_left")


if __name__ == "__main__":
    main()
