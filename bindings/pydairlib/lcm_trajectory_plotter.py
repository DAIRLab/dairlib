import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
import numpy as np
from pydrake.trajectories import PiecewisePolynomial

def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    # loadedTrajs.LoadFromFile(
    #     "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d_processed")
    loadedTrajs.LoadFromFile(
        "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/walking_0.16.0_processed")

    lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory0")
    lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory0")

    left_foot_traj = PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[0:3], lcm_left_foot_traj.datapoints[3:6])
    right_foot_traj = PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[0:3], lcm_right_foot_traj.datapoints[3:6])
    for mode in range(1, 2):
        lcm_left_foot_traj = loadedTrajs.GetTrajectory("left_foot_trajectory" + str(mode))
        lcm_right_foot_traj = loadedTrajs.GetTrajectory("right_foot_trajectory" + str(mode))

        left_foot_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints[0:3], lcm_left_foot_traj.datapoints[3:6]))
        right_foot_traj.ConcatenateInTime(PiecewisePolynomial.CubicHermite(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints[0:3], lcm_right_foot_traj.datapoints[3:6]))


    times = np.arange(right_foot_traj.start_time(), right_foot_traj.end_time(), 0.001)
    accel = np.zeros((times.shape[0], 3))
    pos = np.zeros((times.shape[0], 3))
    vel = np.zeros((times.shape[0], 3))
    for i in range(times.shape[0]):
        accel[i, :] = right_foot_traj.EvalDerivative(times[i], 2)[:, 0]
        pos[i, :] = right_foot_traj.value(times[i])[:, 0]
        vel[i, :] = right_foot_traj.EvalDerivative(times[i], 1)[:, 0]

    # plt.figure('accel')
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,2:3])
    # plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,5:6])
    # plt.plot(times, accel[:, -1])
    plt.figure("left_foot pos")
    plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,0:3])
    plt.legend(['x','y','z'])
    plt.figure("left_foot vel")
    plt.plot(lcm_left_foot_traj.time_vector, lcm_left_foot_traj.datapoints.T[:,3:6])
    plt.legend(['x','y','z'])
    plt.figure("right_foot pos")
    plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,0:3])
    plt.legend(['x','y','z'])
    plt.figure("right_foot vel")
    plt.plot(lcm_right_foot_traj.time_vector, lcm_right_foot_traj.datapoints.T[:,3:6])
    plt.legend(['x','y','z'])

    plt.figure("right foot vel")
    plt.plot(times, vel)
    plt.legend(['x','y','z'])
    # plt.legend(['pos','vel','accel'])

    plt.show()

if __name__ == "__main__":
    main()
