import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
import numpy as np
from pydrake.trajectories import PiecewisePolynomial

def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    # loadedTrajs.loadFromFile(
    #     "/home/yangwill/Documents/research/dairlib/examples/jumping"
    #     "/saved_trajs/jumping_1_14")
    loadedTrajs.LoadFromFile(
        "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.15h_0.3d_processed")
    # print(loadedTrajs.getTrajectoryNames())
    # traj_name = loadedTrajs.getTrajectoryNames()[2]
    traj_mode0 = loadedTrajs.GetTrajectory("left_foot_trajectory")
    traj_mode1 = loadedTrajs.GetTrajectory("right_foot_trajectory")

    traj0 = PiecewisePolynomial.CubicHermite(traj_mode0.time_vector, traj_mode0.datapoints[0:3], traj_mode0.datapoints[3:6])

    times = np.arange(traj0.start_time(), traj0.end_time(), 0.001)
    accel = np.zeros((times.shape[0], 3))
    for i in range(times.shape[0]):
        accel[i, :] = traj0.EvalDerivative(times[i], 2)[:, 0]

    plt.figure('accel')
    plt.plot(traj_mode0.time_vector, traj_mode0.datapoints.T[:,2:3])
    plt.plot(traj_mode0.time_vector, traj_mode0.datapoints.T[:,5:6])
    plt.plot(times, accel[:, -1])
    plt.legend(['pos','vel','accel'])
    # plt.figure("right_foot")
    # plt.plot(traj_mode1.time_vector, traj_mode1.datapoints.T[:,2:3])
    # plt.plot(traj_mode1.time_vector, traj_mode1.datapoints.T[:,5:6])
    # plt.legend(['x','y','z'])

    plt.show()

if __name__ == "__main__":
    main()
