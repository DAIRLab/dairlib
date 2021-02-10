import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial


def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
    "trajectory_optimization/centroidal_to/CoMtraj.lcmtraj")
    print(loadedTrajs.GetTrajectoryNames())
    traj_name = loadedTrajs.GetTrajectoryNames()[0]
    traj = loadedTrajs.GetTrajectory(traj_name)
    print(traj.datatypes)
    plt.plot(traj.time_vector, traj.datapoints.T)
    plt.show()

if __name__ == "__main__":
    main()
