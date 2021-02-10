import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

def ReconstructStateTrajectory(x, x_dot):
    traj = PiecewisePolynomial.CubicHermite(x.time_vector, x.datapoints, x_dot.datapoints)
    return traj



def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
    "trajectory_optimization/centroidal_to/CoMtraj.lcmtraj")
    print(loadedTrajs.GetTrajectoryNames())

    traj_names = ['x', 'y', 'theta']

    n_points = 500
    for name in traj_names:
        q = loadedTrajs.GetTrajectory(name)
        qdot = loadedTrajs.GetTrajectory(name + '_dot')
        pp = ReconstructStateTrajectory(q, qdot)
        t = np.linspace(pp.start_time(), pp.end_time(), n_points)
        qsamples = np.zeros((n_points, pp.value(0).shape[0]))
        qdotsamples = np.zeros((n_points, pp.value(0).shape[0]))
        for i in range(n_points):
            qsamples[i] = pp.value(t[i])[:,0]
            qdotsamples[i] = pp.derivative(1).value(t[i])[:,0]

        plt.figure(name)
        plt.plot(t, qsamples)
        plt.plot(t, qdotsamples)
        plt.legend([name, name + '_dot'])

    plt.show()


if __name__ == "__main__":
    main()
