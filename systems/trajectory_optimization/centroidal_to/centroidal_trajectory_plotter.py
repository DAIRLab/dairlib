import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

def ReconstructStateTrajectory(x, x_dot):
    traj = PiecewisePolynomial.CubicHermite(x.time_vector, x.datapoints, x_dot.datapoints)
    return traj

def ReconstructStanceTrajectory(stance):
    traj = PiecewisePolynomial.ZeroOrderHold(stance.time_vector, stance.datapoints)
    return traj


def main():
    stateTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    stateTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
    "trajectory_optimization/centroidal_to/CoMtraj.lcmtraj")

    stanceTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    stanceTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
                             "trajectory_optimization/centroidal_to/stancetraj.lcmtraj")

    print(stateTrajs.GetTrajectoryNames())
    print(stanceTrajs.GetTrajectoryNames())

    traj_names = ['x', 'y', 'theta']
    stance_names = stanceTrajs.GetTrajectoryNames()

    n_points = 5000
    for name in traj_names:
        q = stateTrajs.GetTrajectory(name)
        qdot = stateTrajs.GetTrajectory(name + '_dot')
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
    n_points = 3

    plt.figure('stance pos')
    for name in stance_names:
        s = stanceTrajs.GetTrajectory(name)
        pp = ReconstructStanceTrajectory(s)
        t = np.linspace(pp.start_time(), pp.end_time(), n_points)
        samples = np.zeros((n_points, pp.value(0).shape[0]))
        for i in range(n_points):
            samples[i] = pp.value(t[i])[0,0]

        plt.plot(t, samples)

    plt.show()


if __name__ == "__main__":
    main()
