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

def ReconstructForceTrajectory(f):
    traj = PiecewisePolynomial.FirstOrderHold(f.time_vector, f.datapoints)
    return traj

def SaveTrajDatapoints(x_poly, theta_poly, stances, force_poly):
    npoints = 10000
    return 0

class StanceTrajGenerator:
    def __init__(self, stances):
        self.stances = stances
        self.start_times = np.zeros((stances.len(),))
        self.end_times = np.zeros((stances.len(),))

        for i,stance in enumerate(stances):
            self.start_times[i] = stance.start_time()
            self.end_times[i] = stance.end_time()
            self.stance_intervals.append((self.start_times[i], self.end_times[i]))


    def GetSwingLegLoc(self, t):
        idx2 = np.argwhere(self.end_times >=t)[0]
        print(self.stances[idx2].value(t))
        return self.stances[idx2].value(t)




def main():
    stateTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    stateTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
    "trajectory_optimization/centroidal_to/CoMtraj.lcmtraj")

    stanceTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    stanceTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
                             "trajectory_optimization/centroidal_to/stancetraj.lcmtraj")
    forceTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    forceTrajs.LoadFromFile("/home/brian/workspace/dairlib/systems/"
                            "trajectory_optimization/centroidal_to/forcetraj.lcmtraj")

    print(stateTrajs.GetTrajectoryNames())
    print(stanceTrajs.GetTrajectoryNames())

    traj_names = ['x', 'y', 'theta']
    stance_names = stanceTrajs.GetTrajectoryNames()
    force_names = forceTrajs.GetTrajectoryNames()

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

    plt.figure('contact_forces')
    for name in force_names:
        f = forceTrajs.GetTrajectory(name)
        pp = ReconstructForceTrajectory(f)
        force_samples = np.zeros((n_points, pp.value(0).shape[0]))
        for i in range(n_points):
            force_samples[i] = pp.value(t[i])[:,0]

        plt.plot(t, force_samples)

    plt.legend([force_names[0] + '_x', force_names[0] + '_y',
                force_names[1] + '_x', force_names[1] + '_y'])


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
