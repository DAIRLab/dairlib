import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

stance_dim = 2


def ReconstructStateTrajectory(x, x_dot):
    traj = PiecewisePolynomial.CubicHermite(x.time_vector, x.datapoints, x_dot.datapoints)
    return traj

def ReconstructStanceTrajectory(stance):
    traj = PiecewisePolynomial.ZeroOrderHold(stance.time_vector, stance.datapoints)
    return traj

def ReconstructForceTrajectory(f):
    traj = PiecewisePolynomial.FirstOrderHold(f.time_vector, f.datapoints)
    return traj

def SaveTrajDatapoints(x_poly, theta_poly, stance_polys, force_poly):
    npoints = 10000
    return 0


def ReconstructFootTrajectory(traj_set):
    z = np.zeros(traj_set[0].value(0).shape)
    y = np.zeros(traj_set[0].value(0).shape)
    y[-1][0] = 0.05
    print(y)
    knots = traj_set[0].value(0)
    knot_derivs = np.zeros(traj_set[0].value(0).shape)
    breaks = np.zeros((1,))
    for i in range(len(traj_set) - 1 ):
        if np.isclose(traj_set[i].end_time(), traj_set[i+1].start_time()):
            knots = np.hstack((knots, traj_set[i].value(traj_set[i].end_time())))
            knot_derivs = np.hstack((knot_derivs,z))
            breaks = np.append(breaks, [traj_set[i].end_time()])
        else:
            end = traj_set[i].value(traj_set[i].end_time())
            beg = traj_set[i+1].value(traj_set[i+1].start_time())
            mid = 0.5 * (beg + end) + y
            vel_mid = (2 / (traj_set[i+1].start_time() - traj_set[i].end_time())) * (beg-end)
            knots = np.hstack((knots, end, mid, beg))
            knot_derivs = np.hstack((knot_derivs, z, vel_mid, z))
            tmid = 0.5 * (traj_set[i].end_time() + traj_set[i+1].start_time())
            breaks = np.append(breaks,  [traj_set[i].end_time(), tmid, traj_set[i+1].start_time()])

    knots = np.hstack((knots, traj_set[-1].value(traj_set[-1].end_time())))
    knot_derivs = np.hstack((knot_derivs,z))
    breaks = np.append(breaks, [traj_set[-1].end_time()])

    return PiecewisePolynomial.CubicHermite(breaks, knots, knot_derivs)



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
    print(sorted(stanceTrajs.GetTrajectoryNames()))
    n = len(stanceTrajs.GetTrajectoryNames())
    left_stance = sorted(stanceTrajs.GetTrajectoryNames())[0:n//2]
    right_stance = sorted(stanceTrajs.GetTrajectoryNames())[n//2:-1]

    traj_names = ['x', 'y', 'theta']
    stance_names = stanceTrajs.GetTrajectoryNames()
    force_names = forceTrajs.GetTrajectoryNames()

    n_points = 5001
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



    l_trajs = []
    r_trajs = []
    for name in left_stance:
        s = stanceTrajs.GetTrajectory(name)
        l_trajs.append(ReconstructStanceTrajectory(s))
    left_foot_traj = ReconstructFootTrajectory(l_trajs)

    for name in right_stance:
        s = stanceTrajs.GetTrajectory(name)
        r_trajs.append(ReconstructStanceTrajectory(s))

    right_foot_traj = ReconstructFootTrajectory(r_trajs)

    left_samples = np.zeros((n_points, left_foot_traj.value(0).shape[0]))
    right_samples = np.zeros((n_points, right_foot_traj.value(0).shape[0]))
    for i in range(n_points):
        left_samples[i] = left_foot_traj.value(t[i])[:,0]
        right_samples[i] = right_foot_traj.value(t[i])[:,0]

    plt.figure('Left Foot Traj')
    plt.plot(t, left_samples)
    plt.legend(['x', 'y'])

    plt.figure('Right Foot Traj')
    plt.plot(t, right_samples)
    plt.legend(['x', 'y'])


    plt.show()


if __name__ == "__main__":
    main()
