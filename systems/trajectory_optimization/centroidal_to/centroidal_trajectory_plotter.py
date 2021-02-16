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

def SaveTrajDatapoints(x_poly, y_poly, theta_poly, left_poly, right_poly, left_force_poly, right_force_poly):
    npoints = 10001

    t = np.linspace(x_poly.start_time(), x_poly.end_time(), npoints)
    traj = np.zeros((18, npoints))
    input = np.zeros((8,npoints))

    for i in range(npoints):
        traj[0:1,i] = x_poly.value(t[i])[:,0]
        traj[1:2,i] = y_poly.value(t[i])[:,0]
        traj[2:3,i] = theta_poly.value(t[i])[:,0]
        traj[3:5,i] = left_poly.value(t[i])[:,0]
        traj[5:7,i] = right_poly.value(t[i])[:,0]
        traj[7:8,i] = x_poly.derivative(1).value(t[i])[:,0]
        traj[8:9,i] = y_poly.derivative(1).value(t[i])[:,0]
        traj[9:10,i] = theta_poly.derivative(1).value(t[i])[:,0]
        traj[10:12,i] = left_poly.derivative(1).value(t[i])[:,0]
        traj[12:14,i] = right_poly.derivative(1).value(t[i])[:,0]
        traj[14:16,i] = left_force_poly.value(t[i])[:,0]
        traj[16:18,i] = right_force_poly.value(t[i])[:,0]

        input[0:2,i] = left_poly.derivative(2).value(t[i])[:,0]
        input[2:4,i] = right_poly.derivative(2).value(t[i])[:,0]
        input[4:6,i] = left_force_poly.derivative(1).value(t[i])[:,0]
        input[6:8,i] = right_force_poly.derivative(1).value(t[i])[:,0]

    np.savetxt("/home/brian/Documents/Research/Quals/Traj.csv", traj, delimiter=",")
    np.savetxt("/home/brian/Documents/Research/Quals/Input.csv", input, delimiter="," )
    np.savetxt("/home/brian/Documents/Research/Quals/Time.csv", t, delimiter=",")

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

    n = len(stanceTrajs.GetTrajectoryNames())
    left_stance = sorted(stanceTrajs.GetTrajectoryNames())[0:n//2]
    print(left_stance)
    right_stance = sorted(stanceTrajs.GetTrajectoryNames())[n//2:-1]
    print(right_stance)

    traj_names = ['x', 'y', 'theta']
    stance_names = stanceTrajs.GetTrajectoryNames()
    force_names = sorted(forceTrajs.GetTrajectoryNames())
    print(force_names)

    state_traj_polys = []
    foot_traj_polys = []
    force_traj_polys = []

    n_points = 5001
    for name in traj_names:
        q = stateTrajs.GetTrajectory(name)
        qdot = stateTrajs.GetTrajectory(name + '_dot')
        pp = ReconstructStateTrajectory(q, qdot)

        state_traj_polys.append(pp)

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
        force_traj_polys.append(pp)
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
    foot_traj_polys.append(left_foot_traj)

    for name in right_stance:
        s = stanceTrajs.GetTrajectory(name)
        r_trajs.append(ReconstructStanceTrajectory(s))

    right_foot_traj = ReconstructFootTrajectory(r_trajs)
    foot_traj_polys.append(right_foot_traj)

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


    #plt.show()

    SaveTrajDatapoints(state_traj_polys[0], state_traj_polys[1], state_traj_polys[2],
                       foot_traj_polys[0], foot_traj_polys[1],
                       force_traj_polys[0], force_traj_polys[1])

    plt.show()
if __name__ == "__main__":
    main()
