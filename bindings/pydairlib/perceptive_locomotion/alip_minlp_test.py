import numpy as np
from matplotlib import pyplot as plt

from pydairlib.geometry.convex_foothold import ConvexFoothold
from pydairlib.perceptive_locomotion.controllers import AlipMINLP


def PlotCoMTrajSolution(trajopt):
    xx = trajopt.GetStateSolution()
    pp = trajopt.GetFootstepSolution()
    xy_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][k][:2] + pp[n][:2], axis=-1) for k in range(trajopt.nknots()[n])]
            ) for n in range(trajopt.nmodes())
        ]
    )
    plt.plot(-xy_traj[1], xy_traj[0])
    for i in range(trajopt.nmodes()):
        for k in range(trajopt.nknots()[i]):
            xy = xx[i][k].ravel()[:2] + pp[i][:2]
            line_x = [-xy[1], -pp[i][1]]
            line_y = [xy[0], pp[i][0]]
            plt.plot(line_x, line_y, color='black')
    plt.xlim([-0.5, 0.5])
    plt.ylim([-0.5, pp[-1][0] + xx[-1][-1][0]])


def PlotInputSolution(trajopt):
    plt.figure()
    u = trajopt.GetInputSolution()
    t = trajopt.GetTimingSolution()
    tcum = np.concatenate(([0], np.cumsum(t)))
    tknots = np.concatenate([
        np.linspace(tcum[i], tcum[i + 1], trajopt._nknots[i])[:-1] for i in range(len(t))
    ])
    plt.plot(tknots, np.hstack(u).T)
    plt.legend(['ankle torque'])


def main():
    trajopt = AlipMINLP(32, 0.9)

    footholds = []
    p0 = [0.0, 0, 0]
    for o in [
        p0,
        np.array([0.0, -0.1, 0.0])
    ]:
        # Make a rhombic foothold
        foothold = ConvexFoothold()
        foothold.SetContactPlane(np.array([0, 0, 1]), np.array([0, 0, 0]))
        for i in [-1, 1]:
            for j in [-1, 1]:
                foothold.AddFace(
                    np.array([i, j, 0]),
                    np.array(o) + np.array([10*i, 10*j, 0])
                )
        footholds.append(foothold)
    trajopt.AddFootholds(footholds)
    nk = 10
    trajopt.AddMode(nk)
    trajopt.AddMode(nk)
    trajopt.AddMode(nk)
    xd = trajopt.MakeXdesTrajForVdes(np.array([0.5, 0]), 0.3, 0.35, nk, -1)
    trajopt.AddTrackingCost(xd, 1*np.eye(4))
    trajopt.SetNominalStanceTime(0.35, 0.35)
    trajopt.SetMinimumStanceTime(0.1)
    trajopt.AddInputCost(0.1)
    trajopt.Build()
    # trajopt.ActivateInitialTimeConstraint(0.35)
    trajopt.CalcOptimalFootstepPlan(xd[0][0], np.array(p0), False)
    print(trajopt.GetTimingSolution())
    PlotCoMTrajSolution(trajopt)
    plt.show()
    # import pdb; pdb.set_trace()


if __name__ == "__main__":
    main()
