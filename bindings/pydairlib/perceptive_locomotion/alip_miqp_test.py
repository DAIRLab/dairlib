import numpy as np
from matplotlib import pyplot as plt

from pydrake.systems.all import Diagram

from pydairlib.geometry.convex_polygon import ConvexPolygon
from pydairlib.systems.footstep_planning import (
    AlipMultiQP,
    AlipMIQP,
    Stance,
    ResetDiscretization
)

"""
    This file contains a small python demo of the model predictive footstep
     controller from  "Bipedal Walking on Constrained Footholds with 
     MPC Footstep Control" (https://arxiv.org/abs/2309.07993)
"""


def plot_com_traj_solution(trajopt):
    """
        Plots an overhead view of the CoM trajectory
        :param trajopt: an ALipMIQP or AlipMultiQP
        :return: nothing
    """
    xx = trajopt.GetStateSolution()
    pp = trajopt.GetFootstepSolution()
    xy_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][4 * k:4 * k + 2] + pp[n][:2], axis=-1) for k in range(trajopt.nknots())]
            ) for n in range(trajopt.nmodes())
        ]
    )
    plt.plot(-xy_traj[1], xy_traj[0])
    for i in range(trajopt.nmodes()):
        for k in range(trajopt.nknots()):
            xy = xx[i][4 * k:4 * k + 2].ravel()[:2] + pp[i][:2]
            line_x = [-xy[1], -pp[i][1]]
            line_y = [xy[0], pp[i][0]]
            plt.plot(line_x, line_y, color='black')
    plt.xlim([-0.5, 0.5])
    plt.ylim([pp[0][0] + xx[0][0] - 0.05, pp[-1][0] + xx[-1][-4] + 0.05])


def plot_desired(xx):
    xy_traj = np.hstack(
        [
            np.hstack(
                [np.expand_dims(xx[n][4 * k:4 * k + 2], axis=-1) for k in
                 range(int(xx[0].size / 4))]
            ) for n in range(len(xx))
        ]
    )
    plt.plot(-xy_traj[1], xy_traj[0])
    plt.xlim([-0.5, 0.5])


def plot_input_solution(trajopt):
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
    # AlipMultiQP is always available, and solves the MIQP by solving a QP
    # for each possible foothold sequence. AlipMIQP uses Gurobi's mixed integer
    # solver, which requires building against gurobi.
    trajopt1 = AlipMultiQP(32, 0.9, 10, ResetDiscretization.kFOH, 3)
    trajopt2 = AlipMIQP(32, 0.9, 10, ResetDiscretization.kFOH, 3)

    for trajopt in [trajopt1, trajopt2]:
        trajopt.SetDoubleSupportTime(0.05)

        footholds = []
        p0 = [0.0, 0.0, 0.0]
        for o in [
            p0,
        ]:
            # Make a rhombic foothold
            foothold = ConvexPolygon()
            foothold.SetPlane(np.array([0, 0, 1]), np.array([0, 0, 0]))
            for i in [-1, 1]:
                for j in [-1, 1]:
                    foothold.AddFace(
                        np.array([i, j, 0]),
                        np.array(o) + np.array([10*i, 10*j, 0])
                    )
            footholds.append(foothold)
        trajopt.AddFootholds(footholds)

        xd = trajopt.MakeXdesTrajForVdes(
            np.array([[0.2], [0.0]]),
            0.35,
            0.35,
            10,
            Stance.kLeft
        )

        trajopt.UpdateNominalStanceTime(0.35, 0.35)
        trajopt.AddTrackingCost(xd, 1*np.eye(4), 0*np.eye(4))
        trajopt.SetMinimumStanceTime(0.1)
        trajopt.SetMaximumStanceTime(0.35)
        trajopt.SetInputLimit(1)
        trajopt.AddInputCost(10)
        trajopt.Build()

        # Solve the trajectory optimzation demo
        trajopt.CalcOptimalFootstepPlan(xd[0][:4], np.array(p0), False)

        plot_com_traj_solution(trajopt)
    plt.show()
    # import pdb; pdb.set_trace()


if __name__ == "__main__":
    main()
