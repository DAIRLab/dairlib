import numpy as np

from pydairlib.geometry import ConvexFoothold
from pydrake.solvers import MathematicalProgram, Solve


def find_foothold_to_point_distance(footholds, query_point):
    prog = MathematicalProgram()
    p = [prog.NewContinuousVariables(3) for f in footholds]

    for i, foothold in enumerate(footholds):
        Aeq, beq = foothold.GetEqualityConstraintMatrices()
        A, b = foothold.GetConstraintMatrices()
        prog.AddLinearEqualityConstraint(Aeq, beq, p[i])
        prog.AddLinearConstraint(A, lb=-np.inf*np.ones_like(b), ub=b, vars=p[i])
        prog.AddQuadraticErrorCost(np.eye(3), query_point, p[i])

    result = Solve(prog)
    import pdb; pdb.set_trace()


def main():
    footholds = []
    for o in [
        [0.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0]
    ]:
        # Make a rhombic foothold
        foothold = ConvexFoothold()
        foothold.SetContactPlane(np.array([0, 0, 1]), np.array([0, 0, 0]))
        for i in [-1, 1]:
            for j in [-1, 1]:
                foothold.AddFace(
                    np.array([i, j, 0]),
                    np.array(o) + np.array([0.25*i, 0.25*j, 0])
                )
        footholds.append(foothold)

    find_foothold_to_point_distance(footholds, np.zeros((3,)))

if __name__ == main():
    main()