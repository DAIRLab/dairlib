import numpy as np

from pydairlib.perceptive_locomotion.controllers import \
    MakeMinSnapTrajFromWaypoints


def test():
    t = [0, 0.5, 1.0]
    wp = np.array([[0, 0, -0.85], [0, 0, -0.79], [0, 0, -0.85]])
    traj = MakeMinSnapTrajFromWaypoints(wp, t)
    import pdb; pdb.set_trace()


if __name__ == "__main__":
    test()
