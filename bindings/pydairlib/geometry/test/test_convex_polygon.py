import time
import numpy as np
from pydairlib.geometry import ConvexPolygon


def main():
    poly = ConvexPolygon()
    poly.SetPlane(np.array([0, 0, 1]), np.array([0,0,0]))
    for i in [-1, 1]:
        for j in [-1, 1]:
            poly.AddFace(
                np.array([i, j, 0]),
                np.array([10*i, 10*j, 0])
            )
            poly.AddFace(
                np.array([i, j, 0]),
                np.array([10*i, 10*j, 0])
            )
    s = time.time()
    poly.ReduceFaces(4)
    e = time.time()
    print(e - s)
    print(poly.GetConstraintMatrices()[0])
    
    
if __name__ == '__main__':
    main()