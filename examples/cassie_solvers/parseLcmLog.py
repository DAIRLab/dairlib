# test file for lcm_log_utils in python.

import sys
import numpy as np
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree
from lcm_log_utils import parseLcmOutputLog

def parseLcmLog(filename):
    urdf_path = "/home/zhshen/workspace/dairlib/examples/Cassie/urdf/cassie.urdf"
    tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kQuaternion)
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, filename, channel, 10e6)

    print(t)
    print(x)
    print(u)

def main():

    script = sys.argv[0]
    filename = sys.argv[1]

    parseLcmLog(filename)

if __name__ == "__main__":
    main()
