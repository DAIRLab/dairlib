# test file for lcm_log_utils in python.

import numpy as np
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree
from lcm_log_utils import parseLcmOutputLog

# sample inputs
def main():
    urdf_path = "/home/zhshen/workspace/dairlib/examples/Cassie/urdf/cassie.urdf"
    tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kQuaternion)
    file = "/home/zhshen/workspace/dairlib/examples/LCM/data/Cassie_walking"
    channel = "CASSIE_STATE"
    t = np.arange(6).reshape((6, 1))
    x = np.zeros(6).reshape((3, 2))
    u = np.zeros(6).reshape((3, 2))
    [t, x, u] = parseLcmOutputLog(tree, file, channel, t, x, u, 10e6)
    print(t)

if __name__ == "__main__":
    main()
