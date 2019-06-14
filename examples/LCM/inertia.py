import numpy as np
import matplotlib.pyplot as plt
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree
from lcm_log_utils import parseLcmOutputLog

# sample inputs
def main():
    urdf_path = "/home/zhshen/workspace/dairlib/examples/Cassie/urdf/cassie.urdf"
    tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kQuaternion)
    file = "/home/zhshen/workspace/dairlib/examples/LCM/data/Cassie_walking"
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, file, channel, 10e6)
    q = x[:, 0] # sample time t = 0 used
    cache = tree.doKinematics(q[:23]) # first 23 out of 45 terms correspond to positions
    print(tree.massMatrix(cache))

    # sample plots
    plt.plot(t, x[0, :]) # t vs first position of x
    plt.show()

    plt.plot(t, x[15, :]) # t vs 16th position of x
    plt.show()

    plt.plot(t, u[0, :]) # t vs first position of u
    plt.show()

    plt.plot(x[0, :], u[0, :]) # first position of x vs first position of u
    plt.show()
    
if __name__ == "__main__":
    main()
