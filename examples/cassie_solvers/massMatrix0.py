# test script for finding mass matrix of floating base (quaternion) Casssie robot at t = 0

import sys
import numpy as np
import matplotlib.pyplot as plt
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree
from lcm_log_utils import parseLcmOutputLog

# sample inputs
def massMatrix(filename):
    urdf_path = "/home/zhshen/workspace/dairlib/examples/Cassie/urdf/cassie.urdf"
    tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kQuaternion)
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, filename, channel, 10e6)
    q = x[:, 0] # sample time t = 0 used
    cache = tree.doKinematics(q[:tree.get_num_positions()])
    print(tree.massMatrix(cache))

    '''
    # sample plots
    plt.plot(t, x[0, :]) # t vs first position of x
    plt.show()

    plt.plot(t, x[15, :]) # t vs 16th position of x
    plt.show()

    plt.plot(t, u[0, :]) # t vs first position of u
    plt.show()

    plt.plot(x[0, :], u[0, :]) # first position of x vs first position of u
    plt.show()
    '''

def main():

    script = sys.argv[0]
    filename = sys.argv[1]

    massMatrix(filename)
    
if __name__ == "__main__":

    main()
