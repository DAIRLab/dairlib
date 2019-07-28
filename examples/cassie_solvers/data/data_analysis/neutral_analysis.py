######################### DO NOT USE - THIS IS INCOMPLETE (SEE README) #########################

import numpy as np
import matplotlib.pyplot as plt
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree, AddFlatTerrainToWorld

from lcm_log_utils import parseLcmOutputLog

from cassie_utils import buildCassieTree, getVelocityName, getPositionName

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator, FormatStrFormatter

def main():
    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kFixed)
    AddFlatTerrainToWorld(tree, 100, 0.1)
    channel = "CASSIE_STATE"

    file1 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/d_rect"
    [t1, x1, u1] = parseLcmOutputLog(tree, file1, channel, 10e6)

    file2 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/m_rect"
    [t2, x2, u2] = parseLcmOutputLog(tree, file2, channel, 10e6)

    file3 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/g_rect"
    [t3, x3, u3] = parseLcmOutputLog(tree, file3, channel, 10e6)

    t1 = np.around(t1, decimals = 3)
    t2 = np.around(t2, decimals = 3)
    t3 = np.around(t3, decimals = 3)

    common_times = np.intersect1d(t1, t2)
    common_times = np.intersect1d(common_times, t3)

    num_q = tree.get_num_positions()
    num_v = tree.get_num_velocities()

    x1_init = x1[:, np.where(t1 == common_times[0])[0][0]]
    x2_init = x2[:, np.where(t2 == common_times[0])[0][0]]
    x3_init = x3[:, np.where(t3 == common_times[0])[0][0]]
    u1_init = u1[:, np.where(t1 == common_times[0])[0][0]]
    u2_init = u2[:, np.where(t2 == common_times[0])[0][0]]
    u3_init = u3[:, np.where(t3 == common_times[0])[0][0]]

    q1 = x1_init[:num_q]
    v1 = x1_init[num_q:]
    q2 = x2_init[:num_q]
    v2 = x2_init[num_q:]
    q3 = x3_init[:num_q]
    v3 = x3_init[num_q:]

    cache_1 = tree.doKinematics(q1, v1)
    cache_2 = tree.doKinematics(q2, v2)
    cache_3 = tree.doKinematics(q3, v3)

    M_1 = tree.massMatrix(cache_1)
    M_2 = tree.massMatrix(cache_2)
    M_3 = tree.massMatrix(cache_3)

    C_1 = tree.inverseDynamics(cache_1, {}, np.zeros(num_v), False)
    #print(C_1)
    #print(q1)
    #print(v1)

    print("drake's mass matrix - mujoco's mass matrix")
    diff = M_1 - M_2
    diff = np.around(diff, decimals = 3)
    print('\n'.join(['\t'.join([str(cell) for cell in row]) for row in diff]))
    print("----------------------------------------------------------------")

    M_1 = np.around(M_1, decimals = 3)
    M_2 = np.around(M_2, decimals = 3)
    print("drake's mass matrix")
    print('\n'.join(['\t'.join([str(cell) for cell in row]) for row in M_1]))
    print("----------------------------------------------------------------")
    print('\n'.join(['\t'.join([str(cell) for cell in row]) for row in M_2]))    


   # print("------")
   # print(M_2)
   # print("------")
   # print(M_3)

if __name__ == "__main__":
    main()
