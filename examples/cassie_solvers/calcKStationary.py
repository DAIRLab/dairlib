# python script for approximating floating base (quaternion) Cassie robot's spring constants using 
# least squares method, assuming robot is stationary

import sys
import numpy as np
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree, AddFlatTerrainToWorld

from lcm_log_utils import parseLcmOutputLog
from contact_toolkit import ContactInfo, ContactToolkit
from cassie_utils import buildCassieTree, ComputeCassieContactInfo

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator, FormatStrFormatter

from scipy.linalg import block_diag
from scipy.sparse.linalg import lsqr


def calcKStationary(filename):

    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1); # flat ground in world

    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, filename, channel, 10e6)
    num_v = tree.get_num_velocities()
    v = np.zeros(num_v)

    C = np.array([])
    Bu = np.array([])
    JctT = np.array([])
    JcT = np.array([])

    ind = 0
    while ind < t.size:

        xCurr = x[:, ind]
        q = xCurr[:tree.get_num_positions()]

        cache = tree.doKinematics(q, v)

        CCurr = tree.dynamicsBiasTerm(cache, {}, True).reshape(num_v, 1) # Coriolis

        BuCurr = tree.B.dot(u[:, ind]).reshape(num_v, 1) # Bu

        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose().reshape(num_v, 12) # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cache, False)
        JcTCurr = JcCurr.transpose().reshape(num_v, 2) # position constraint jacobian transpose

        if ind == 0:
            C = CCurr
            Bu = BuCurr
            JctT = JctTCurr
            JcT = JcTCurr
        else:
            C = np.concatenate((C, CCurr), axis = 0)
            Bu = np.concatenate((Bu, BuCurr), axis = 0)
            JctT = block_diag(JctT, JctTCurr)
            JcT = block_diag(JcT, JcTCurr)
        ind += 1
    
    qMat = np.zeros((22 * t.size, 4))
    
    ind = 0
    while ind < t.size:
        pos = 0;
        while pos < 22:
            if pos == 14:
                qMat[ind * 22 + pos, 0] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index(), ind]
            if pos == 15:
                qMat[ind * 22 + pos, 1] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index(), ind]
            if pos == 18:
                qMat[ind * 22 + pos, 2] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index(), ind]
            if pos == 19:
                qMat[ind * 22 + pos, 3] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index(), ind]
            pos += 1
        ind += 1

    b = C - Bu
    A = np.concatenate((qMat, JctT, JcT), axis = 1)

    X = lsqr(A,b,iter_lim = 1000) # least squares solution using sparse matrix

    print("------------------------------------------------------------------")
    print("Difference between predicted and calculated spring constants are: ")
    print(X[0][:4])
    print("norm(b - Ax) = " + str(X[3]))

def main():

    script = sys.argv[0]
    filename = sys.argv[1]

    calcKStationary(filename)
    
if __name__ == "__main__":
    main()
