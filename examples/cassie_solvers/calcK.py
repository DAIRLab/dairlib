# python script for approximating floating base (quaternion) Cassie robot's spring constants using 
# least squares method

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

from dircon_position_data import DirconPositionData

def calcK(filename):
    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1) # flat ground in world
    channel = "CASSIE_STATE"

    [t, x, u] = parseLcmOutputLog(tree, filename, channel, 10e6)

    numQ = tree.get_num_positions()
    numV = tree.get_num_velocities()
    numJ = 14

    C = np.array([])
    Bu = np.array([])
    Ma = np.array([])
    JctT = np.array([])
    JcT = np.array([])

    a = np.zeros((numV, t.size - 1)) # discard last data set eventually to accomodate for acceleration calculation
    
    ind = 0
    while ind < numV:
        time = 0
        while time < t.size - 1:
            a[ind, time] = (x[numQ + ind, time + 1] - x[numQ + ind, time]) / (t[time + 1] - t[time])
            time += 1
        ind += 1

    ind = 0
    while ind < t.size - 1:

        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cache = tree.doKinematics(q, v) # 22 positions

        CCurr = tree.dynamicsBiasTerm(cache, {}, True)

        BuCurr = np.matmul(tree.B, (u[:, ind]))

        MCurr = tree.massMatrix(cache)
        MaCurr = np. matmul(MCurr, a[:, ind])

        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cache, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

        if ind == 0:
            C = CCurr
            Bu = BuCurr
            JctT = JctTCurr
            JcT = JcTCurr
            Ma = MaCurr
        else:
            C = np.concatenate((C, CCurr), axis = 0)
            Bu = np.concatenate((Bu, BuCurr), axis = 0)
            JctT = block_diag(JctT, JctTCurr)
            JcT = block_diag(JcT, JcTCurr)
            Ma = np.concatenate((Ma, MaCurr), axis = 0)

        ind += 1
    
    qMat = np.zeros((numV * (t.size - 1), 4))
    ind = 0
    while ind < t.size - 1:
        pos = 0;
        while pos < numV:
            if pos == 14: # knee_spring_left
                qMat[ind * numV + pos, 0] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index(), ind]
            if pos == 15: # knee_spring_right
                qMat[ind * numV + pos, 1] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index(), ind]
            if pos == 18: # heel_spring_left
                qMat[ind * numV + pos, 2] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index(), ind]
            if pos == 19: # heel_spring_right
                qMat[ind * numV + pos, 3] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index(), ind]
            pos += 1
        ind += 1

    b = C - Bu + Ma
    A = np.concatenate((qMat, JctT, JcT), axis = 1)

    X = lsqr(A, b, iter_lim = 1000) # least squares solution using sparse matrix

    print("------------------------------------------------------------------")
    print("Difference between predicted and calculated spring constants are: ")
    print(X[0][:4])
    print("norm(b - Ax) = " + str(X[3]))

def main():

    script = sys.argv[0]
    filename = sys.argv[1]

    calcK(filename)
    
if __name__ == "__main__":
    main()
