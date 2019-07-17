import numpy as np
import matplotlib.pyplot as plt
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree, AddFlatTerrainToWorld

from lcm_log_utils import parseLcmOutputLog
from contact_toolkit import ContactInfo, ContactToolkit
from cassie_utils import buildCassieTree, ComputeCassieContactInfo, getVelocityName

from dircon_position_data import DirconPositionData

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator, FormatStrFormatter

from scipy.linalg import block_diag
from scipy.sparse.linalg import lsqr

from numpy.linalg import matrix_rank

def main():
    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1)
    file = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/Cassie_5"
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, file, channel, 10e6)

    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED
    '''
    t = t[80:93]
    x = x[:, 80:93]
    u = u[:, 80:93]
    '''
    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED

    numQ = tree.get_num_positions()
    numV = tree.get_num_velocities()
    numJ = 14

    deltaK = np.zeros((numV, 4))
    deltaK[14, 0] = 0.0
    deltaK[15, 1] = 0.0
    deltaK[18, 2] = 0.0
    deltaK[19, 3] = 0.0 # difference between non-zero K values in cassie_utils and computed K values from data (ideally 0)

    A = np.array([])

    data_begin = True

    ind = 0
    while ind < t.size:
        xCurr = x[:, ind]
        qCurr = xCurr[:numQ]
        vCurr = xCurr[numQ:]

        cacheCurr = tree.doKinematics(qCurr, vCurr)

        CCurr = tree.dynamicsBiasTerm(cacheCurr, {}, True) # C

        BuCurr = np.matmul(tree.B, u[:, ind]) # Bu

        springsQCurr = [x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index(), ind],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index(), ind],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index(), ind],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index(), ind]]

        KqCurr = np.matmul(deltaK, springsQCurr) # Kq

        MCurr = tree.massMatrix(cacheCurr) # M

        cInfoCurr = ComputeCassieContactInfo(tree, qCurr)
        cToolkitCurr = ContactToolkit(tree, cInfoCurr)
        JctCurr = cToolkitCurr.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cacheCurr, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose
     
        A1Curr = np.concatenate((MCurr, -JctTCurr, -JcTCurr), axis = 1)
        A2Curr = np.concatenate((np.concatenate((JctCurr, JcCurr), axis = 0), np.zeros((14, 14))), axis = 1)
        ACurr = np.concatenate((A1Curr, A2Curr), axis = 0) # A

        ACurr = np.delete(ACurr, [23, 26, 29, 32], axis = 0)
        ACurr = np.delete(ACurr, [23, 26, 29, 32], axis = 1)


        JcDotVCurr = tree.positionConstraintsJacDotTimesV(cacheCurr) # JcDot

        dpd_toe_left1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cacheCurr)
        dpd_toe_left2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cacheCurr)
        dpd_toe_right1 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cacheCurr)
        dpd_toe_right2 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cacheCurr)

        JctDotVCurr = np.concatenate((dpd_toe_left1, dpd_toe_left2, dpd_toe_right1, dpd_toe_right2), axis = 0)

        bCurr = np.concatenate((KqCurr + BuCurr - CCurr, -JctDotVCurr, -JcDotVCurr), axis = 0) # b

        bCurr = np.delete(bCurr, [23, 26, 29, 32])

        if data_begin == True:
            A = ACurr
            b = bCurr
            data_begin = False
        else:
            A = block_diag(A, ACurr)
            b = np.concatenate((b, bCurr), axis = 0)
        ind += 1

    X = lsqr(A, b, iter_lim = 1000)
    sol = X[0] # least squares solution using sparse matrix

    print("--------------------------- PLOTTING ACCELERATIONS ---------------------------")

    aMatrix = np.zeros((numV, t.size)) # matrix of accelerations from this function
    i = 0
    while i < t.size:
        j = 0
        while j < numV:
            aMatrix[j, i] = sol[32 * i + j]
            j += 1
        i += 1
    print(aMatrix)

    a = np.zeros((numV, t.size - 1)) # discard last data set eventually to accomodate for acceleration calculation

    ind = 0
    while ind < numV:
        time = 0
        while time < t.size - 1:
            a[ind, time] = (x[numQ + ind, time + 1] - x[numQ + ind, time]) / (t[time + 1] - t[time])
            time += 1
        ind += 1

    ind = 0
    while ind < numV:
        plt.title("Plot of acceleration of " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot((t[:t.size - 1]), aMatrix[ind,:t.size - 1], 'r', (t[:t.size - 1]), a[ind,:], 'g') # calculated accel vs approx accel of position against t
        plt.show()
        ind += 1

    # add test

if __name__ == "__main__":
    main()
