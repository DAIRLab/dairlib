import numpy as np
import matplotlib.pyplot as plt
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

def main():
    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1)
    file = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/Cassie_5"
    channel = "CASSIE_STATE"

    [t, x, u] = parseLcmOutputLog(tree, file, channel, 10e6)

    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED
    #t = t[0:6]
    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED

    numQ = tree.get_num_positions()
    numV = tree.get_num_velocities()
    numJ = 14

    data_begin = True
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

        if data_begin == True:
            C = CCurr
            Bu = BuCurr
            JctT = JctTCurr
            JcT = JcTCurr
            Ma = MaCurr
            data_begin = False
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

    print(X[0][:4]) # first 4 correspond to spring constants
    sol = X[0] # least squares solution using sparse matrix
    print(sol[:4])
    print(X[3]) # norm(b - Ax)
    lambdaMatrix = np.zeros((12, t.size - 1))
    i = 0
    while i < t.size - 1:
        j = 0
        while j < 12:
            lambdaMatrix[j, i] = sol[4 + 12 * i + j]
            j += 1
        i += 1
    
    print("---------------------------TESTING---------------------------")

    xinit = x[:, 0]
    qinit = xinit[:numQ]
    vinit = xinit[numQ:]
    cacheinit = tree.doKinematics(qinit, vinit)
    cInfoinit = ComputeCassieContactInfo(tree, qinit)
    cToolkitinit = ContactToolkit(tree, cInfoinit)

    Jainit = np.matmul(np.concatenate((cToolkitinit.CalcContactJacobian(xinit, False), tree.positionConstraintsJacobian(cacheinit, False)), axis = 0), a[:, 0])

    dpd1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cacheinit)
    dpd2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cacheinit)
    dpd3 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cacheinit)
    dpd4 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cacheinit)

    JctDotinit = np.concatenate((dpd1, dpd2, dpd3, dpd4), axis = 0)
    JcDotinit = tree.positionConstraintsJacDotTimesV(cacheinit)

    Jdotvinit = np.concatenate((JctDotinit, JcDotinit), axis = 0)

    y = (Jainit + Jdotvinit).reshape(numJ, 1)

    ind = 0
    while ind < t.size - 1:

        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]
        cache = tree.doKinematics(q, v)
        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        Ja = np.matmul(np.concatenate((cToolkit.CalcContactJacobian(xCurr, False), tree.positionConstraintsJacobian(cache, False)), axis = 0), a[:, ind])

        dpd1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cache)
        dpd2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cache)
        dpd3 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cache)
        dpd4 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cache)

        JctDot = np.concatenate((dpd1, dpd2, dpd3, dpd4), axis = 0)
        JcDot = tree.positionConstraintsJacDotTimesV(cache)

        Jdotv = np.concatenate((JctDot, JcDot), axis = 0)
        y = np.concatenate((y, (Ja + Jdotv).reshape(numJ, 1)), axis = 1)
        ind += 1

    '''
    print("--------------------------- PLOTTING ACCELERATIONS ---------------------------")
    print(a[21,:])
    ind = 0
    while ind < numV:
        plt.title("Plot of acceleration of " + str(ind))
        plt.plot(t[:t.size - 1], a[ind,:], 'g') # calculated accel vs approx accel of position against t
        plt.ylim(-5, 5)
        plt.show()
        ind += 1
    '''
    '''
    ind = 0
    while ind < numJ:
        plt.title("Plot of " + str(ind))
        plt.plot(t[:t.size - 1], y[ind, :t.size - 1])
        plt.ylim(-5, 5)
        plt.show()
        ind += 1

    print("---------------------------PLOTTING TESTS---------------------------")

    ind = 0
    while ind < 22:
        plt.title("Plot of " + str(ind) + " vs t (t.size = " + str(t.size) + ")")
        plt.plot(t, x[ind + 23,:]) # v vs t
        plt.show()
        ind += 1

    print("---------------------- Testing ----------------------")
    print(X[0])
    print(X[1]) # 2 means problem solved
    print(X[2]) # iteration number when terminated
    print(X[3]) # norm(b - Ax)
    print("---------------------- Testing ----------------------")
    '''
if __name__ == "__main__":
    main()

