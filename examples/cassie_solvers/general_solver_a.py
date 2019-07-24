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

        JcDotVCurr = tree.positionConstraintsJacDotTimesV(cacheCurr) # JcDot

        dpd_toe_left1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cacheCurr)
        dpd_toe_left2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cacheCurr)
        dpd_toe_right1 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cacheCurr)
        dpd_toe_right2 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cacheCurr)

        JctDotVCurr = np.concatenate((dpd_toe_left1, dpd_toe_left2, dpd_toe_right1, dpd_toe_right2), axis = 0)
        # always very small (JcDotVCurr)
        # always very small (JctDotVCurr)
        # always very small (JcCurr)
        # always very small (JctCurr)

        bCurr = np.concatenate((KqCurr + BuCurr - CCurr, -JctDotVCurr, -JcDotVCurr), axis = 0) # b

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

    print("--------------------------- CHECKING ---------------------------")

    aMatrix = np.zeros((numV, t.size)) # matrix of accelerations from this function
    i = 0
    while i < t.size:
        j = 0
        while j < numV:
            aMatrix[j, i] = sol[36 * i + j]
            j += 1
        i += 1
    print(aMatrix)

    lambdaMatrix = np.zeros((14, t.size)) # matrix of lambdas from this function
    i = 0
    while i < t.size:
        j = 0
        while j < 14:
            lambdaMatrix[j, i] = sol[22 + 36 * i + j]
            j += 1
        i += 1

    # compare result with general_solver_K's acceleration data

    a = np.zeros((numV, t.size - 1)) # discard last data set eventually to accomodate for acceleration calculation

    ind = 0
    while ind < numV:
        time = 0
        while time < t.size - 1:
            a[ind, time] = (x[numQ + ind, time + 1] - x[numQ + ind, time]) / (t[time + 1] - t[time])
            time += 1
        ind += 1

    print("--------------------------- PLOTTING ACCELERATIONS WITH CONTACT FORCES---------------------------")
    
    ind = 0
    while ind < numV:
        plt.title("Plot of acceleration of " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot((t[:t.size - 1]), aMatrix[ind,:t.size - 1], 'r', label = 'matrix solution')
        plt.plot((t[:t.size - 1]), a[ind,:], 'g', label = 'limit solution') # calculated accel vs approx accel of position against t
        plt.legend()
        plt.show()
        ind += 1
    
    '''

    Jv = np.zeros((12, 1))
    time = 0
    while time < t.size:
        qCurr = x[:23, time]
        cInfoCurr = ComputeCassieContactInfo(tree, qCurr)
        cToolkitCurr = ContactToolkit(tree, cInfoCurr)
        JctCurr = cToolkitCurr.CalcContactJacobian(xCurr, False)
        JvCurr = np.matmul(JctCurr, x[23:, time]).reshape(12, 1)
        print(Jv.shape)
        print(JvCurr.shape)
        Jv = np.concatenate((Jv, JvCurr), axis = 1)
        time += 1
    Jv = Jv[:,1:]
    ind = 0
    while ind < 12:
        plt.title("Plot of acceleration of position 11 vs +/- Jv")
        plt.plot(t, aMatrix[11,:], 'g+') # calculated accel vs approx accel of Jv against t
        plt.plot(t, Jv[ind,:])
        ind += 1
    plt.show()
    '''

    print("--------------------------------- t = 0 ------------------------------------")

    x0 = x[:, 0]
    q0 = x0[:numQ]
    v0 = x0[numQ:]

    lambda0 = [84.69107, 0.36542079, -2.88779, 87.37902, 0.54163, 1.571763, 79.42712, 1.06329, 3.43513, 104.7781, 1.0549, -3.465996, -455.95446, -469.77942379]

    cache = tree.doKinematics(q0, v0) # 22 positions

    CCurr = tree.dynamicsBiasTerm(cache, {}, True)

    BuCurr = np.matmul(tree.B, (u[:, 0]))

    MCurr = tree.massMatrix(cache)
    MaCurr = np. matmul(MCurr, a[:, 0])

    springsQCurr = [x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index(), 0],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index(), ind],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index(), 0],
                    x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index(), 0]]

    KqCurr = np.matmul(deltaK, springsQCurr) # Kq

    cInfo = ComputeCassieContactInfo(tree, q0)
    cToolkit = ContactToolkit(tree, cInfo)
    JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
    JctTCurr = JctCurr.transpose() # contact jacobian transpose

    JcCurr = tree.positionConstraintsJacobian(cache, False)
    JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

    C = CCurr
    Bu = BuCurr
    JT = np.concatenate((JctTCurr, JcTCurr), axis = 1)
    JTlambda = np.matmul(JT, lambda0)
    Ma = MaCurr

    JcDotVCurr = tree.positionConstraintsJacDotTimesV(cache) # JcDot

    dpd_toe_left1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cache)
    dpd_toe_left2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cache)
    dpd_toe_right1 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cache)
    dpd_toe_right2 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cache)

    JctDotVCurr = np.concatenate((dpd_toe_left1, dpd_toe_left2, dpd_toe_right1, dpd_toe_right2), axis = 0)
    
    JDotVCurr = np.concatenate((JctDotVCurr, JcDotVCurr), axis = 0)

    print(Ma + C - JTlambda - Bu - KqCurr)

    J = np.concatenate((JctCurr, JcCurr), axis = 0)

    print(np.matmul(J, a[:, 11]) - JDotVCurr)

    
    print("-------------------------- Solve backwards to find delta K's -------------------------")

    data_begin = True
    C = np.array([])
    Bu = np.array([])
    Ma = np.array([])
    JctT = np.array([])
    JcT = np.array([])

    ind = 0
    while ind < t.size - 1:

        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cache = tree.doKinematics(q, v) # 22 positions

        CCurr = tree.dynamicsBiasTerm(cache, {}, True)

        BuCurr = np.matmul(tree.B, (u[:, ind]))

        MCurr = tree.massMatrix(cache)
        MaCurr = np. matmul(MCurr, aMatrix[:, ind])

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
    print(X[1]) # success = 2
    print(X[3]) # norm(b - Ax)
    '''
    print("--------------------------- TESTING Jdotv + Ja ---------------------------")
    
    # using general_solver_a's accels

    xinit = x[:, 0]
    qinit = xinit[:numQ]
    vinit = xinit[numQ:]
    cacheinit = tree.doKinematics(qinit, vinit)
    cInfoinit = ComputeCassieContactInfo(tree, qinit)
    cToolkitinit = ContactToolkit(tree, cInfoinit)

    Jainit = np.matmul(np.concatenate((cToolkitinit.CalcContactJacobian(xinit, False), tree.positionConstraintsJacobian(cacheinit, False)), axis = 0), aMatrix[:, 0])

    dpd1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cacheinit)
    dpd2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cacheinit)
    dpd3 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cacheinit)
    dpd4 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cacheinit)

    JctDotinit = np.concatenate((dpd1, dpd2, dpd3, dpd4), axis = 0)
    JcDotinit = tree.positionConstraintsJacDotTimesV(cacheinit)

    Jdotvinit = np.concatenate((JctDotinit, JcDotinit), axis = 0)

    y = (Jainit + Jdotvinit).reshape(14, 1)

    ind = 0
    while ind < t.size - 1:

        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]
        cache = tree.doKinematics(q, v)
        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        Ja = np.matmul(np.concatenate((cToolkit.CalcContactJacobian(xCurr, False), tree.positionConstraintsJacobian(cache, False)), axis = 0), aMatrix[:, ind])

        dpd1 = DirconPositionData(tree, 18, [-0.0457, 0.112, 0], False).Jdotv(cache)
        dpd2 = DirconPositionData(tree, 18, [0.088, 0, 0], False).Jdotv(cache)
        dpd3 = DirconPositionData(tree, 20, [-0.0457, 0.112, 0], False).Jdotv(cache)
        dpd4 = DirconPositionData(tree, 20, [0.088, 0, 0], False).Jdotv(cache)

        JctDot = np.concatenate((dpd1, dpd2, dpd3, dpd4), axis = 0)
        JcDot = tree.positionConstraintsJacDotTimesV(cache)

        Jdotv = np.concatenate((JctDot, JcDot), axis = 0)
        y = np.concatenate((y, (Ja + Jdotv).reshape(numJ, 1)), axis = 1)
        ind += 1

    # using general_solver_K's accels

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

    y2 = (Jainit + Jdotvinit).reshape(numJ, 1)

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
        y2 = np.concatenate((y2, (Ja + Jdotv).reshape(numJ, 1)), axis = 1)
        ind += 1

    ind = 0
    while ind < numJ:
        plt.title("Plot of Jdotv + Ja residuals " + str(ind) + " (Position)")
        plt.plot(t[:t.size - 1], y[ind, :t.size - 1], 'r', t[:t.size - 1], y2[ind, :t.size - 1], 'g')
        plt.ylim(-5, 5)
        plt.show()
        ind += 1

    print("--------------------------- TESTING Correct accels in matrix equation ---------------------------")
    '''
    # add test

if __name__ == "__main__":
    main()
