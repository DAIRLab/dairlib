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
    t = t[78:93]
    x = x[:, 78:93]
    u = u[:, 78:93]
    '''
    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED

    numQ = tree.get_num_positions()
    numV = tree.get_num_velocities()
    numJ = 14

    print("------------------------------- acceleration using limits ------------------------------") ##################################### LIMITS

    aLim = np.zeros((numV, t.size - 1)) # discard last data set eventually to accomodate for acceleration calculation
    
    ind = 0
    while ind < numV:
        time = 0
        while time < t.size - 1:
            aLim[ind, time] = (x[numQ + ind, time + 1] - x[numQ + ind, time]) / (t[time + 1] - t[time])
            time += 1
        ind += 1
    
    print("----------------------------- solve for acceleration and lambda -------------------------") ##################################### GENERAL_SOLVER_A

    deltaK = np.zeros((numV, 4))
    deltaK[14, 0] = 0.0
    deltaK[15, 1] = 0.0
    deltaK[18, 2] = 0.0
    deltaK[19, 3] = 0.0 # difference between non-zero K values in cassie_utils and computed K values from data (ideally 0)

    A_a = np.array([])
    JDotV = np.array([])

    data_begin_a = True
    ind_a = 0
    while ind_a < t.size - 1:
        xCurr = x[:, ind_a]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cacheCurr = tree.doKinematics(q, v)

        CCurr = tree.dynamicsBiasTerm(cacheCurr, {}, True) # C

        BuCurr = np.matmul(tree.B, u[:, ind_a]) # Bu

        Kq_a = np.zeros(22)

        MCurr = tree.massMatrix(cacheCurr) # M

        cInfoCurr = ComputeCassieContactInfo(tree, q)
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

        bCurr = np.concatenate((Kq_a + BuCurr - CCurr, -JctDotVCurr, -JcDotVCurr), axis = 0) # b

        if data_begin_a == True:
            A_a = ACurr
            b_a = bCurr
            JDotV = np.concatenate((JctDotVCurr.reshape(12, 1), JcDotVCurr.reshape(2, 1)), axis = 0)
            data_begin_a = False
        else:
            A_a = block_diag(A_a, ACurr)
            b_a = np.concatenate((b_a, bCurr), axis = 0)
            JDotV = np.concatenate((JDotV, np.concatenate((JctDotVCurr.reshape(12, 1), JcDotVCurr.reshape(2, 1)), axis = 0)), axis = 1)
        ind_a += 1

    X_a = lsqr(A_a, b_a, iter_lim = 1000)
    sol_a = X_a[0] # least squares solution using sparse matrix

    #### SAVING A_DATA (DISCARD LAST SET FOR CONSISTENCY) ####

    a_a = np.zeros((22, t.size - 1)) # saving accelerations
    i = 0
    while i < t.size - 1:
        j = 0
        while j < 22:
            a_a[j, i] = sol_a[j + 36 * i]
            j += 1
        i += 1

    a_lambda = np.zeros((14, t.size - 1)) # saving lambdas
    i = 0
    while i < t.size - 1:
        j = 0
        while j < 14:
            a_lambda[j, i] = sol_a[22 + j + 36 * i]
            j += 1
        i += 1

    a_JDotV = JDotV # saving JDotV

    print("------------------------------ plotting ----------------------------------")
    '''
    ind = 0
    while ind < numV:
        plt.title("Plot of acceleration of " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot((t[:t.size - 1]), a_a[ind,:], 'r', (t[:t.size - 1]), aLim[ind,:], 'g') # calculated accel vs approx accel of position against t
        plt.ylim(-4, 4)
        plt.show()
        ind += 1
    '''
    print("----------------------------- solve for K and lambda -------------------------") ##################################### GENERAL_SOLVER_K

    data_begin_K = True
    C = np.array([])
    Bu = np.array([])
    Ma = np.array([])
    JctT = np.array([])
    JcT = np.array([])

    ind_K = 0
    while ind_K < t.size - 1:

        xCurr = x[:, ind_K]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cache = tree.doKinematics(q, v) # 22 positions

        CCurr = tree.dynamicsBiasTerm(cache, {}, True)

        BuCurr = np.matmul(tree.B, (u[:, ind_K]))

        MCurr = tree.massMatrix(cache)
        MaCurr = np.matmul(MCurr, aLim[:, ind_K])

        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cache, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

        if data_begin_K == True:
            C = CCurr
            Bu = BuCurr
            JctT = JctTCurr
            JcT = JcTCurr
            Ma = MaCurr
            data_begin_K = False
        else:
            C = np.concatenate((C, CCurr), axis = 0)
            Bu = np.concatenate((Bu, BuCurr), axis = 0)
            JctT = block_diag(JctT, JctTCurr)
            JcT = block_diag(JcT, JcTCurr)
            Ma = np.concatenate((Ma, MaCurr), axis = 0)

        ind_K += 1
    
    qMat_K = np.zeros((numV * (t.size - 1), 4))
    ind_K = 0
    while ind_K < t.size - 1:
        pos = 0;
        while pos < numV:
            if pos == 14: # knee_spring_left
                qMat_K[ind_K * numV + pos, 0] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index(), ind_K]
            if pos == 15: # knee_spring_right
                qMat_K[ind_K * numV + pos, 1] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index(), ind_K]
            if pos == 18: # heel_spring_left
                qMat_K[ind_K * numV + pos, 2] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index(), ind_K]
            if pos == 19: # heel_spring_right
                qMat_K[ind_K * numV + pos, 3] = x[tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index(), ind_K]
            pos += 1
        ind_K += 1

    b_K = C - Bu + Ma
    A_K = np.concatenate((qMat_K, JctT, JcT), axis = 1)

    X_K = lsqr(A_K, b_K, iter_lim = 1000) # least squares solution using sparse matrix
    sol_K = X_K[0] # least squares solution using sparse matrix

    #### SAVING K_DATA (DISCARD LAST SET FOR CONSISTENCY) ####

    K_a = aLim # saving accelerations

    K_K = sol_K[:4] # saving spring constants

    K_Jct = np.zeros((12, t.size - 1))
    i = 0
    while i < t.size - 1:
        j = 0
        while j < 12:
            K_Jct[j, i] = sol_K[4 + j + 12 * i]
            j += 1
        i += 1
    
    K_Jc = np.zeros((2, t.size - 1))
    i = 0
    while i < t.size - 1:
        j = 0
        while j < 2:
            K_Jc[j, i] = sol_K[(4 + 12 * (t.size - 1)) + j + 2 * i]
            j += 1
        i += 1

    K_lambda = np.concatenate((K_Jct, K_Jc), axis = 0) # saving lambdas

    print("--------------------------------- COMPARE ------------------------------------") ##################################### DIFFERENCE

    #### comparing lambdas ####
    
    delta_lambda = np.divide(np.absolute(a_lambda - K_lambda), np.absolute(K_lambda))

    ind = 0
    while ind < 14:
        plt.title("Plot of " + str(ind) + " delta_lambda of 2 methods vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], delta_lambda[ind, :])
        plt.show()
        ind += 1
    
    print(a_lambda[:, 0]) # all lambdas at t = 0
    print(K_lambda[:, 0])
    print(np.divide(delta_lambda[:, 0], a_lambda[:, 0]))
    print(np.divide(delta_lambda[:, 0], K_lambda[:, 0]))
    
    #### plotting lambdas ####
    
    ind = 0
    while ind < 14:
        plt.title("Plot of " + str(ind) + " lambdas of 2 methods vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], a_lambda[ind, :], 'r', t[:t.size - 1], K_lambda[ind, :], 'g')
        plt.show()
        ind += 1
    
    #### comparing JTlambdas ####

    x0 = x[:, 0]
    q0 = x0[:numQ]
    v0 = x0[numQ:]

    cache0 = tree.doKinematics(q0, v0)

    cInfo0 = ComputeCassieContactInfo(tree, q0)
    cToolkit0 = ContactToolkit(tree, cInfo0)
    Jct0 = cToolkit0.CalcContactJacobian(x0, False)
    JctT0 = Jct0.transpose() # contact jacobian transpose
    
    Jc0 = tree.positionConstraintsJacobian(cache0, False)
    JcT0 = Jc0.transpose() # position constraint jacobian transpose

    a_JTL = np.matmul(JctT0, a_lambda[:12, 0]) + np.matmul(JcT0, a_lambda[12:14, 0])
    K_JTL = np.matmul(JctT0, K_lambda[:12, 0]) + np.matmul(JcT0, K_lambda[12:14, 0])

    a_JTL = np.matmul(np.concatenate((JctT0, JcT0), axis = 1), a_lambda[:, 0])
    K_JTL = np.matmul(np.concatenate((JctT0, JcT0), axis = 1), K_lambda[:, 0])

    '''
    ind = 0
    while ind < 14:
        print(np.absolute(a_JTL - K_JTL)[ind])
        ind += 1
    '''

    print("---------------------------- Seeing if first equations match ------------------------------")

    # a_method - solve for Kq (ignore the C and Bu terms)

    RHS_a = np.zeros(22)

    data_begin_a = True
    ind_a = 0
    while ind_a < t.size - 1:
        xCurr = x[:, ind_a]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cacheCurr = tree.doKinematics(q, v)
        MCurr = tree.massMatrix(cacheCurr) # M
        if ind_a < 1:
            print(MCurr)

        cInfoCurr = ComputeCassieContactInfo(tree, q)
        cToolkitCurr = ContactToolkit(tree, cInfoCurr)
        JctCurr = cToolkitCurr.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cacheCurr, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

        JTCurr = np.concatenate((JctTCurr, JcTCurr), axis = 1)

        RHSCurr = (np.matmul(MCurr, a_a[:, ind_a]) - np.matmul(JTCurr, a_lambda[:, ind_a])).reshape(22, 1)

        if data_begin_a == True:
            RHS_a = RHSCurr
            data_begin_a = False
        else:
            RHS_a = np.concatenate((RHS_a, RHSCurr), axis = 1)
        ind_a += 1

    # K_method - solve for Kq (ignore the C and Bu terms)

    RHS_K = np.zeros(22)

    data_begin_K = True
    ind_K = 0
    while ind_K < t.size - 1:
        xCurr = x[:, ind_K]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cacheCurr = tree.doKinematics(q, v)
        MCurr = tree.massMatrix(cacheCurr) # M

        cInfoCurr = ComputeCassieContactInfo(tree, q)
        cToolkitCurr = ContactToolkit(tree, cInfoCurr)
        JctCurr = cToolkitCurr.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cacheCurr, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

        JTCurr = np.concatenate((JctTCurr, JcTCurr), axis = 1)

        RHSCurr = (np.matmul(MCurr, K_a[:, ind_K]) - np.matmul(JTCurr, K_lambda[:, ind_K])).reshape(22, 1)

        if data_begin_K == True:
            RHS_K = RHSCurr
            data_begin_K = False
        else:
            RHS_K = np.concatenate((RHS_K, RHSCurr), axis = 1)
        ind_K += 1

    '''
    ind = 0
    while ind < 22:
        plt.title("Plot of " + str(ind) + " diff in first eq RHS vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], np.absolute(RHS_a - RHS_K)[ind, :])
        plt.show()
        ind += 1
    '''
    ################################################## no more than 0.37 diff; most very small ####################################################

    print("----------------------------- seeing if JDotV + Ja = 0 for both ------------------------------")

    Ja_a = np.array([])
    Ja_K = np.array([])

    data_begin = True
    ind = 0
    while ind < t.size - 1:
        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cacheCurr = tree.doKinematics(q, v)

        cInfoCurr = ComputeCassieContactInfo(tree, q)
        cToolkitCurr = ContactToolkit(tree, cInfoCurr)
        JctCurr = cToolkitCurr.CalcContactJacobian(xCurr, False)

        JcCurr = tree.positionConstraintsJacobian(cacheCurr, False)

        JCurr = np.concatenate((JctCurr, JcCurr), axis = 0)

        Ja_aCurr = np.matmul(JCurr, a_a[:, ind]).reshape(14, 1)
        Ja_KCurr = np.matmul(JCurr, K_a[:, ind]).reshape(14, 1)

        if data_begin == True:
            Ja_a = Ja_aCurr
            Ja_K = Ja_KCurr
            data_begin = False
        else:
            Ja_a = np.concatenate((Ja_a, Ja_aCurr), axis = 1)
            Ja_K = np.concatenate((Ja_K, Ja_KCurr), axis = 1)
        ind += 1
    '''
    ind = 0
    while ind < 14:
        plt.title("Plot of " + str(ind) + " JDotV + Ja (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], (a_JDotV[ind, :] + Ja_a[ind, :]), 'r', t[:t.size - 1], (a_JDotV[ind, :] + Ja_K[ind, :]), 'g')
        plt.show()
        ind += 1
    '''
    #################################################### VERY SMALL FOR ALL 14 ##################################################

    print("---------------------------- checking M and Ma ----------------------------")

    Ma_a = np.array([])
    Ma_K = np.array([])

    data_begin = True
    ind = 0
    while ind < t.size - 1:
        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        MCurr = tree.massMatrix(cacheCurr) # M
        MaCurr_a = np.matmul(MCurr, a_a[:, ind]).reshape(22, 1)
        MaCurr_K = np.matmul(MCurr, K_a[:, ind]).reshape(22, 1)

        if data_begin == True:
            Ma_a = MaCurr_a
            Ma_K = MaCurr_K
            data_begin = False
        else:
            Ma_a = np.concatenate((Ma_a, MaCurr_a), axis = 1)
            Ma_K = np.concatenate((Ma_K, MaCurr_K), axis = 1)
        ind += 1
    '''
    ind = 0
    while ind < 22:
        plt.title("Plot of " + str(ind) + " Ma (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], Ma_a[ind, :], 'r', t[:t.size - 1], Ma_K[ind, :], 'g')
        plt.show()
        ind += 1
    '''
    ################################################# M remains the same regardless of time, thus Ma mirrors a's shape with less disturbance ###########################################
    print("---------------------------- JTL plots ------------------------------")
    '''
    delta_JTlambda12_matrix = np.zeros(22)
    delta_JTlambda2_matrix = np.zeros(22)
    a_JTlambda12 = np.zeros(22)
    a_JTlambda2 = np.zeros(22)
    K_JTlambda12 = np.zeros(22)
    K_JTlambda2 = np.zeros(22)
    begin_concat = True
    ind = 0
    while ind < t.size - 1:

        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cache = tree.doKinematics(q, v)

        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose
        
        JcCurr = tree.positionConstraintsJacobian(cache, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

        JTL_a12 = np.matmul(JctTCurr, a_lambda[:12, ind]).reshape(22,1)
        JTL_a2 = np.matmul(JcTCurr, a_lambda[12:14, ind]).reshape(22,1)
        JTL_K12 = np.matmul(JctTCurr, K_lambda[:12, ind]).reshape(22,1)
        JTL_K2 = np.matmul(JcTCurr, K_lambda[12:14, ind]).reshape(22,1)

        DJTL_12 = np.absolute(np.matmul(JctTCurr, a_lambda[:12, ind]) - np.matmul(JctTCurr, K_lambda[:12, ind])).reshape(22,1)
        DJTL_2 = np.absolute(np.matmul(JcTCurr, a_lambda[12:14, ind]) - np.matmul(JcTCurr, K_lambda[12:14, ind])).reshape(22,1)

        if begin_concat == True:
            delta_JTlambda12_matrix = DJTL_12
            delta_JTlambda2_matrix = DJTL_2
            a_JTlambda12 = JTL_a12
            a_JTlambda2 = JTL_a2
            K_JTlambda12 = JTL_K12
            K_JTlambda2 = JTL_K2
            begin_concat = False
        else:
            delta_JTlambda12_matrix = np.concatenate((delta_JTlambda12_matrix, DJTL_12), axis = 1)
            delta_JTlambda2_matrix = np.concatenate((delta_JTlambda12_matrix, DJTL_2), axis = 1)
            a_JTlambda12 = np.concatenate((a_JTlambda12, JTL_a12), axis = 1)
            a_JTlambda2 = np.concatenate((a_JTlambda2, JTL_a2), axis = 1)
            K_JTlambda12 = np.concatenate((K_JTlambda12, JTL_K12), axis = 1)
            K_JTlambda2 = np.concatenate((K_JTlambda2, JTL_K2), axis = 1)
        ind += 1

    ind = 0
    while ind < 12:
        plt.title("Plot of 12 " + str(ind) + "JTlambdas of 2 methods vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], a_JTlambda12[ind, :], 'r', t[:t.size - 1], K_JTlambda12[ind, :], 'g')
        plt.show()
        ind += 1

    ind = 0
    while ind < 2:
        plt.title("Plot of 2 " + str(ind) + " JTlambdas of 2 methods vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], a_JTlambda2[ind, :], 'r', t[:t.size - 1], K_JTlambda2[ind, :], 'g')
        plt.show()
        ind += 1

    #### plotting JTlambdas ####

    ind = 0
    while ind < 14:
        plt.title("Plot of " + str(ind) + " lambdas of 2 methods vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], a_lambda[ind, :], 'r', t[:t.size - 1], K_lambda[ind, :], 'g')
        plt.show()
        ind += 1
    '''
    print("--------------------------------------- Optimization ---------------------------------------")

    



if __name__ == "__main__":
    main()
