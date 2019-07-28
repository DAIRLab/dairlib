# various debugging functions to compare accelerations from matrix equations method and results from limits method
# note that Cassie must be floating base quaternion on ground

import sys
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

# comparing accelerations
def plotAccels(t, a_a, aLim, tree, numV):

    ind = 0
    while ind < numV:
        plt.title("Plot of acceleration of " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot((t[:t.size - 1]), a_a[ind,:t.size - 1], 'r', label = 'matrix solution')
        plt.plot((t[:t.size - 1]), aLim[ind,:], 'g', label = 'limit solution') # calculated accel vs approx accel of position against t
        plt.minorticks_on()
        plt.grid(which = 'major', linestyle = '-', linewidth = '0.5', color ='black')
        plt.grid(which = 'minor', linestyle = ':', linewidth = '0.5', color = 'blue')
        plt.legend()
        plt.show()
        ind += 1

# comparing lambdas
def plotLambdas(t, a_lambda, K_lambda, tree):

    lambdaNames = ["toe_left_contact_1_z", "toe_left_contact_1_x", "toe_left_contact_1_y", "toe_left_contact_2_z", "toe_left_contact_2_x", "toe_left_contact_2_y",
                   "toe_right_contact_1_z", "toe_right_contact_1_x", "toe_right_contact_1_y", "toe_right_contact_2_z", "toe_right_contact_2_x", "toe_right_contact_2_y",
                   "geo_constraint_1", "geo_constraint_2"]

    ind = 0
    while ind < 14:
        plt.title("Plot of lambda of " + str(lambdaNames[ind]) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], a_lambda[ind, :], 'r', label = 'matrix solution')
        plt.plot(t[:t.size - 1], K_lambda[ind, :], 'g', label = 'limit solution')
        plt.minorticks_on()
        plt.grid(which = 'major', linestyle = '-', linewidth = '0.5', color ='black')
        plt.grid(which = 'minor', linestyle = ':', linewidth = '0.5', color = 'blue')
        plt.legend()
        plt.show()
        ind += 1

# comparing manipulator equation
def compareManipulatorEq(t, x, a_a, a_lambda, K_a, K_lambda, tree, numQ, numV):

    # a_method - solve for Kq (ignore the C and Bu terms)

    RHS_a = np.zeros(22)

    ind_a = 0
    while ind_a < t.size - 1:
        xCurr = x[:, ind_a]
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

        RHSCurr = (np.matmul(MCurr, a_a[:, ind_a]) - np.matmul(JTCurr, a_lambda[:, ind_a])).reshape(22, 1)

        if ind_a == 0:
            RHS_a = RHSCurr
        else:
            RHS_a = np.concatenate((RHS_a, RHSCurr), axis = 1)
        ind_a += 1

    # K_method - solve for Kq (ignore the C and Bu terms)

    RHS_K = np.zeros(22)

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

        if ind_K == 0:
            RHS_K = RHSCurr
        else:
            RHS_K = np.concatenate((RHS_K, RHSCurr), axis = 1)
        ind_K += 1

    ind = 0
    while ind < numV:
        plt.title("Plot of manipulator eq difference abs. value " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], np.absolute(RHS_a - RHS_K)[ind, :])
        plt.minorticks_on()
        plt.grid(which = 'major', linestyle = '-', linewidth = '0.5', color ='black')
        plt.grid(which = 'minor', linestyle = ':', linewidth = '0.5', color = 'blue')
        plt.show()
        ind += 1
    
# comparing constraint equation
def compareConstraintEq(t, x, tree, numQ, a_a, K_a, JDotV):

    Ja_a = np.array([])
    Ja_K = np.array([])

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

        if ind == 0:
            Ja_a = Ja_aCurr
            Ja_K = Ja_KCurr
        else:
            Ja_a = np.concatenate((Ja_a, Ja_aCurr), axis = 1)
            Ja_K = np.concatenate((Ja_K, Ja_KCurr), axis = 1)
        ind += 1
    
    ind = 0
    while ind < 14:
        plt.title("Plot of " + str(ind) + " JDotV + Ja (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], (JDotV[ind, :] + Ja_a[ind, :]), 'r', label = 'matrix solution')
        plt.plot(t[:t.size - 1], (JDotV[ind, :] + Ja_K[ind, :]), 'g', label = 'limit solution')
        plt.minorticks_on()
        plt.grid(which = 'major', linestyle = '-', linewidth = '0.5', color ='black')
        plt.grid(which = 'minor', linestyle = ':', linewidth = '0.5', color = 'blue')
        plt.legend()
        plt.show()
        ind += 1

# show mass matrix
def showMassMatrix(t, x, tree, numQ):

    ############# CUSTOMIZE THIS TO OBTAIN MASS MATRIX AT DESIRED TIME #############
    ############# CUSTOMIZE THIS TO OBTAIN MASS MATRIX AT DESIRED TIME #############
    ind = 0
    ############# CUSTOMIZE THIS TO OBTAIN MASS MATRIX AT DESIRED TIME #############
    ############# CUSTOMIZE THIS TO OBTAIN MASS MATRIX AT DESIRED TIME #############

    xCurr = x[:, ind]
    q = xCurr[:numQ]
    v = xCurr[numQ:]

    cacheCurr = tree.doKinematics(q, v)
    MCurr = tree.massMatrix(cacheCurr)
    print("Mass matrix at t = " + str(ind) + ":")
    print(MCurr)

# compare Ma
def compareMa(t, x, tree, a_a, K_a, numQ, numV):

    Ma_a = np.array([])
    Ma_K = np.array([])

    ind = 0
    while ind < t.size - 1:
        xCurr = x[:, ind]
        q = xCurr[:numQ]
        v = xCurr[numQ:]

        cacheCurr = tree.doKinematics(q, v)

        MCurr = tree.massMatrix(cacheCurr) # M
        MaCurr_a = np.matmul(MCurr, a_a[:, ind]).reshape(22, 1)
        MaCurr_K = np.matmul(MCurr, K_a[:, ind]).reshape(22, 1)

        if ind == 0:
            Ma_a = MaCurr_a
            Ma_K = MaCurr_K
        else:
            Ma_a = np.concatenate((Ma_a, MaCurr_a), axis = 1)
            Ma_K = np.concatenate((Ma_K, MaCurr_K), axis = 1)
        ind += 1
    
    ind = 0
    while ind < numV:
        plt.title("Plot of Ma of " + getVelocityName(tree, ind) + " (index = " + str(ind) + ") vs t (t.size = " + str(t.size) + ")")
        plt.plot(t[:t.size - 1], Ma_a[ind, :], 'r', label = 'matrix solution')
        plt.plot(t[:t.size - 1], Ma_K[ind, :], 'g', label = 'limit solution')
        plt.minorticks_on()
        plt.grid(which = 'major', linestyle = '-', linewidth = '0.5', color ='black')
        plt.grid(which = 'minor', linestyle = ':', linewidth = '0.5', color = 'blue')
        plt.show()
        ind += 1

# main
def main():

    script = sys.argv[0]
    filename = sys.argv[1]

    ############# CHANGE THIS TO CHOOSE WHICH FUNCTION TO RUN #############
    ############# CHANGE THIS TO CHOOSE WHICH FUNCTION TO RUN #############
    selection = 5
    ############# CHANGE THIS TO CHOOSE WHICH FUNCTION TO RUN #############
    ############# CHANGE THIS TO CHOOSE WHICH FUNCTION TO RUN #############

    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1)
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, filename, channel, 10e6)

    numQ = tree.get_num_positions()
    numV = tree.get_num_velocities()
    numJ = 14

    # accelerations found through limits method
    aLim = np.zeros((numV, t.size - 1)) # discard last data set eventually to accomodate for acceleration calculation
    
    ind = 0
    while ind < numV:
        time = 0
        while time < t.size - 1:
            aLim[ind, time] = (x[numQ + ind, time + 1] - x[numQ + ind, time]) / (t[time + 1] - t[time])
            time += 1
        ind += 1

    # accelerations found through matrix equations method

    deltaK = np.zeros((numV, 4))
    deltaK[14, 0] = 0.0
    deltaK[15, 1] = 0.0
    deltaK[18, 2] = 0.0
    deltaK[19, 3] = 0.0 # difference between non-zero K values in cassie_utils and computed K values from data (ideally 0)

    A_a = np.array([])
    JDotV = np.array([])

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

        if ind_a == 0:
            A_a = ACurr
            b_a = bCurr
            JDotV = np.concatenate((JctDotVCurr.reshape(12, 1), JcDotVCurr.reshape(2, 1)), axis = 0)
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

    # solve for lambdas given accelerations found through limits

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

        if ind_K == 0:
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
    sol_K = X_K[0]

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

    if selection == 0:
        plotAccels(t, a_a, aLim, tree, numV)
    elif selection == 1:
        plotLambdas(t, a_lambda, K_lambda, tree)
    elif selection == 2:
        compareManipulatorEq(t, x, a_a, a_lambda, K_a, K_lambda, tree, numQ, numV)
    elif selection == 3:
        compareConstraintEq(t, x, tree, numQ, a_a, K_a, JDotV)
    elif selection == 4:
        showMassMatrix(t, x ,tree, numQ)
    elif selection == 5:
        compareMa(t, x, tree, a_a, K_a, numQ, numV)
    else:
        print("Select a valid function:")
        print("0: plot accels from both methods onto same graph")
        print("1: plot lambdas from both methods onto same graph")
        print("2: plot difference in manipulator equation Ma + C = Bu + Kq + JTlambda from both methods onto same graph")
        print("3: plot difference in constraint equation Ja + Jdotv = 0 from both methods onto same graph")
        print("4: show mass matrices of both methods at specified time")
        print("5: plot difference in Ma of both methods onto same graph")

if __name__ == "__main__":
    main()
