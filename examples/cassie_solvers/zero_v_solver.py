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

# sample inputs
def main():

    urdf_path = "examples/Cassie/urdf/cassie_v2.urdf"
    tree = RigidBodyTree()
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1);
    file = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/Cassie_5"
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, file, channel, 10e6)

    x0 = x[:, 0]
    q0 = x0[:tree.get_num_positions()]

    v = np.zeros(tree.get_num_velocities()) # stationary
    cache0 = tree.doKinematics(q0, v) # 22 positions

    C = tree.dynamicsBiasTerm(cache0, {}, True)

    Bu = tree.B.dot(u[:, 0])

    cInfo0 = ComputeCassieContactInfo(tree, q0)
    cToolkit0 = ContactToolkit(tree, cInfo0)
    Jct = cToolkit0.CalcContactJacobian(x0, False)
    JctT = Jct.transpose() # contact jacobian transpose

    Jc = tree.positionConstraintsJacobian(cache0, False)
    JcT = Jc.transpose() # position constraint jacobian transpose
    
    ind = 1
    while ind < t.size:

        xCurr = x[:, ind]
        q = xCurr[:tree.get_num_positions()]

        cache = tree.doKinematics(q, v) # 22 positions

        CCurr = tree.dynamicsBiasTerm(cache, {}, True)

        BuCurr = tree.B.dot(u[:, ind])

        cInfo = ComputeCassieContactInfo(tree, q)
        cToolkit = ContactToolkit(tree, cInfo)
        JctCurr = cToolkit.CalcContactJacobian(xCurr, False)
        JctTCurr = JctCurr.transpose() # contact jacobian transpose

        JcCurr = tree.positionConstraintsJacobian(cache, False)
        JcTCurr = JcCurr.transpose() # position constraint jacobian transpose

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

    print(X[0][:4]) # first 4 correspond to spring constants


    
    print("---------------------- Testing ----------------------")
    print(X[0])
    print(X[1]) # 2 means problem solved
    print(X[2]) # iteration number when terminated
    print(X[3]) # norm(b - Ax)

    print("---------------------- Testing ----------------------")
    
    '''
    print(tree.getBodyOrFrameName(12) + " knee_joint_left")
    print(tree.getBodyOrFrameName(13) + " knee_joint_right")
    print(tree.getBodyOrFrameName(16) + " ankle_spring_joint_left")
    print(tree.getBodyOrFrameName(17) + " ankle_spring_joint_right")
    '''

    '''
    print(tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_left")).get_position_start_index())
    print(tree.get_body(tree.FindIndexOfChildBodyOfJoint("knee_joint_right")).get_position_start_index())
    print(tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")).get_position_start_index())
    print(tree.get_body(tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")).get_position_start_index())

    '''
    '''

    print("---------------------- Testing ----------------------")
    print(np.count_nonzero(A))
    print(np.count_nonzero(qMat))
    print(np.count_nonzero(JctT))
    print(np.count_nonzero(JcT))

    '''

    print(tree.getBodyOrFrameName(0))
    print(tree.getBodyOrFrameName(1))
    print(tree.getBodyOrFrameName(2))
    print(tree.getBodyOrFrameName(3))
    print(tree.getBodyOrFrameName(4))
    print(tree.getBodyOrFrameName(5))
    print(tree.getBodyOrFrameName(6))
    print(tree.getBodyOrFrameName(7))
    print(tree.getBodyOrFrameName(8))
    print(tree.getBodyOrFrameName(9))
    print(tree.getBodyOrFrameName(10))
    print(tree.getBodyOrFrameName(11))
    print(tree.getBodyOrFrameName(12))
    print(tree.getBodyOrFrameName(13))
    print(tree.getBodyOrFrameName(14))
    print(tree.getBodyOrFrameName(15))
    print(tree.getBodyOrFrameName(16))
    print(tree.getBodyOrFrameName(17))
    print(tree.getBodyOrFrameName(18) + " toe left")
    print(tree.getBodyOrFrameName(19))
    print(tree.getBodyOrFrameName(20) + " toe right")
    print(tree.getBodyOrFrameName(21))
    print(tree.getBodyOrFrameName(22))
    print(tree.getBodyOrFrameName(23))

    

    '''
        print("--Spring Bodies to q id--")

    index1 = tree.FindIndexOfChildBodyOfJoint("knee_joint_left")
    body1 = tree.get_body(index1)
    print(body1.get_position_start_index())

    index2 = tree.FindIndexOfChildBodyOfJoint("knee_joint_right")
    body2 = tree.get_body(index2)
    print(body2.get_position_start_index())

    index3 = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left")
    body3 = tree.get_body(index3)
    print(body3.get_position_start_index())

    index4 = tree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")
    body4 = tree.get_body(index4)
    print(body4.get_position_start_index())  

    '''
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

    plt.plot(x[0, :], x[1, :]) # first position of x vs second position of x
    plt.show()

    plt.plot(t, x[40, :]) # t vs 41st position of x
    plt.show()

    plt.plot(t, x[0, :], 'r', t, x[1, :], 'b', t, x[2, :], 'g') # multiple plots against t
    plt.show()

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    pz = t
    px = x[0, :]
    py = x[1, :]
    ax.set_xlabel('x0')
    ax.set_ylabel('x1')
    ax.set_zlabel('t')
    ax.plot(px, py, pz) # 3D plot of the first 2 positions vs time
    plt.show()

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    px, py = np.meshgrid(px, py)
    surf = ax.plot_surface(px, py, pz, cmap=cm.coolwarm, linewidth=0, antialiased=False)
    ax.set_zlim(0, 7)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show() # surface plot of the first 2 positions vs time
    '''
    
if __name__ == "__main__":
    main()
