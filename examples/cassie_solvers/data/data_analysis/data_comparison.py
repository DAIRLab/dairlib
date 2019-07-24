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
    buildCassieTree(tree, urdf_path, FloatingBaseType.kQuaternion)
    AddFlatTerrainToWorld(tree, 100, 0.1)
    channel = "CASSIE_STATE"

    file1 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/d_p"
    [t1, x1, u1] = parseLcmOutputLog(tree, file1, channel, 10e6)

    file2 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/m_p"
    [t2, x2, u2] = parseLcmOutputLog(tree, file2, channel, 10e6)

    file3 = "/home/zhshen/workspace/dairlib/examples/cassie_solvers/data/g_p"
    [t3, x3, u3] = parseLcmOutputLog(tree, file3, channel, 10e6)

    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED
    '''
    t = t[78:93]
    x = x[:, 78:93]
    u = u[:, 78:93]
    '''
    # EDIT THIS TO CHANGE RANGE OF DATA TO BE USED

    t1 = np.around(t1, decimals = 3)
    t2 = np.around(t2, decimals = 3)
    t3 = np.around(t3, decimals = 3)

    common_times = np.intersect1d(t1, t2)
    common_times = np.intersect1d(common_times, t3)

    new_x1 = x1[:, np.where(t1 == common_times[0])[0]].reshape(45, 1)
    new_x2 = x2[:, np.where(t2 == common_times[0])[0]].reshape(45, 1)
    new_x3 = x3[:, np.where(t3 == common_times[0])[0]].reshape(45, 1)
    new_u1 = u1[:, np.where(t1 == common_times[0])[0]].reshape(10, 1)
    new_u2 = u2[:, np.where(t2 == common_times[0])[0]].reshape(10, 1)
    new_u3 = u3[:, np.where(t3 == common_times[0])[0]].reshape(10, 1)

    i = 1
    while i < common_times.size:
        new_x1 = np.concatenate((new_x1, x1[:, np.where(t1 == common_times[i])[0][0]].reshape(45, 1)), axis = 1)
        new_x2 = np.concatenate((new_x2, x2[:, np.where(t2 == common_times[i])[0][0]].reshape(45, 1)), axis = 1)
        new_x3 = np.concatenate((new_x3, x3[:, np.where(t3 == common_times[i])[0][0]].reshape(45, 1)), axis = 1)
        new_u1 = np.concatenate((new_u1, u1[:, np.where(t1 == common_times[i])[0][0]].reshape(10, 1)), axis = 1)
        new_u2 = np.concatenate((new_u2, u2[:, np.where(t2 == common_times[i])[0][0]].reshape(10, 1)), axis = 1)
        new_u3 = np.concatenate((new_u3, u3[:, np.where(t3 == common_times[i])[0][0]].reshape(10, 1)), axis = 1)
        i += 1

    inputs = ["hip_roll_left_motor", "hip_roll_right_motor", "hip_yaw_left_motor", "hip_yaw_right_motor", "hip_pitch_left_motor", 
              "hip_pitch_right_motor", "knee_left_motor", "knee_right_motor", "toe_left_motor", "toe_right_motor"]

    # plotting q
    ind = 0
    while ind < 23:
        plt.title("Plot of " + getPositionName(tree, ind) + " " + str(ind) + " q")
        plt.plot(common_times, new_x1[ind, :], 'r-', label = 'Drake')
        plt.plot(common_times, new_x2[ind, :], 'g-', label = 'Mujoco')
        plt.plot(common_times, new_x3[ind, :], 'b-', label = 'Gazebo')
        plt.legend()
        plt.show()
        ind += 1

    # plotting v
    ind = 0
    while ind < 22:
        plt.title("Plot of " + getVelocityName(tree, ind) + " " + str(ind) + " v")
        plt.plot(common_times, new_x1[23 + ind, :], 'r-', label = 'Drake')
        plt.plot(common_times, new_x2[23 + ind, :], 'g-', label = 'Mujoco')
        plt.plot(common_times, new_x3[23 + ind, :], 'b-', label = 'Gazebo')
        plt.legend()
        plt.show()
        ind += 1
    
    # plotting u
    ind = 0
    while ind < 10:
        plt.title("Plot of " + inputs[ind] + " " + str(ind) + " u")
        plt.plot(common_times, new_u1[ind, :], 'r-', label = 'Drake')
        plt.plot(common_times, new_u2[ind, :], 'g-', label = 'Mujoco')
        plt.plot(common_times, new_u3[ind, :], 'b-', label = 'Gazebo')
        plt.legend()
        plt.show()
        ind += 1
    
if __name__ == "__main__":
    main()
