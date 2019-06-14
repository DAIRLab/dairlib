import numpy as np
import matplotlib.pyplot as plt
from pydrake.common import FindResourceOrThrow
from pydrake.attic.multibody.rigid_body_tree import FloatingBaseType, RigidBodyTree
from lcm_log_utils import parseLcmOutputLog
from contact_toolkit import ContactInfo, ContactToolkit
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

# sample inputs
def main():
    urdf_path = "/home/zhshen/workspace/dairlib/examples/Cassie/urdf/cassie.urdf"
    tree = RigidBodyTree(urdf_path,
                     floating_base_type=FloatingBaseType.kQuaternion)
    file = "/home/zhshen/workspace/dairlib/examples/LCM/data/Cassie_walking"
    channel = "CASSIE_STATE"
    [t, x, u] = parseLcmOutputLog(tree, file, channel, 10e6)
    q = x[:, 0] # sample time t = 0 used
    v = np.zeros(tree.get_num_velocities())
    cache = tree.doKinematics(q[:tree.get_num_positions()], v) # first 22 out of 45 terms correspond to positions

    print(tree.massMatrix(cache)) # inertia (M) term

    print(tree.dynamicsBiasTerm(cache, {})) # coriolis (C) term

    print(tree.B.dot(u[:, 0])) # Bu term for t = 0

    # bodies 3 - 17 (indices) are desired
    cInfo = ContactInfo(); # inputs (CHANGE THIS)
    cToolkit = ContactToolkit(tree, cInfo);
    print(cToolkit.CalcContactJacobian(q, False)) # contact jacobian



    # AddPositionConstraint()



    # sample plots
    #plt.plot(t, x[0, :]) # t vs first position of x
    #plt.show()

    #plt.plot(t, x[15, :]) # t vs 16th position of x
    #plt.show()

    #plt.plot(t, u[0, :]) # t vs first position of u
    #plt.show()

    #plt.plot(x[0, :], u[0, :]) # first position of x vs first position of u
    #plt.show()

    #plt.plot(x[0, :], x[1, :]) # first position of x vs second position of x
    #plt.show()

    #plt.plot(t, x[40, :]) # t vs 41st position of x
    #plt.show()

    #plt.plot(t, x[0, :], 'r', t, x[1, :], 'b', t, x[2, :], 'g') # multiple plots against t
    #plt.show()

    #fig = plt.figure()
    #ax = fig.gca(projection='3d')
    #pz = t
    #px = x[0, :]
    #py = x[1, :]
    #ax.set_xlabel('x0')
    #ax.set_ylabel('x1')
    #ax.set_zlabel('t')
    #ax.plot(px, py, pz) # 3D plot of the first 2 positions vs time
    #plt.show()

    #fig = plt.figure()
    #ax = fig.gca(projection='3d')
    #px, py = np.meshgrid(px, py)
    #surf = ax.plot_surface(px, py, pz, cmap=cm.coolwarm, linewidth=0, antialiased=False)
    #ax.set_zlim(0, 7)
    #ax.zaxis.set_major_locator(LinearLocator(10))
    #ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    #fig.colorbar(surf, shrink=0.5, aspect=5)
    #plt.show() # surface plot of the first 2 positions vs time
    
if __name__ == "__main__":
    main()
