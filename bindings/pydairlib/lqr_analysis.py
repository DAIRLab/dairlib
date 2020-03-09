import numpy as np
import pydairlib.lcm_trajectory
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable
import scipy.io


def main():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/five_link_biped/five_link_biped.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.mutable_gravity_field().set_gravity_vector(
        -9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nu = plant.num_actuators()
    nx = nq + nv
    nc = 2
    nd = nx - 2 * nc

    l_trajs = pydairlib.lcm_trajectory.LcmTrajectory()
    l_trajs.loadFromFile(
        "/home/yangwill/Documents/research/projects/hybrid_lqr/saved_trajs"
        "/L_traj_min")
    x_trajs = pydairlib.lcm_trajectory.LcmTrajectory()
    x_trajs.loadFromFile(
        "/home/yangwill/Documents/research/projects/hybrid_lqr/saved_trajs/2_step_walking_1_20")
    p_trajs = pydairlib.lcm_trajectory.LcmTrajectory()
    p_trajs.loadFromFile(
        "/home/yangwill/Documents/research/projects/hybrid_lqr/saved_trajs"
        "/P_traj")

    # mode 0
    p0 = p_trajs.getTrajectory("P0").datapoints
    p1 = p_trajs.getTrajectory("P1").datapoints
    p2 = p_trajs.getTrajectory("P2").datapoints
    # L matrix is calculated in backwards time!!!
    l0 = l_trajs.getTrajectory("l_traj0").datapoints
    l1 = l_trajs.getTrajectory("l_traj1").datapoints
    l2 = l_trajs.getTrajectory("l_traj2").datapoints

    P0 = np.reshape(p0[:, -1], (nd, nx), order='F')
    P1 = np.reshape(p1[:, 0], (nd, nx), order='F')
    # Get the beginning of the last mode (or end of the first mode)
    L0 = np.reshape(l0[:, 0], (nd, nd), order='F')
    # Get the end of the second mode (or beginning of the second mode)
    L1 = np.reshape(l1[:, -1], (nd, nd), order='F')
    S0 = L0 @ L0.T
    S1 = L1 @ L1.T
    S_max0 = P0.T @ S0 @ P0
    S_max1 = P1.T @ S1 @ P1
    # import pdb; pdb.set_trace()

    scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
                     '/plotting/L0.mat', {'l0': L0})
    scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
                     '/plotting/L1.mat', {'l1': L1})
    # scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
    #                  '/plotting/L2.mat', {'l2': L2})
    scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
                     '/plotting/S0.mat', {'s0': S0})
    scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
                     '/plotting/S1.mat', {'s1': S1})
    # scipy.io.savemat('/home/yangwill/Documents/research/projects/hybrid_lqr'
    #                  '/plotting/S2.mat', {'s2': S2})

    trajectory0 = x_trajs.getTrajectory("walking_trajectory_x_u0")
    trajectory1 = x_trajs.getTrajectory("walking_trajectory_x_u1")
    x_traj0 = trajectory0.datapoints[0:14, :]
    x_traj1 = trajectory1.datapoints[0:14, :]

    delta_t = 0.01
    # t_s = 0.01
    t_s = 0.00
    # t_s = 0.005

    x_minus = x_traj0[:, -1]
    x_plus = x_traj1[:, 0]
    # print(S_max0 * (x_minus - x_plus))
    # print(S_max1 * (x_minus - x_plus))
    context = plant.CreateDefaultContext()
    plant.SetPositionsAndVelocities(context, x_traj0[:, -1])
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    B = plant.MakeActuationMatrix()
    B_linear = np.vstack((np.zeros((nq, nu)), B))
    R = 0.01 * np.eye(nu, nu)
    R_inv = np.linalg.inv(R)
    M_inv = np.linalg.inv(M)
    S0_terms = M_inv @ B @ (-R_inv) @ B_linear.T @ S_max0
    S1_terms = M_inv @ B @ (-R_inv) @ B_linear.T @ S_max1
    net_impulse_early = S0_terms @ (x_minus - x_plus) * (t_s * t_s) / (
            2 * delta_t)
    net_impulse_late = -S1_terms @ (x_minus - x_plus) * (delta_t - t_s) * (
            delta_t - t_s) / (2 * delta_t)
    net_impulse_from_contact = (x_minus - x_plus)[7:14]
    inputs_0 = (-R_inv) @ B_linear.T @ S_max0 @ (x_minus - x_plus) * (
            t_s * t_s) / (2 * delta_t)
    inputs_1 = (-R_inv) @ B_linear.T @ (-S_max1) @ (x_minus - x_plus) * \
               (delta_t - t_s) * (delta_t - t_s) / (2 * delta_t)

    # import pdb
    # pdb.set_trace()
    print("max impulse: " )
    print(M_inv @ B @ np.array([30, 30, 30, 30]) * delta_t)

    print("input values: ")
    print((-R_inv) @ B_linear.T @ S_max0 @ (x_minus - x_plus))
    print((-R_inv) @ B_linear.T @ S_max1 @ (x_minus - x_plus))

    print("delta v from inputs early")
    print(net_impulse_early)
    print("delta v from inputs late")
    print(net_impulse_late)
    print("combined")
    print(net_impulse_early + net_impulse_late)
    print("delta v from contact")
    print(net_impulse_from_contact)

    print("impulse from inputs/impulse from contact:")
    print((net_impulse_early + net_impulse_late)/net_impulse_from_contact)

    print("Gain values for mode 0: ")
    print((-R_inv) @ B_linear.T @ S_max0)
    print("Gain values for mode 1: ")
    print((-R_inv) @ B_linear.T @ S_max1)
    # print((net_impulse_early) / (
    #     np.linalg.norm(net_impulse_from_contact)))
    # print(np.linalg.norm(net_impulse_from_contact))
    # print(inputs_0)
    # print(inputs_1)
    # print(inputs_0 + inputs_1)
    # print(M_inv @ B @ (inputs_0 + inputs_1))


if __name__ == "__main__":
    main()
