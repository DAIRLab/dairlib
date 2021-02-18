import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt

import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody
import pydairlib.lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
from pydrake.math import RigidTransform
import numpy as np





def main():

    # Default filename for the example
    logfile_name = FindResourceOrThrow("examples/PlanarWalker/trajectories/good_trajectories/walking_traj.lcmtraj")
    plant_file = FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf")


    if len(sys.argv) == 2:
        logfile_name = sys.argv[1]
    dircon_traj = pydairlib.lcm_trajectory.DirconTrajectory(logfile_name)

    # Reconstructing state and input trajectory as piecewise polynomials
    state_traj = dircon_traj.ReconstructStateTrajectory()
    input_traj = dircon_traj.ReconstructInputTrajectory()
    state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
    input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes

    t_mode_switch = np.zeros((dircon_traj.GetNumModes()-1,))
    for i in range(dircon_traj.GetNumModes() - 1):
        t_mode_switch[i] = dircon_traj.GetForceBreaks(i)[-1][0]

    force_traj0 = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(0), dircon_traj.GetForceSamples(0))
    force_traj1 = PiecewisePolynomial.ZeroOrderHold(dircon_traj.GetForceBreaks(1), dircon_traj.GetForceSamples(1))
    force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes


    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(plant, scene_graph)
    parser.AddModelFromFile(plant_file)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                     RigidTransform())
    plant.Finalize()

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = nq + nv
    nu = plant.num_actuators()

    context = plant.CreateDefaultContext()

    foot_pt = np.array((0, 0, -0.5))

    pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
    print(pos_map)
    vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)

    theta_idx = pos_map["planar_roty"]
    omega_idx = vel_map["planar_rotydot"]

    n_points = 10000
    t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
    state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
    input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
    force_samples0 = np.zeros((n_points, force_traj0.value(0).shape[0]))
    force_samples1 = np.zeros((n_points, force_traj1.value(0).shape[0]))

    com_pos_samples = np.zeros((n_points, 2))
    theta_samples = np.zeros((n_points, 1))
    left_pos_samples = np.zeros((n_points,2))
    right_pos_samples = np.zeros((n_points, 2))
    com_vel_samples = np.zeros((n_points, 2))
    omega_samples = np.zeros((n_points, 1))
    left_vel_samples = np.zeros((n_points,2))
    right_vel_samples = np.zeros((n_points, 2))
    left_force_samples = np.zeros((n_points, 2))
    right_force_samples = np.zeros((n_points, 2))

    left_accel_samples = np.zeros((n_points, 2))
    right_accel_samples = np.zeros((n_points,2))


    for i in range(n_points):
        state_samples[i] = state_traj.value(t[i])[:, 0]
        input_samples[i] = input_traj.value(t[i])[:, 0]
        if t[i] <= t_mode_switch[0]:
            force_samples0[i] = force_traj0.value(t[i])[:, 0]
            force_samples1[i] = np.zeros((3,))
        else :
            force_samples1[i] = force_traj1.value(t[i])[:, 0]
            force_samples0[i] = np.zeros((3,))

        #print(state_samples[i])
        plant.SetPositionsAndVelocities(context, state_samples[i])

        com = plant.CalcCenterOfMassPosition(context)
        comdot = \
            plant.CalcJacobianCenterOfMassTranslationalVelocity(
            context=context, with_respect_to=JacobianWrtVariable.kV,
            frame_A=plant.world_frame(), frame_E=plant.world_frame()) @ \
            plant.GetVelocities(context)

        lp = plant.CalcPointsPositions(context=context,
                                    frame_B=plant.GetFrameByName("left_lower_leg"),
                                    p_BQi=foot_pt, frame_A=plant.world_frame())
        rp = plant.CalcPointsPositions(context=context,
                                       frame_B=plant.GetFrameByName("right_lower_leg"),
                                       p_BQi=foot_pt, frame_A=plant.world_frame())

        lp_dot = plant.CalcJacobianTranslationalVelocity(context=context,
                       with_respect_to=JacobianWrtVariable.kV,
                       frame_B=plant.GetFrameByName("left_lower_leg"),
                       p_BoBi_B=foot_pt, frame_A=plant.world_frame(),
                       frame_E=plant.world_frame()) @  plant.GetVelocities(context)

        rp_dot = plant.CalcJacobianTranslationalVelocity(context=context,
                       with_respect_to=JacobianWrtVariable.kV,
                       frame_B=plant.GetFrameByName("right_lower_leg"),
                       p_BoBi_B=foot_pt, frame_A=plant.world_frame(),
                       frame_E=plant.world_frame()) @  plant.GetVelocities(context)

        theta = plant.GetPositions(context)[theta_idx]
        omega = plant.GetVelocities(context)[omega_idx]


        left_force_samples[i,:] = force_samples0[i,::2]
        right_force_samples[i,:] = force_samples1[i,::2]
        com_pos_samples[i,:] = com[::2]
        com_vel_samples[i,:] = comdot[::2]
        left_pos_samples[i,:] = np.array((lp[0][0], lp[-1][0]))
        right_pos_samples[i,:] = np.array((rp[0][0], rp[-1][0]))
        left_vel_samples[i,:] = lp_dot[::2]
        right_vel_samples[i,:] = rp_dot[::2]

        if (i!=0):
            left_accel_samples[i,:] = (1/(t[i] - t[i-1]))*left_vel_samples[i,:] - left_vel_samples[i-1]
            right_accel_samples[i,:] = (1/(t[i] - t[i-1]))*right_vel_samples[i,:] - right_vel_samples[i-1]

        theta_samples[i] = theta
        omega_samples[i] = omega

    centroidal_traj = np.hstack((com_pos_samples, theta_samples, left_pos_samples,
                                right_pos_samples, com_vel_samples, omega_samples,
                                left_vel_samples, right_vel_samples, left_force_samples,
                                right_force_samples)).T

    input_traj = np.hstack((left_accel_samples, right_accel_samples,
                           np.zeros((n_points,4)))).T

    np.savetxt("/home/brian/Documents/Research/Quals/DirconTraj25cm.csv", centroidal_traj, delimiter=",")
    np.savetxt("/home/brian/Documents/Research/Quals/DirconInput25cm.csv", input_traj, delimiter=",")
    np.savetxt("/home/brian/Documents/Research/Quals/DirconTime25cm.csv", t, delimiter=",")

    # Plotting reconstructed state trajectories
    plt.figure("COM traj")
    plt.plot(t, com_pos_samples)
    plt.legend(['x', 'y'])

    plt.figure("COM vel")
    plt.plot(t, com_vel_samples)
    plt.legend(['x', 'y'])

    plt.figure("state trajectory")
    plt.plot(t, state_samples)
    plt.legend(state_datatypes)

    plt.figure("input trajectory")
    plt.plot(t, input_samples)
    plt.legend(input_datatypes)

    plt.figure("force trajectory")
    plt.plot(t, force_samples0)
    plt.plot(t, force_samples1)
    plt.legend(['0x', '0y', '0z', '1x', '1y', '1z'])

    plt.show()

if __name__ == "__main__":
    main()
