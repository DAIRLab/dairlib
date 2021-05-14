import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from process_koopman_lcm_log import *
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow

def save_poses_to_csv(plant, x, t_x, dt,  filename):
    t_vis = np.argwhere(np.isclose(np.fmod(t_x, dt), np.zeros((t_x.shape[0],)), atol=.001))
    t_vis = np.ravel(t_vis)
    x_out = x[t_vis,0:plant.num_positions()]
    np.savetxt(filename, x_out.T, delimiter=",")

def main():

    builder = DiagramBuilder()
    plant, _  =AddMultibodyPlantSceneGraph(builder, 0.0)

    Parser(plant).AddModelFromFile(
        FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf"))

    plant.mutable_gravity_field().set_gravity_vector(-9.81*np.array([0,0,1]))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.Finalize()

    pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
    vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
    act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

    context = plant.CreateDefaultContext()
    world_frame = plant.world_frame()
    torso_frame = plant.GetBodyByName("torso_mass").body_frame()
    left_leg_frame = plant.GetBodyByName("left_lower_leg").body_frame()
    right_leg_frame = plant.GetBodyByName("right_lower_leg").body_frame()
    foot_offest = np.array([0, 0, -0.5])

    plant_params = {'world_frame': world_frame, 'torso_frame': torso_frame,
                    'left_leg_frame' : left_leg_frame, 'right_leg_frame' : right_leg_frame,
                    'foot_offset' : foot_offest, 'pos_map' : pos_map, 'vel_map' : vel_map}

    try:
        logfile_name = sys.argv[1]
    except:
        logfile_name = input("PLEASE ENTER LOG FILE NAME: ")
        pass

    log = lcm.EventLog(logfile_name, "r")
    robot_out_channel = "PLANAR_STATE"
    mpc_channel = "KOOPMAN_MPC_OUT"
    osc_channel = "PLANAR_INPUT"
    osc_debug_channel = "OSC_DEBUG_WALKING"

    mpc_output, x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
    t_contact_info, osc_debug, fsm, t_controller_switch, t_pd, kp, kd, \
    robot_out, osc_output, full_log = process_mpc_log(log, pos_map, vel_map,
        act_map, robot_out_channel, mpc_channel, osc_channel, osc_debug_channel)

    outfolder = "examples/KoopmanMPC/saved_runs/srbd_trajectories/z_experiments/" + str(sys.argv[2]) + "cm"

    save_x_srbd(outfolder, plant, context, plant_params, t_x, x)
    # t_u_slice = slice(0, len(t_u) -1)
    #
    # ders = ["pos", "vel", "accel"]
    # for i in range(3):
    #     plot_osc(osc_debug, t_u_slice, "swing_ft_traj", 0, ders[i])
    #     plot_osc(osc_debug, t_u_slice, "swing_ft_traj", 2, ders[i])
    #     plot_osc(osc_debug, t_u_slice, "com_traj", 0, ders[i])
    #     plot_osc(osc_debug, t_u_slice, "com_traj", 2, ders[i])
    #     plot_osc(osc_debug, t_u_slice, "base_angle", 0, ders[i])
    #
    # plot_mpc_com_sol(mpc_output[0], 0)
    # plot_mpc_com_sol(mpc_output[0], 1)
    # plot_mpc_swing_sol(mpc_output[0], 0)
    #
    # plt.show()

def calc_srbd_state_from_plant(x, plant, context, plant_params):
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nxi = 12

    x_srbd = np.zeros((nxi,))

    plant.SetPositionsAndVelocities(context, x)
    xyz_com = plant.CalcPointsPositions(context, plant_params['torso_frame'],
                            np.zeros((3,1)), plant_params['world_frame'])

    xyz_left = plant.CalcPointsPositions(context, plant_params['left_leg_frame'],
                              plant_params['foot_offset'], plant_params['world_frame'])

    xyz_right = plant.CalcPointsPositions(context, plant_params['right_leg_frame'],
                              plant_params['foot_offset'], plant_params['world_frame'])

    jac_com = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, plant_params['torso_frame'],
                                            np.zeros((3,)), plant_params['world_frame'],
                                            plant_params['world_frame'])
    vel_com = jac_com @ x[nv:nv+nq]

    x_srbd[0:2] = np.ravel(xyz_com[[0,2]])
    x_srbd[2] = np.ravel(x[plant_params['pos_map']['planar_roty']])
    x_srbd[3:5] = np.ravel(vel_com[[0,2]])
    x_srbd[5] = np.ravel(x[plant_params['vel_map']['planar_rotydot']])
    x_srbd[6:8] = np.ravel(xyz_left[[0,2]])
    x_srbd[8:10] = np.ravel(xyz_right[[0,2]])
    return x_srbd

def save_x_srbd(filename, plant, context, params, t_x, x):
    x_srbd = np.zeros((12,t_x.shape[0]))
    for i in range(t_x.shape[0]):
        x_srbd[:,i] = calc_srbd_state_from_plant(np.ravel(x[i,:]), plant, context, params)

    np.savetxt(filename + ".csv", x_srbd, delimiter=",")
    np.savetxt(filename + "_time.csv", t_x, delimiter=",")

def plot_mpc_com_sol(mpc_sol, dim):
    fig_com = plt.figure("Koopman mpc com_traj " + str(dim))
    com_traj = mpc_sol.trajectories["com_traj"]
    plt.plot(com_traj.time_vec, com_traj.datapoints[dim, :])
    t, r = mpc_sol.traj_as_cubic_hermite("com_traj", 100)
    plt.plot(t,r[:,dim])

def plot_mpc_swing_sol(mpc_sol, dim):
    fig_swing_ft = plt.figure("Koopman mpc swing ft traj " + str(dim))
    swing_ft_traj = mpc_sol.trajectories["swing_foot_traj"]
    plt.plot(swing_ft_traj.time_vec, swing_ft_traj.datapoints[dim, :])
    t, r = mpc_sol.traj_as_cubic_with_continuous_second_derivatives("swing_foot_traj",100)
    plt.plot(t, r[:,dim])


def plot_osc(osc_debug,t_u_slice, osc_traj, dim, derivative):
    fig = plt.figure(osc_traj + " " + str(derivative) + " tracking " + str(dim))
    if (derivative == "pos"):
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y_des[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_y[t_u_slice, dim])
        plt.legend(["y_des", "y", "error_y"])
        # plt.legend(["y_des", "y"])
    elif (derivative == "vel"):
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim])
        plt.legend(["ydot_des", "ydot", "error_ydot"])
    elif (derivative == "accel"):
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_des[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command[t_u_slice, dim])
        plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command_sol[t_u_slice, dim])
        plt.legend(["yddot_des", "yddot_command", "yddot_command_sol"])

if __name__ == "__main__":
    main()