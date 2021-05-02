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

    t_u_slice = slice(0, len(t_u) -1)

    ders = ["pos", "vel", "accel"]
    for i in range(3):
        plot_osc(osc_debug, t_u_slice, "swing_ft_traj", 0, ders[i])
        plot_osc(osc_debug, t_u_slice, "swing_ft_traj", 2, ders[i])
        plot_osc(osc_debug, t_u_slice, "com_traj", 0, ders[i])
        plot_osc(osc_debug, t_u_slice, "com_traj", 2, ders[i])
        plot_osc(osc_debug, t_u_slice, "base_angle", 0, ders[i])

    plot_mpc_com_sol(mpc_output[0], 0)
    plot_mpc_com_sol(mpc_output[0], 1)
    plot_mpc_swing_sol(mpc_output[0], 0)

    plt.show()


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