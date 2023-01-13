import sys
import lcm
import numpy as np
import scipy.linalg as linalg
import pathlib

import matplotlib.pyplot as plt
import matplotlib
from pydairlib.lcm import lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
from pydairlib.common import plot_styler, plotting_utils
from pydairlib.cassie.cassie_utils import *

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder

def rabbit_main():

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile(
        "examples/impact_invariant_control/five_link_biped.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.Finalize()
    context = plant.CreateDefaultContext()

    world = plant.world_frame()

    l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
    r_contact_frame = plant.GetBodyByName("right_foot").body_frame()

    pos_map = pydairlib.multibody.MakeNameToPositionsMap(plant)
    vel_map = pydairlib.multibody.MakeNameToVelocitiesMap(plant)
    act_map = pydairlib.multibody.MakeNameToActuatorsMap(plant)

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = plant.num_positions() + plant.num_velocities()
    nu = plant.num_actuators()
    nc = 2
    n_samples = 1000
    window_size = 0.05


    # Map to 2d
    TXZ = np.array([[1, 0, 0], [0, 0, 1]])

    filename = "examples/impact_invariant_control/saved_trajectories/rabbit_walking"
    dircon_traj = lcm_trajectory.DirconTrajectory(plant, filename)
    state_traj = dircon_traj.ReconstructStateTrajectory()
    breaks = dircon_traj.GetBreaks()

    x_pre = state_traj.value(dircon_traj.GetStateBreaks(0)[-1] - 1e-6)
    x_post = state_traj.value(dircon_traj.GetStateBreaks(1)[0])

    plant.SetPositionsAndVelocities(context, x_pre)
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    pt_on_body = np.zeros(3)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_contact_frame, pt_on_body, world, world)
    M_Jt = M_inv @ J_r.T
    P = linalg.null_space(M_Jt.T).T

    proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
    J_M_Jt = J_r @ M_inv @ J_r.T
    proj_y_ii = np.eye(2) - J_M_Jt @ J_M_Jt @ np.linalg.inv(J_M_Jt.T @ J_M_Jt) @ J_M_Jt.T

    v_pre = x_pre[-nv:]
    v_post = x_post[-nv:]
    t_impact = dircon_traj.GetStateBreaks(0)[-1]

    ydot_pre = J_r @ v_pre
    ydot_post = J_r @ v_post
    transform = J_r @ M_inv @ J_r.T @ np.linalg.pinv(J_r @ M_inv @ J_r.T)

    t = np.linspace(t_impact - window_size, t_impact + window_size, n_samples)

    cc = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
    # cc_vel = np.zeros((t.shape[0], nv - nc))
    cc_vel = np.zeros((t.shape[0], nv))
    # TV_cc_vel = np.zeros((t.shape[0], nv - nc))
    TV_cc_vel = np.zeros((t.shape[0], nv))
    vel = np.zeros((t.shape[0], nv))
    for i in range(t.shape[0]):
        x = state_traj.value(t[i])
        vel[i] = x[-nv:, 0]
        plant.SetPositionsAndVelocities(context, x)
        M = plant.CalcMassMatrixViaInverseDynamics(context)
        M_inv = np.linalg.inv(M)
        J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
        TV_J = M_inv @ J_r.T
        TV_J_space = linalg.null_space(TV_J.T).T
        cc_vel[i] = P.T @ P @ vel[i]
        TV_cc_vel[i] = TV_J_space.T @ TV_J_space @ vel[i]

    gen_vel_plot = plot_styler.PlotStyler()
    gen_vel_plot.plot(t, vel, title='Generalized Velocities near Impact', xlabel='time (s)', ylabel='velocity (m/s)')
    ylim = gen_vel_plot.fig.gca().get_ylim()
    # ps.save_fig('generalized_velocities_around_impact.png')
    proj_vel_plot = plot_styler.PlotStyler()
    # proj_vel_plot.plot(t, cc_vel, title='Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)')
    proj_vel_plot.plot(t, cc_vel, title='Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    # tv_proj_vel_plot = plot_styler.PlotStyler()
    # tv_proj_vel_plot.plot(t, TV_cc_vel, title='Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    # ps.save_fig('projected_velocities_around_impact.png')
    plt.show()

if __name__ == '__main__':
    rabbit_main()