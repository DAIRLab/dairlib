import sys
import lcm
import numpy as np
from scipy.linalg import null_space
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

def blend_sigmoid(t, tau, window):
    x = (t + window) / tau
    return np.exp(x) / (1 + np.exp(x))

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
    window_length = 0.05
    tau = 0.005
    # selected_joint_idxs = slice(6,7)
    selected_joint_idxs = slice(0,7)


    # Map to 2d
    TXZ = np.array([[1, 0, 0], [0, 0, 1]])

    filename = "examples/impact_invariant_control/saved_trajectories/rabbit_walking"
    dircon_traj = lcm_trajectory.DirconTrajectory(plant, filename)
    state_traj = dircon_traj.ReconstructStateTrajectory()
    breaks = dircon_traj.GetBreaks()

    transition_time = dircon_traj.GetStateBreaks(1)[0]
    x_pre = state_traj.value(transition_time - 1e-6)
    x_post = state_traj.value(transition_time)

    plant.SetPositionsAndVelocities(context, x_pre)
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    pt_on_body = np.zeros(3)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_contact_frame, pt_on_body, world, world)
    M_Jt = M_inv @ J_r.T
    P = null_space(M_Jt.T).T

    proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
    J_M_Jt = J_r @ M_inv @ J_r.T
    proj_y_ii = np.eye(2) - J_M_Jt @ J_M_Jt @ np.linalg.inv(J_M_Jt.T @ J_M_Jt) @ J_M_Jt.T

    v_pre = x_pre[-nv:]
    v_post = x_post[-nv:]
    t_impact = dircon_traj.GetStateBreaks(0)[-1]

    ydot_pre = J_r @ v_pre
    ydot_post = J_r @ v_post
    # transform = J_r @ M_inv @ J_r.T @ np.linalg.pinv(J_r @ M_inv @ J_r.T)
    transform = M_inv @ J_r.T @ np.linalg.pinv(J_r @ M_inv @ J_r.T)

    t = np.linspace(t_impact - 2*window_length, t_impact + 2*window_length, n_samples)
    t_proj = np.array([t_impact[0] - window_length, t_impact[0] + window_length])


    vel_proj = np.zeros((t.shape[0], nv))
    vel_time_varying_proj = np.zeros((t.shape[0], nv))
    vel = np.zeros((t.shape[0], nv))
    vel_corrected = np.zeros((t.shape[0], nv))
    vel_corrected_blend = np.zeros((t.shape[0], nv))
    alphas = np.zeros((t.shape[0], 1))
    for i in range(t.shape[0]):
        x = state_traj.value(t[i])
        vel[i] = x[-nv:, 0]
        vel_proj[i] = P.T @ P @ vel[i]

        plant.SetPositions(context, x[:nq])
        M = plant.CalcMassMatrixViaInverseDynamics(context)
        M_inv = np.linalg.inv(M)
        J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
        TV_J = M_inv @ J_r.T
        TV_J_space = null_space(TV_J.T).T

        if (t[i] < transition_time):
            alpha = blend_sigmoid(t[i] - transition_time, tau, 0.5 * window_length)
        else:
            alpha = blend_sigmoid(transition_time - t[i], tau, 0.5 * window_length)
        alphas[i] = alpha
        vel_time_varying_proj[i] = TV_J_space.T @ TV_J_space @ vel[i]
        # import pdb; pdb.set_trace()
        vel_correction = transform @ (J_r @ (vel[i] - np.zeros(nv)))
        # vel_correction = transform @ (J_r @ (vel[i] - np.ones(nv)))
        vel_corrected[i] = vel[i] - vel_correction
        vel_corrected_blend[i] = vel[i] - alpha * vel_correction

    gen_vel_plot = plot_styler.PlotStyler()
    gen_vel_plot.plot(t, vel, title='Generalized Velocities near Impact', xlabel='time (s)', ylabel='velocity (m/s)')
    ylim = gen_vel_plot.fig.gca().get_ylim()
    gen_vel_plot.save_fig('gen_vel_plot.png')
    # ps.save_fig('generalized_velocities_around_impact.png')

    proj_vel_plot = plot_styler.PlotStyler()
    proj_vel_plot.plot(t, vel_proj, title='Constant Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    proj_vel_plot.save_fig('proj_vel_plot.png')

    corrected_vel_plot = plot_styler.PlotStyler()
    corrected_vel_plot.plot(t, vel_corrected, title='Impact-Invariant Correction', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    corrected_vel_plot.save_fig('corrected_vel_plot.png')

    blended_vel_plot = plot_styler.PlotStyler()
    ax = blended_vel_plot.fig.axes[0]
    blended_vel_plot.plot(t, vel[:,selected_joint_idxs], title='Blended Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    blended_vel_plot.plot(t, vel_corrected_blend[:,selected_joint_idxs], title='Blended Impact-Invariant Projection', xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    ax.fill_between(t_proj, ylim[0], ylim[1], color=blended_vel_plot.grey, alpha=0.2)
    blended_vel_plot.save_fig('blended_vel_plot.png')

    blending_function_plot = plot_styler.PlotStyler()
    ax = blending_function_plot.fig.axes[0]
    blending_function_plot.plot(t, alphas, title="Blending Function")
    ax.fill_between(t_proj, ax.get_ylim()[0], ax.get_ylim()[1], color=blending_function_plot.grey, alpha=0.2)
    blending_function_plot.save_fig('blending_function_plot.png')

    plt.show()

if __name__ == '__main__':
    rabbit_main()