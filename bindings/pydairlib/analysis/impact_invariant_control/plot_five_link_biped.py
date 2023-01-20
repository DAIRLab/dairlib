import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

from scipy.linalg import null_space

import dairlib

from bindings.pydairlib.lcm import lcm_trajectory
from bindings.pydairlib.multibody import MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
    CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap
from bindings.pydairlib.common import FindResourceOrThrow
from bindings.pydairlib.common import plot_styler, plotting_utils
from bindings.pydairlib.cassie.cassie_utils import *
from bindings.pydairlib.lcm.process_lcm_log import get_log_data
import pydairlib.analysis.mbp_plotting_utils as mbp_plots

from pydrake.trajectories import PiecewisePolynomial
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder


def blend_sigmoid(t, tau, window):
    x = (t + window) / tau
    return np.exp(x) / (1 + np.exp(x))


def compute_control_law(N, d):
    nu = d.shape[0]
    nc = N.shape[1]
    # A = np.array([[np.eye(nu), N], [N.T, np.zeros((nc, nc))]])
    A = np.zeros((nc + nu, nc + nu))
    b = np.zeros((nc + nu))
    A[:nu, :nu] = np.eye(nu)
    A[-nc:, :nu] = N.T
    A[:nu, -nc:] = N
    A[-nc:, -nc:] = np.zeros((nc, nc))
    b[:nu] = d
    b[-nc:] = N.T @ d

    A_inv = np.linalg.inv(A)
    u = A_inv @ b


def load_logs(plant, t_impact, window):
    # log_dir = '/media/yangwill/backups/home/yangwill/Documents/research/projects/five_link_biped/invariant_impacts/logs/nominal/'
    log_dir = '/home/yangwill/Documents/research/papers/impact_invariant_control/data/five_link_biped/'
    # filename = 'lcmlog-000'
    # filename = 'lcmlog-nominal'
    filename = 'lcmlog-error'
    print(log_dir + filename)
    log = lcm.EventLog(log_dir + filename, "r")
    default_channels = {'RABBIT_STATE': dairlib.lcmt_robot_output,
                        'RABBIT_INPUT': dairlib.lcmt_robot_input,
                        'OSC_DEBUG_WALKING': dairlib.lcmt_osc_output}
    callback = mbp_plots.load_default_channels
    start_time = 0
    duration = -1
    robot_output, robot_input, osc_debug = \
        get_log_data(log, default_channels, start_time, duration, callback,
                     plant,
                     'RABBIT_STATE', 'RABBIT_INPUT', 'OSC_DEBUG_WALKING')
    return robot_output, robot_input, osc_debug


def main():
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

    pos_map = MakeNameToPositionsMap(plant)
    vel_map = MakeNameToVelocitiesMap(plant)
    act_map = MakeNameToActuatorsMap(plant)

    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = plant.num_positions() + plant.num_velocities()
    nu = plant.num_actuators()
    nc = 2
    n_samples = 1000
    window_length = 0.05
    tau = 0.0025
    selected_joint_idxs = slice(6,7)
    # selected_joint_idxs = slice(0, 7)

    # Map to 2d
    TXZ = np.array([[1, 0, 0], [0, 0, 1]])

    filename = "examples/impact_invariant_control/saved_trajectories/rabbit_walking"
    dircon_traj = lcm_trajectory.DirconTrajectory(plant, filename)
    state_traj = dircon_traj.ReconstructStateTrajectory()

    transition_time = dircon_traj.GetStateBreaks(1)[0]
    x_pre = state_traj.value(transition_time - 1e-6)
    x_post = state_traj.value(transition_time)

    robot_output, robot_input, osc_debug = load_logs(plant, transition_time, window_length)

    plant.SetPositionsAndVelocities(context, x_pre)
    M = plant.CalcMassMatrixViaInverseDynamics(context)
    M_inv = np.linalg.inv(M)
    pt_on_body = np.zeros(3)
    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV,
                                                        r_contact_frame,
                                                        pt_on_body, world,
                                                        world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context,
                                                        JacobianWrtVariable.kV,
                                                        l_contact_frame,
                                                        pt_on_body, world,
                                                        world)
    M_Jt = M_inv @ J_r.T

    # compute_control_law(M_Jt, np.ones(nv))

    P = null_space(M_Jt.T).T

    proj_ii = np.eye(nv) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
    J_M_Jt = J_r @ M_inv @ J_r.T
    proj_y_ii = np.eye(2) - J_M_Jt @ J_M_Jt @ np.linalg.inv(
        J_M_Jt.T @ J_M_Jt) @ J_M_Jt.T

    v_pre = x_pre[-nv:]
    v_post = x_post[-nv:]
    t_impact = dircon_traj.GetStateBreaks(0)[-1]

    # J_y = np.zeros((4, 7))
    J_y = np.eye((7))
    # J_y[0, 0] = 1
    # J_y[0, 1] = 1
    # J_y[0, 2] = 1
    # J_y[0, 3] = 1
    # J_y[1, 4] = 1
    # J_y[2, 5] = 1
    # J_y[3, 6] = 1
    ydot_pre = J_r @ v_pre
    ydot_post = J_r @ v_post
    # transform = J_r @ M_inv @ J_r.T @ np.linalg.pinv(J_r @ M_inv @ J_r.T)
    transform = M_inv @ J_r.T @ np.linalg.pinv(J_y @ M_inv @ J_r.T)


    start_time = t_impact - 2 * window_length
    end_time = t_impact + 2 * window_length
    start_idx = np.argwhere(np.abs(robot_output['t_x'] - start_time) < 1e-3)[0][0]
    end_idx = np.argwhere(np.abs(robot_output['t_x'] - end_time) < 1e-3)[0][0]
    t = np.linspace(start_time, end_time,
                    end_idx - start_idx)
    print(start_idx)
    print(end_idx)
    t_proj = np.array(
        [t_impact[0] - window_length, t_impact[0] + window_length])

    vel_proj = np.zeros((t.shape[0], nv))
    vel_proj_actual = np.zeros((t.shape[0], nv))
    vel_time_varying_proj = np.zeros((t.shape[0], nv))
    vel = np.zeros((t.shape[0], nv))
    vel_actual = np.zeros((t.shape[0], nv))
    vel_corrected = np.zeros((t.shape[0], nv))
    vel_correction = np.zeros((t.shape[0], nv))
    vel_corrected_blend = np.zeros((t.shape[0], nv))
    alphas = np.zeros((t.shape[0], 1))
    for i in range(t.shape[0]):
    # for i in range(0, end_idx - start_idx):
        # x = state_traj.value(t[i])
        x = state_traj.value(robot_output['t_x'][i + start_idx])
        vel[i] = x[-nv:, 0]
        vel_actual[i] = robot_output['v'][i + start_idx]
        vel_proj[i] = P.T @ P @ vel[i]
        vel_proj_actual[i] = P.T @ P @ vel_actual[i]

        plant.SetPositions(context, x[:nq])
        M = plant.CalcMassMatrixViaInverseDynamics(context)
        M_inv = np.linalg.inv(M)
        J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context,
                                                            JacobianWrtVariable.kV,
                                                            r_contact_frame,
                                                            pt_on_body, world,
                                                            world)
        TV_J = M_inv @ J_r.T
        TV_J_space = null_space(TV_J.T).T

        if (t[i] < transition_time):
            alpha = blend_sigmoid(t[i] - transition_time, tau,
                                  0.5 * window_length)
        else:
            alpha = blend_sigmoid(transition_time - t[i], tau,
                                  0.5 * window_length)
        alphas[i] = alpha
        vel_time_varying_proj[i] = TV_J_space.T @ TV_J_space @ vel[i]
        # vel_correction[i] = transform @ (J_r @ (vel[i] - vel_actual[i]))
        vel_correction[i] = transform @ (J_y @ vel[i] - J_y @ vel_actual[i])
        vel_corrected[i] = vel_actual[i] + vel_correction[i]
        vel_corrected_blend[i] = vel_actual[i] + alpha * vel_correction[i]
    # t_plot = robot_output['t_x'][start_idx:end_idx]

    gen_vel_plot = plot_styler.PlotStyler()
    gen_vel_plot.plot(t, vel[:, selected_joint_idxs],
                      xlabel='time (s)', ylabel='velocity (m/s)')
    gen_vel_plot.plot(t, vel_actual[:, selected_joint_idxs],
                      xlabel='time (s)', ylabel='velocity (m/s)')
    gen_vel_plot.plot(t, vel[:, selected_joint_idxs] - vel_actual[:, selected_joint_idxs],
                      xlabel='time (s)', ylabel='velocity (m/s)', grid=False)
    gen_vel_plot.add_legend(['Desired Velocity', 'Measured Velocity', 'Velocity Error'])
    ylim = gen_vel_plot.fig.gca().get_ylim()
    gen_vel_plot.save_fig('gen_vel_plot.png')
    # ps.save_fig('generalized_velocities_around_impact.png')

    proj_vel_plot = plot_styler.PlotStyler()
    proj_vel_plot.plot(t, vel_proj,
                       title='Constant Impact-Invariant Projection',
                       xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    proj_vel_plot.plot(t, vel_proj_actual,
                       title='Constant Impact-Invariant Projection',
                       xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    proj_vel_plot.save_fig('proj_vel_plot.png')

    corrected_vel_plot = plot_styler.PlotStyler()
    corrected_vel_plot.plot(t, vel_corrected,
                            title='Impact-Invariant Correction',
                            xlabel='time (s)', ylabel='velocity (m/s)',
                            ylim=ylim)
    corrected_vel_plot.save_fig('corrected_vel_plot.png')

    blended_vel_plot = plot_styler.PlotStyler()
    ax = blended_vel_plot.fig.axes[0]
    # blended_vel_plot.plot(t, vel_actual[:, selected_joint_idxs],
    #                       title='Blended Impact-Invariant Correction',
    #                       xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    blended_vel_plot.plot(t, vel_corrected_blend[:, selected_joint_idxs],
                          title='Blended Impact-Invariant Correction',
                          xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    ax.fill_between(t_proj, ylim[0], ylim[1], color=blended_vel_plot.grey,
                    alpha=0.2)
    blended_vel_plot.save_fig('blended_vel_plot.png')



    gen_vel_error = plot_styler.PlotStyler()
    ax = gen_vel_error.fig.axes[0]
    gen_vel_error.plot(t, vel[:, selected_joint_idxs],
                      xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.blue)
    gen_vel_error.plot(t, vel_actual[:, selected_joint_idxs],
                      xlabel='time (s)', ylabel='velocity (m/s)', color=gen_vel_error.red)
    gen_vel_error.plot(t, vel[:, selected_joint_idxs] - vel_actual[:, selected_joint_idxs],
                      xlabel='Time', ylabel='Velocity', grid=False, color=gen_vel_error.grey)
    gen_vel_error.add_legend(['Desired Velocity', 'Measured Velocity', 'Velocity Error'])
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    # ax.spines['bottom'].set_visible(False)
    # ax.spines['left'].set_visible(False)
    ylim = gen_vel_error.fig.gca().get_ylim()
    gen_vel_error.save_fig('gen_vel_error.png')

    projected_vel_error = plot_styler.PlotStyler()
    ax = projected_vel_error.fig.axes[0]
    projected_vel_error.plot(t, vel_proj[:, selected_joint_idxs] - vel_proj_actual[:, selected_joint_idxs],
                                title='Impact-Invariant Projection Error',
                                xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    ax.fill_between(t_proj, ylim[0], ylim[1], color=projected_vel_error.grey,
                    alpha=0.2)
    projected_vel_error.save_fig('projected_vel_error.png')

    corrected_vel_error = plot_styler.PlotStyler()
    ax = corrected_vel_error.fig.axes[0]
    corrected_vel_error.plot(t, vel[:, selected_joint_idxs] - vel_corrected[:, selected_joint_idxs],
                          title='Impact-Invariant Correction Error',
                          xlabel='time (s)', ylabel='velocity (m/s)', ylim=ylim)
    ax.fill_between(t_proj, ylim[0], ylim[1], color=corrected_vel_error.grey,
                    alpha=0.2)
    corrected_vel_error.save_fig('corrected_vel_error.png')

    blending_function_plot = plot_styler.PlotStyler()
    ax = blending_function_plot.fig.axes[0]
    blending_function_plot.plot(t, alphas, title="Blending Function")
    ax.fill_between(t_proj, ax.get_ylim()[0], ax.get_ylim()[1],
                    color=blending_function_plot.grey, alpha=0.2)
    blending_function_plot.save_fig('blending_function_plot.png')

    plt.show()


if __name__ == '__main__':
    main()
