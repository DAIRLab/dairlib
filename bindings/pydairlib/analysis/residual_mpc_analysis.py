import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
import drake
from process_lcm_log import get_log_data
from cassie_plot_config import CassiePlotConfig
import cassie_plotting_utils as cassie_plots
import mbp_plotting_utils as mbp_plots
import mpc_debug as mpc

from pydairlib.common import plot_styler, plotting_utils
from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import LeftLoopClosureEvaluator, RightLoopClosureEvaluator
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.math import RollPitchYaw as RPY
import examples.Cassie.lstsq_srb_estimator as lstsq


def get_x_and_xdot_from_plant_data(robot_output, vdot, plant, context):
    nx = 12
    x = np.zeros((robot_output['t_x'].shape[0], nx))
    xdot = np.zeros(x.shape)
    world = plant.world_frame()
    pelvis = plant.GetBodyByName("pelvis")
    for i in range(robot_output['t_x'].size):
        plant.SetPositions(context, robot_output['q'][i])
        plant.SetVelocities(context, robot_output['v'][i])
        p_com = plant.CalcCenterOfMassPositionInWorld(context)
        theta_pelvis = RPY(
            plant.EvalBodyPoseInWorld(context, pelvis).rotation()).vector()
        omega_pelvis = plant.EvalBodySpatialVelocityInWorld(context,
                                                            pelvis).rotational()
        J_com = plant.CalcJacobianCenterOfMassTranslationalVelocity(
            context, JacobianWrtVariable.kV, world, world)
        v_com = J_com @ robot_output['v'][i]
        Jdot_v_com = plant.CalcBiasCenterOfMassTranslationalAcceleration(
            context, JacobianWrtVariable.kV, world, world)
        J_pelvis = plant.CalcJacobianSpatialVelocity(
            context, JacobianWrtVariable.kV, pelvis.body_frame(),
            np.zeros((3,)), world, world)[:3, :]
        Jdot_v_pelvis = plant.CalcBiasSpatialAcceleration(
            context, JacobianWrtVariable.kV, pelvis.body_frame(),
            np.zeros((3,)), world, world).rotational()
        a_com = Jdot_v_com + J_com @ vdot['vdot'][i]
        a_pelvis = Jdot_v_pelvis + J_pelvis @ vdot['vdot'][i]

        x[i] = np.vstack((p_com, theta_pelvis, v_com, omega_pelvis)).ravel()
        xdot[i] = np.vstack((v_com, omega_pelvis, a_com, a_pelvis)).ravel()

    return {'t_x': robot_output['t_x'], 'x': x, 'xdot': xdot}


def process_vdot_channel(data, vdot_channel):
    t = []
    vd = []
    for msg in data[vdot_channel]:
        t.append(msg.utime / 1e6)
        vd.append(msg.value)
    return {'t_vdot': np.array(t), 'vdot': np.array(vd)}


def process_lambda_channel(data, contact_channel):
    tl = []
    lambdas = {'toe_left': ([], []), 'toe_right': ([], [])}

    for msg in data[contact_channel]:
        tl.append(msg.timestamp / 1e6)
        force_count = {'toe_left': 0, 'toe_right': 0}
        for pp_info in msg.point_pair_contact_info:
            toe_name = pp_info.body2_name.split('(')[0]
            lambdas[toe_name][force_count[toe_name]].append(pp_info.contact_force)
            force_count[toe_name] = force_count[toe_name] + 1
        for key in lambdas:
            while force_count[key] < 2:
                lambdas[key][force_count[key]].append([0.0 for _ in range(3)])
                force_count[key] = force_count[key] + 1
    for key in lambdas:
        lambdas[key] = np.hstack(
            (np.array(lambdas[key][0]), np.array(lambdas[key][1])))

    return {'t_lambda': np.array(tl),
            'lambda_toe_left': lambdas['toe_left'],
            'lambda_toe_right': lambdas['toe_right']}


def get_srb_input_traj(lambda_c, osc_debug):
    fsm_values = np.interp(lambda_c['t_lambda'],
                           osc_debug['t_osc'],
                           osc_debug['fsm'])
    l = 0.112/2.0
    u = np.zeros((lambda_c['t_lambda'].size, 5))
    for i in range(u.shape[0]):
        if fsm_values[i] == 0 or fsm_values[i] == 3:
            key = 'lambda_toe_left'
        else:
            key = 'lambda_toe_right'

        u[i, :3] = lambda_c[key][i, :3] + lambda_c[key][i, 3:]
        u[i, 3] = l * (lambda_c[key][i, 1] - lambda_c[key][i, 4])
        u[i, 4] = l * (lambda_c[key][i, 5] - lambda_c[key][i, 2])

    return {'t_u': lambda_c['t_lambda'], 'u': u}


def get_srbd_modes(robot_output, osc_debug):
    fsm_values = np.interp(robot_output['t_x'],
                           osc_debug['t_osc'], osc_debug['fsm'])
    modes = [ 0 if (mode == 0 or mode == 3)  else 1 for mode in fsm_values]
    return {'t': robot_output['t_x'], 'mode': modes}


def get_srb_stance_locations(robot_output, osc_debug, plant, context):
    frames, pts = cassie_plots.get_toe_frames_and_points(plant)
    mid = pts['mid']
    fsm_values = np.interp(robot_output['t_x'],
                           osc_debug['t_osc'],
                           osc_debug['fsm'])
    p = np.zeros((robot_output['t_x'].size, 3))
    left_foot_pos = mbp_plots.make_point_positions_from_q(
        robot_output['q'], plant, context, frames['left'].body_frame(), mid)
    right_foot_pos = mbp_plots.make_point_positions_from_q(
        robot_output['q'], plant, context, frames['right'].body_frame(), mid)

    for i in range(p.shape[0]):
        if fsm_values[i] == 0 or fsm_values[i] == 3:
            p[i] = left_foot_pos[i]
        else:
            p[i] = right_foot_pos[i]
    return {'t_p': robot_output['t_x'],  'p': p}


def calc_residual_smoothness(srb_data, srb_input, srb_stance,
                             modes, dynamics, T, N):
    states_and_steps = np.hstack((srb_data['x'], srb_stance['p']))
    res_norm_sparse = {'A': [], 'B': [], 'b': []}
    res_norm_dense = {'A': [], 'B': [], 'b': []}
    prev_sparse = lstsq.sparse_residual_estimator(
        states_and_steps[:T],
        srb_input['u'][:T],
        srb_data['xdot'][:T],
        modes[:T],
        dynamics
    )
    prev_dense = lstsq.dense_residual_estimator(
        states_and_steps[:T],
        srb_input['u'][:T],
        srb_data['xdot'][:T],
        modes[:T],
        dynamics
    )
    for i in range(1, N-T):
        if not i % 10:
            print(i)
        sparse_residual_dynamics = lstsq.sparse_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics
        )
        dense_residual_dynamics = lstsq.dense_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics
        )
        res_norm_sparse['A'].append(np.linalg.norm(sparse_residual_dynamics.A -
                                                   prev_sparse.A, ord="fro"))
        res_norm_sparse['B'].append(np.linalg.norm(sparse_residual_dynamics.B -
                                                   prev_sparse.B, ord="fro"))
        res_norm_sparse['b'].append(np.linalg.norm(sparse_residual_dynamics.b -
                                                   prev_sparse.b))
        res_norm_dense['A'].append(np.linalg.norm(dense_residual_dynamics.A -
                                                  prev_dense.A, ord="fro"))
        res_norm_dense['B'].append(np.linalg.norm(dense_residual_dynamics.B -
                                                  prev_dense.B, ord="fro"))
        res_norm_dense['b'].append(np.linalg.norm(dense_residual_dynamics.b -
                                                  prev_dense.b))
        prev_sparse = sparse_residual_dynamics
        prev_dense = dense_residual_dynamics

    return {'sparse': res_norm_sparse,
            'dense': res_norm_dense}


def calc_residual_norms(srb_data, srb_input, srb_stance,
                        modes, dynamics, T, N):
    states_and_steps = np.hstack((srb_data['x'], srb_stance['p']))
    res_norm_sparse = {'A': [], 'B': [], 'b': []}
    res_norm_dense = {'A': [], 'B': [], 'b': []}
    for i in range(N-T):
        if not i % 10:
            print(i)
        sparse_residual_dynamics = lstsq.sparse_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics
        )
        dense_residual_dynamics = lstsq.dense_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics
        )
        res_norm_sparse['A'].append(np.linalg.norm(sparse_residual_dynamics.A, ord="fro"))
        res_norm_sparse['B'].append(np.linalg.norm(sparse_residual_dynamics.B, ord="fro"))
        res_norm_sparse['b'].append(np.linalg.norm(sparse_residual_dynamics.b))
        res_norm_dense['A'].append(np.linalg.norm(dense_residual_dynamics.A, ord="fro"))
        res_norm_dense['B'].append(np.linalg.norm(dense_residual_dynamics.B, ord="fro"))
        res_norm_dense['b'].append(np.linalg.norm(dense_residual_dynamics.b))

    return {'sparse': res_norm_sparse,
            'dense': res_norm_dense}


def calc_error_norms(srb_data, srb_input, srb_stance, modes, dynamics, T, N):
    states_and_steps = np.hstack((srb_data['x'], srb_stance['p']))
    error_norm_nominal = []
    error_norm_sparse_residual = []
    error_norm_dense_residual = []
    timestamps = []
    for i in range(T):
        nominal_dynamics = dynamics[modes[i+T]].forward(states_and_steps[i],
                                                        srb_input['u'][i])
        actual_dynamics = srb_data['xdot'][i]
        error_norm_sparse_residual.append(
            np.linalg.norm(nominal_dynamics - actual_dynamics))
        error_norm_dense_residual.append(
            np.linalg.norm(nominal_dynamics - actual_dynamics))
        error_norm_nominal.append(
            np.linalg.norm(nominal_dynamics - actual_dynamics))
        timestamps.append(srb_data['t_x'][i])

    for i in range(N-T):
        if not i % 10:
            print(i)
        sparse_residual_dynamics = lstsq.sparse_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics, reg=0.2
        )
        dense_residual_dynamics = lstsq.dense_residual_estimator(
            states_and_steps[i:i+T],
            srb_input['u'][i:i+T],
            srb_data['xdot'][i:i+T],
            modes[i:i+T],
            dynamics, reg=0.2
        )
        actual_deriv = srb_data['xdot'][i+T]
        nominal_deriv = dynamics[modes[i+T]].forward(
            states_and_steps[i+T], srb_input['u'][i+T])
        sparse_residual_deriv = sparse_residual_dynamics.forward(
            states_and_steps[i+T], srb_input['u'][i+T])
        dense_residual_deriv = dense_residual_dynamics.forward(
            states_and_steps[i+T], srb_input['u'][i+T])

        error_norm_nominal.append(np.linalg.norm(nominal_deriv - actual_deriv))
        error_norm_dense_residual.append(
            np.linalg.norm((nominal_deriv+dense_residual_deriv) - actual_deriv))
        error_norm_sparse_residual.append(
            np.linalg.norm((nominal_deriv + sparse_residual_deriv) - actual_deriv))
        timestamps.append(srb_data['t_x'][i+T])

    return {'t': timestamps,
            'nominal': np.array(error_norm_nominal),
            'sparse':  np.array(error_norm_sparse_residual),
            'dense':  np.array(error_norm_dense_residual)}


def load_srb_dynamics():
    dirname = "/home/brian/workspace/srb_dynamics/"
    Al = np.loadtxt(dirname + "Al.csv", delimiter=',')
    Bl = np.loadtxt(dirname + "Bl.csv", delimiter=',')
    bl = np.loadtxt(dirname + "bl.csv", delimiter=',')
    Ar = np.loadtxt(dirname + "Ar.csv", delimiter=',')
    Br = np.loadtxt(dirname + "Br.csv", delimiter=',')
    br = np.loadtxt(dirname + "br.csv", delimiter=',')
    return [lstsq.LinearDynamics(Al, Bl, bl), lstsq.LinearDynamics(Ar, Br, br)]


def main():
    config_file = \
        'bindings/pydairlib/analysis/plot_configs/cassie_residual_mpc_plot.yaml'
    plot_config = CassiePlotConfig(config_file)

    use_floating_base = plot_config.use_floating_base
    use_springs = plot_config.use_springs

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc
    # channel_mpc = plot_config.channel_mpc
    channel_lambda = {"CASSIE_CONTACT_DRAKE":
                      drake.lcmt_contact_results_for_viz}
    channel_vdot = {"CASSIE_VDOT":
                    drake.lcmt_scope}

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)
    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)
    I_srbd = np.diag([0.91, 0.55, 0.89])

    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input, osc_debug = \
        get_log_data(log,                                       # log
                     cassie_plots.cassie_default_channels,      # lcm channels
                     mbp_plots.load_default_channels,           # processing callback
                     plant, channel_x, channel_u, channel_osc)  # processing callback arguments
    # mpc_data = get_log_data(log,
    #                         {channel_mpc: dairlib.lcmt_saved_traj},
    #                         mpc.process_mpc_channel, channel_mpc)

    toe_forces = get_log_data(log, channel_lambda,
                              process_lambda_channel, "CASSIE_CONTACT_DRAKE")
    vdot = get_log_data(log, channel_vdot,
                        process_vdot_channel, "CASSIE_VDOT")

    srbd_data = get_x_and_xdot_from_plant_data(robot_output, vdot,
                                               plant, context)
    srbd_input = get_srb_input_traj(toe_forces, osc_debug)
    srbd_stance = get_srb_stance_locations(robot_output, osc_debug,
                                           plant, context)

    T = 210
    N = 2800
    dynamics = load_srb_dynamics()
    modes = get_srbd_modes(robot_output, osc_debug)
    error_norms = calc_error_norms(srbd_data, srbd_input, srbd_stance,
                                   modes['mode'], dynamics, T, N)
    # delta_norms = calc_residual_smoothness(srbd_data, srbd_input, srbd_stance,
    #                                        modes['mode'], dynamics, T, N)
    # res_norms = calc_residual_norms(srbd_data, srbd_input, srbd_stance,
    #                                 modes['mode'], dynamics, T, N)

    error_norm_keys = ['nominal', 'sparse', 'dense']
    error_norm_legend = ['Nominal', 'Sparse Residual', 'Dense Residual']
    ps1 = plot_styler.PlotStyler()
    ps1.set_default_styling('/home/brian/classes/ese618hw/')
    for key in error_norm_keys:
        ps1.plot(error_norms['t'], error_norms[key])
    ps1.add_legend(error_norm_legend)
    plt.xlabel('Time (s)')
    plt.ylabel('$|| \dot{x} - \dot{x}_{model} ||_{2}$')
    plt.title('Dynamics error over 4 steps under closed loop MPC control')

    res_norm_keys = ['sparse', 'dense']

    plt.show()

if __name__ == '__main__':
    main()
