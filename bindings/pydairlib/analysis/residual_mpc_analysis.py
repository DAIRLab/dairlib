import pdb

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

from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import LeftLoopClosureEvaluator, RightLoopClosureEvaluator
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.math import RollPitchYaw as RPY


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
        for forces in lambdas[key]:
            lambdas[key] = np.array(forces)
    lambda_l = np.hstack(lambdas["toe_left"])
    lambda_r = np.hstack(lambdas["toe_right"])
    return {'t_lambda': np.array(tl),
            'lambda_toe_left': lambda_l, 'lambda_toe_right': lambda_r}


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

    srbd_data = get_x_and_xdot_from_plant_data(robot_output, vdot, plant, context)

    plt.show()
    # Define x time slice
    t_x_slice = slice(robot_output['t_x'].size)
    t_osc_slice = slice(osc_debug['t_osc'].size)


if __name__ == '__main__':
    main()
