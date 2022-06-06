import sys
import lcm
import matplotlib.pyplot as plt
import code
import numpy as np

import dairlib
from process_lcm_log import get_log_data
import cassie_plotting_utils as cassie_plots
import mbp_plotting_utils as mbp_plots


def make_foostep_target_global_to_local(pelvis_pose, footstep_target_global):
    target = np.zeros((3,))
    xy = (footstep_target_global[:2] - pelvis_pose.translation().ravel()[:2])
    target[:2] = xy
    target[-1] = footstep_target_global[-1]
    body_x = pelvis_pose.rotation().col(0).ravel()
    yaw = np.arctan2(body_x[1], body_x[0])
    return pelvis_pose.rotation().MakeZRotation(-yaw) @ target


def main():
    use_floating_base = True
    use_springs = True

    channel_x = "CASSIE_STATE_SIMULATION"
    channel_u = "CASSIE_INPUT"
    channel_osc = "OSC_DEBUG_WALKING"

    ds_period = 0.1
    step_period = 0.4

    nq = 23
    nv = 22
    nx = nq + nv

    ''' Get the plant '''
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs)
    pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    ''' Read the log '''
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input, osc_debug = \
        get_log_data(log,  # log
            cassie_plots.cassie_default_channels,  # lcm channels
            -1,
            mbp_plots.load_default_channels,  # processing callback
            plant, channel_x, channel_u, channel_osc)  # processing callback arguments
    swing_ft_data = osc_debug['osc_debug_tracking_datas']['swing_ft_traj']

    ''' Find the middle of the first "post left" double stance period'''
    osc_idx_start = np.argwhere(osc_debug['fsm'] == 3).ravel()[0]
    t_start = osc_debug['t_osc'].ravel()[osc_idx_start] + (ds_period/2.0)
    n_steps = int(np.floor_divide(
                  robot_output['t_x'].ravel()[-1] - t_start, step_period)) - 1

    state = np.zeros((n_steps, nx))
    footstep_target = np.zeros((n_steps, 3))
    footstep_error = np.zeros((n_steps,))
    for k in range(n_steps):
        # Find the relevant timestamps
        ti = t_start + step_period * k
        tf = ti + step_period - ds_period / 2.0
        x_idx = np.argwhere(robot_output['t_x'] >= ti)[0]
        x_idx_2 = np.argwhere(robot_output['t_x'] >= tf)[0] - 1
        swing_ft_idx = np.argwhere(swing_ft_data.t >= tf).ravel()[0] - 2
        assert(abs(swing_ft_data.t[swing_ft_idx] - tf) < 0.01)

        # Append data
        state[k, :nq] = robot_output['q'][x_idx]
        state[k, nq:] = robot_output['v'][x_idx]
        footstep_error[k] = np.linalg.norm(swing_ft_data.error_y[swing_ft_idx])

        plant.SetPositions(context, robot_output['q'][x_idx_2].ravel())
        plant.SetVelocities(context, robot_output['v'][x_idx_2].ravel())
        com = plant.CalcCenterOfMassPositionInWorld(context).ravel()
        foostep_target_global = com + swing_ft_data.y_des[swing_ft_idx]

        plant.SetPositionsAndVelocities(context, state[k])
        pelvis_pose = plant.GetBodyByName("pelvis").EvalPoseInWorld(context)
        footstep_target[k] = make_foostep_target_global_to_local(
            pelvis_pose, foostep_target_global)

    data = np.hstack((state, footstep_target, footstep_error.reshape(-1, 1)))
    np.save("sample_data.npy", data)


if __name__ == '__main__':
    main()
