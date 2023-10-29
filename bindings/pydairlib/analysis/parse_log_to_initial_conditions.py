import io
import sys
import lcm
import glob
import numpy as np
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from pydairlib.analysis.process_lcm_log import get_log_data
import pydairlib.analysis.cassie_plotting_utils as cassie_plots
import pydairlib.analysis.mbp_plotting_utils as mbp_plots
from pydairlib.multibody import ReExpressWorldVector3InBodyYawFrame

def nans(shape, dtype=float):
    a = np.empty(shape, dtype)
    a.fill(np.nan)
    return a


def make_footstep_target_global_to_local(pelvis_pose, footstep_target_global):
    target = np.zeros((3,))
    xy = (footstep_target_global[:2] - pelvis_pose.translation().ravel()[:2])
    target[:2] = xy
    target[-1] = footstep_target_global[-1]
    body_x = pelvis_pose.rotation().col(0).ravel()
    yaw = np.arctan2(body_x[1], body_x[0])
    return pelvis_pose.rotation().MakeZRotation(-yaw) @ target


def parse_single_log_to_robot_ds_states(filename, plant, context, channel_x,
                                        channel_u, channel_osc, T_ds, T_step,
                                        osc_cost_thresh):
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = nq + nv
    ''' Read the log '''
    log = lcm.EventLog(filename, "r")
    robot_output, robot_input, osc_debug = \
        get_log_data(
            log,  # log
            cassie_plots.cassie_default_channels,  # lcm channels
            -1, -1,  # start and end times
            mbp_plots.load_default_channels,  # processing callback
            plant, channel_x, channel_u, channel_osc  # callback args
        )

    ''' Find the middle of the first "post left" double stance period'''
    osc_idx_start = np.argwhere(osc_debug['fsm'] == 3).ravel()[0]
    t_start = osc_debug['t_osc'].ravel()[osc_idx_start] + (T_ds / 2.0)

    # Give some margin by subtracting about 20 seconds from the end of the log
    n_steps = int(
        np.floor_divide(
            robot_output['t_x'].ravel()[-1] - t_start, T_step
        )
    ) - int(10.0 / T_step)
    print(f'Processing {n_steps} steps')
    if n_steps < 1:
        return None

    state = np.zeros((n_steps, nx))
    for k in range(n_steps):
        # Find the relevant timestamps
        ti = t_start + T_step * k
        x_idx = np.argwhere(robot_output['t_x'] >= ti)[0]
        osc_idx = np.argwhere(osc_debug['t_osc'] >= ti)[0]

        # Append data only if we have a good OSC solve - heuristic for
        # eliminating failed steps
        if osc_debug['regularization_costs']['soft_constraint_cost'][
            osc_idx] < osc_cost_thresh:
            state[k, :nq] = robot_output['q'][x_idx]
            state[k, nq:] = robot_output['v'][x_idx]
        else:
            state[k] = nans((nx,))
    return state[~np.isnan(state).any(axis=1)]


def parse_single_log_to_perception_learning_initial_condition(
    filename, plant, channel_x, channel_u, channel_osc, Tds, Tss):
    ''' Read the log '''
    log = lcm.EventLog(filename, "r")
    robot_output, _, osc_debug, _ = \
        get_log_data(
            log,  # log
            cassie_plots.cassie_default_channels,  # lcm channels
            -1, -1,  # start and end times
            mbp_plots.load_default_channels,  # processing callback
            plant, plant, channel_x, channel_u, channel_osc  # callback args
        )

    osc_idx_start = np.argwhere(osc_debug['fsm'] == 0).ravel()[0]
    t_start_osc = osc_debug['t_osc'].ravel()[osc_idx_start]

    context = plant.CreateDefaultContext()
    frames, pts = cassie_plots.get_toe_frames_and_points(plant)

    def stance(t):
        periods = int((t - t_start_osc) / (Tss + Tds))
        return 'left' if periods % 2 == 0 else 'right'

    def phase(t):
        phase = (t - t_start_osc) / (Tss + Tds) - \
                int((t - t_start_osc) / (Tss + Tds))
        phase *= (Tss + Tds)
        return phase if phase < Tss else 0

    def swing_foot_pos(t, q):
        plant.SetPositions(context, q)
        points = {
            'left': plant.CalcPointsPositions(
                context, frames['left'].body_frame(), pts['mid'],
                plant.world_frame()
            ).ravel(),
            'right': plant.CalcPointsPositions(
                context, frames['right'].body_frame(), pts['mid'],
                plant.world_frame()
            ).ravel()
        }

        return ReExpressWorldVector3InBodyYawFrame(
            plant=plant,
            context=context,
            body_name="pelvis",
            vec=points[stance(t + (Tss + Tds))] - points[stance(t)]
        )

    data = []
    swing_start = True
    init_swing_pos = np.zeros((3,))
    for i, t in enumerate(robot_output['t_x']):
        if t < t_start_osc:
            continue
        if phase(t) == 0:
            swing_start = True
            continue
        q = robot_output['q'][i].ravel()
        if swing_start:
            init_swing_pos = swing_foot_pos(t, q)
            swing_start = False

        q[4] = 0
        q[5] = 0  # move us back to the origin
        v = robot_output['v'][i].ravel()
        data.append(
            {
                'stance': stance(t),
                'phase': phase(t),
                'initial_swing_foot_pos': init_swing_pos,
                'q': q,
                'v': v
            }
        )
    return data


def parse_single_log_to_learning_data(filename, plant, context, channel_x,
                                      channel_u, channel_osc, T_ds, T_step):
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nx = nq + nv

    # pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(plant)
    # pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(plant)

    ''' Read the log '''
    log = lcm.EventLog(filename, "r")
    robot_output, _, osc_debug, _ = \
        get_log_data(
            log,  # log
            cassie_plots.cassie_default_channels,  # lcm channels
            -1, -1,
            mbp_plots.load_default_channels,  # processing callback
            plant, channel_x, channel_u, channel_osc
        )  # processing callback arguments
    swing_ft_data = osc_debug['osc_debug_tracking_datas']['swing_ft_traj']

    ''' Find the middle of the first "post left" double stance period'''
    osc_idx_start = np.argwhere(osc_debug['fsm'] == 3).ravel()[0]
    t_start = osc_debug['t_osc'].ravel()[osc_idx_start] + (T_ds / 2.0)
    n_steps = int(
        np.floor_divide(
            robot_output['t_x'].ravel()[-1] - t_start, T_step
        )
    ) - 1

    state = np.zeros((n_steps, nx))
    footstep_target = np.zeros((n_steps, 3))
    footstep_error = np.zeros((n_steps,))
    for k in range(n_steps):
        # Find the relevant timestamps
        ti = t_start + T_step * k
        tf = ti + T_step - T_ds / 2.0
        x_idx = np.argwhere(robot_output['t_x'] >= ti)[0]
        x_idx_2 = np.argwhere(robot_output['t_x'] >= tf)[0] - 1
        swing_ft_idx = np.argwhere(swing_ft_data.t >= tf).ravel()[0] - 2
        assert (abs(swing_ft_data.t[swing_ft_idx] - tf) < 0.01)

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
        footstep_target[k] = make_footstep_target_global_to_local(
            pelvis_pose, foostep_target_global
        )

    data = np.hstack((state, footstep_target, footstep_error.reshape(-1, 1)))
    np.save("sample_data.npy", data)


def simulation_main(logpath):
    use_floating_base = True
    use_springs = True

    channel_x = "CASSIE_STATE_SIMULATION"
    channel_u = "CASSIE_INPUT"
    channel_osc = "OSC_DEBUG_WALKING"

    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs
    )

    parse_single_log_to_learning_data(
        logpath, plant, context, channel_x,
        channel_u, channel_osc, 0.1, 0.4
    )


def parse_log_folders_to_ds_states(folder_list, savepath):
    channel_x = "CASSIE_STATE_DISPATCHER"
    channel_u = "OSC_WALKING"
    channel_osc = "OSC_DEBUG_WALKING"

    use_floating_base = True
    use_springs = True

    plant, context = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs
    )

    dataset = []
    for folder in folder_list:
        print(folder)
        lcmlogs = glob.glob(folder + 'lcmlog-*')
        for logpath in lcmlogs:
            try:
                gains = load(
                    io.open(
                        logpath.replace(
                            'lcmlog-', 'walking_gains_alip_'
                        ) + '.yaml', 'r'
                    ),
                    Loader=Loader
                )
            except Exception as e:
                print(f'cant load the gains for {logpath}' + str(e))
                continue

            T_ds = gains['ds_time']
            T_step = T_ds + gains['ss_time']

            try:
                data = parse_single_log_to_robot_ds_states(
                    logpath, plant, context, channel_x, channel_u, channel_osc,
                    T_ds, T_step, 1.0
                )
                if data is not None:
                    dataset.append(data)
            except Exception as e:
                print(f'cant parse the lcm log{logpath}: ' + str(e))
                continue
    np.save(savepath, np.vstack(dataset))


def parse_hardware_main():
    savepath = '.learning_data/hardware_ics_v2.npy'
    folder_list = \
        [
            f'/media/brian/tb2/cassie_backup/logs/cassie_hardware/2022/'
            f'{subfolder}/' for subfolder in ['08_11_22']
        ]
    parse_log_folders_to_ds_states(folder_list, savepath)


def parse_sim_main():
    savepath = 'bindings/pydairlib/perceptive_locomotion/perception_learning' \
               '/tmp/initial_conditions_2.npz'
    logpath = '/home/brian/logs/2023/10_25_23/lcmlog-06'

    channel_x = "CASSIE_STATE_SIMULATION"
    channel_u = "OSC_WALKING"
    channel_osc = "OSC_DEBUG_WALKING"

    use_floating_base = True
    use_springs = True

    plant, _ = cassie_plots.make_plant_and_context(
        floating_base=use_floating_base, springs=use_springs
    )

    Tds = 0.1
    Tss = 0.32
    data = parse_single_log_to_perception_learning_initial_condition(
        logpath, plant, channel_x, channel_u, channel_osc, Tds, Tss
    )
    np.savez(savepath, data)


def main():
    parse_sim_main()


if __name__ == '__main__':
    main()
