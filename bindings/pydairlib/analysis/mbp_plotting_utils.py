import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from pydairlib.common import plot_styler, plotting_utils
from osc_debug import lcmt_osc_tracking_data_t, osc_tracking_cost, osc_regularlization_tracking_cost
from pydairlib.multibody import makeNameToPositionsMap, \
    makeNameToVelocitiesMap, makeNameToActuatorsMap, \
    createStateNameVectorFromMap, createActuatorNameVectorFromMap


def make_name_to_mbp_maps(plant):
    return makeNameToPositionsMap(plant), \
           makeNameToVelocitiesMap(plant), \
           makeNameToActuatorsMap(plant)


def make_mbp_name_vectors(plant):
    x_names = createStateNameVectorFromMap(plant)
    u_names = createActuatorNameVectorFromMap(plant)
    q_names = x_names[:plant.num_positions()]
    v_names = x_names[plant.num_positions():]
    return q_names, v_names, u_names


def make_joint_order_permutation_matrix(names_in_old_order, names_in_new_order):
    n = len(names_in_new_order)
    perm = np.zeros((n, n), dtype=int)
    for i, name in enumerate(names_in_new_order):
        try:
            j = names_in_old_order.index(name)
        except ValueError:
            print(f"Error: {name} not found in old joint ordering")
            raise
        except BaseException as err:
            print(f"Unexpected {err}, {type(err)}")
            raise
        perm[i, j] = 1
    return perm


def make_joint_order_permutations(robot_output_message, plant):
    qnames, vnames, unames = make_mbp_name_vectors(plant)
    qperm = make_joint_order_permutation_matrix(
        robot_output_message.position_names, qnames)
    vperm = make_joint_order_permutation_matrix(
        robot_output_message.velocity_names, vnames)
    uperm = make_joint_order_permutation_matrix(
        robot_output_message.effort_names, unames)
    return qperm, vperm, uperm


def process_state_channel(state_data, plant):
    t_x = []
    q = []
    u = []
    v = []

    pos_map = makeNameToPositionsMap(plant)
    vel_map = makeNameToVelocitiesMap(plant)
    act_map = makeNameToActuatorsMap(plant)

    for msg in state_data:
        q_temp = [[] for i in range(len(msg.position))]
        v_temp = [[] for i in range(len(msg.velocity))]
        u_temp = [[] for i in range(len(msg.effort))]
        for i in range(len(q_temp)):
            q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
        for i in range(len(v_temp)):
            v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
        for i in range(len(u_temp)):
            u_temp[act_map[msg.effort_names[i]]] = msg.effort[i]
        q.append(q_temp)
        v.append(v_temp)
        u.append(u_temp)
        t_x.append(msg.utime / 1e6)

    return {'t_x': np.array(t_x),
            'q': np.array(q),
            'v': np.array(v),
            'u': np.array(u)}


def process_effort_channel(data, plant):
    u = []
    t = []

    act_map = makeNameToActuatorsMap(plant)
    for msg in data:
        u_temp = [[] for i in range(len(msg.efforts))]
        for i in range(len(u_temp)):
            u_temp[act_map[msg.effort_names[i]]] = msg.efforts[i]
        t.append(msg.utime / 1e6)
        u.append(u_temp)

    return {'t_u': np.array(t), 'u': np.array(u)}


def make_point_positions_from_q(
    q, plant, context, frame, pt_on_frame, frame_to_calc_position_in=None):
    if frame_to_calc_position_in is None:
        frame_to_calc_position_in = plant.world_frame()

    pos = np.zeros((q.shape[0], 3))
    for i, generalized_pos in enumerate(q):
        plant.SetPositions(context, generalized_pos)
        pos[i] = plant.CalcPointsPositions(context, frame, pt_on_frame,
                                           frame_to_calc_position_in).ravel()

    return pos


def process_osc_channel(data):
    t_osc = []
    # input_cost = []
    # accel_cost = []
    # soft_constraint_cost = []
    regularization_costs = osc_regularlization_tracking_cost(data[0].regularization_cost_names)
    qp_solve_time = []
    u_sol = []
    lambda_c_sol = []
    lambda_h_sol = []
    dv_sol = []
    epsilon_sol = []
    osc_output = []
    fsm = []
    osc_debug_tracking_datas = {}

    for msg in data:
        t_osc.append(msg.utime / 1e6)
        regularization_costs.append(msg.regularization_cost_names, msg.regularization_costs)
        qp_solve_time.append(msg.qp_output.solve_time)
        u_sol.append(msg.qp_output.u_sol)
        lambda_c_sol.append(msg.qp_output.lambda_c_sol)
        lambda_h_sol.append(msg.qp_output.lambda_h_sol)
        dv_sol.append(msg.qp_output.dv_sol)
        epsilon_sol.append(msg.qp_output.epsilon_sol)

        osc_output.append(msg)
        for tracking_data in msg.tracking_data:
            if tracking_data.name not in osc_debug_tracking_datas:
                osc_debug_tracking_datas[tracking_data.name] = \
                    lcmt_osc_tracking_data_t()
            osc_debug_tracking_datas[tracking_data.name].append(
                tracking_data, msg.utime / 1e6)

        fsm.append(msg.fsm_state)

    tracking_cost_handler = osc_tracking_cost(osc_debug_tracking_datas.keys())
    for msg in data:
        tracking_cost_handler.append(msg.tracking_data_names, msg.tracking_costs)
    tracking_cost = tracking_cost_handler.convertToNP()

    for name in osc_debug_tracking_datas:
        osc_debug_tracking_datas[name].convertToNP()

    return {'t_osc': np.array(t_osc),
            'regularization_costs': regularization_costs,
            'qp_solve_time': np.array(qp_solve_time),
            'u_sol': np.array(u_sol),
            'lambda_c_sol': np.array(lambda_c_sol),
            'lambda_h_sol': np.array(lambda_h_sol),
            'dv_sol': np.array(dv_sol),
            'epsilon_sol': np.array(epsilon_sol),
            'tracking_cost': tracking_cost,
            'osc_debug_tracking_datas': osc_debug_tracking_datas,
            'fsm': np.array(fsm),
            'osc_output': osc_output}


def permute_osc_joint_ordering(osc_data, robot_output_msg, plant):
    _, vperm, uperm = make_joint_order_permutations(robot_output_msg, plant)
    osc_data['u_sol'] = (osc_data['u_sol'] @ uperm.T)
    osc_data['dv_sol'] = (osc_data['dv_sol'] @ vperm.T)
    return osc_data


def load_default_channels(data, plant, state_channel, input_channel,
                          osc_debug_channel):
    robot_output = process_state_channel(data[state_channel], plant)
    robot_input = process_effort_channel(data[input_channel], plant)
    osc_debug = process_osc_channel(data[osc_debug_channel])
    osc_debug = permute_osc_joint_ordering(
        osc_debug, data[state_channel][0], plant)

    return robot_output, robot_input, osc_debug


def plot_q_or_v_or_u(
    robot_output, key, x_names, x_slice, time_slice,
    ylabel=None, title=None):
    ps = plot_styler.PlotStyler()
    if ylabel is None:
        ylabel = key
    if title is None:
        title = key

    plotting_utils.make_plot(
        robot_output,  # data dict
        't_x',  # time channel
        time_slice,
        [key],  # key to plot
        {key: x_slice},  # slice of key to plot
        {key: x_names},  # legend entries
        {'xlabel': 'Time',
         'ylabel': ylabel,
         'title': title}, ps)
    return ps

def plot_u_cmd(
    robot_input, key, x_names, x_slice, time_slice,
    ylabel=None, title=None):
    ps = plot_styler.PlotStyler()
    if ylabel is None:
        ylabel = key
    if title is None:
        title = key

    plotting_utils.make_plot(
        robot_input,  # data dict
        't_u',  # time channel
        time_slice,
        [key],  # key to plot
        {key: x_slice},  # slice of key to plot
        {key: x_names},  # legend entries
        {'xlabel': 'Time',
         'ylabel': ylabel,
         'title': title}, ps)
    return ps


def plot_floating_base_positions(robot_output, q_names, fb_dim, time_slice):
    return plot_q_or_v_or_u(robot_output, 'q', q_names[:fb_dim], slice(fb_dim),
                            time_slice, ylabel='Position',
                            title='Floating Base Positions')


def plot_joint_positions(robot_output, q_names, fb_dim, time_slice):
    q_slice = slice(fb_dim, len(q_names))
    return plot_q_or_v_or_u(robot_output, 'q', q_names[q_slice], q_slice,
                            time_slice, ylabel='Joint Angle (rad)',
                            title='Joint Positions')


def plot_positions_by_name(robot_output, q_names, time_slice, pos_map):
    q_slice = [pos_map[name] for name in q_names]
    return plot_q_or_v_or_u(robot_output, 'q', q_names, q_slice, time_slice,
                            ylabel='Position', title='Select Positions')


def plot_floating_base_velocities(robot_output, v_names, fb_dim, time_slice):
    return plot_q_or_v_or_u(robot_output, 'v', v_names[:fb_dim], slice(fb_dim),
                            time_slice, ylabel='Velocity',
                            title='Floating Base Velocities')


def plot_joint_velocities(robot_output, v_names, fb_dim, time_slice):
    q_slice = slice(fb_dim, len(v_names))
    return plot_q_or_v_or_u(robot_output, 'v', v_names[q_slice], q_slice,
                            time_slice, ylabel='Joint Vel (rad/s)',
                            title='Joint Velocities')


def plot_velocities_by_name(robot_output, v_names, time_slice, vel_map):
    v_slice = [vel_map[name] for name in v_names]
    return plot_q_or_v_or_u(robot_output, 'v', v_names, v_slice, time_slice,
                            ylabel='Velocity', title='Select Velocities')


def plot_measured_efforts(robot_output, u_names, time_slice):
    return plot_q_or_v_or_u(robot_output, 'u', u_names, slice(len(u_names)),
                            time_slice, ylabel='Efforts (Nm)',
                            title='Joint Efforts')

def plot_commanded_efforts(robot_input, u_names, time_slice):
    return plot_u_cmd(robot_input, 'u', u_names, slice(len(u_names)),
                            time_slice, ylabel='Efforts (Nm)',
                            title='Commanded Joint Efforts')


def plot_measured_efforts_by_name(robot_output, u_names, time_slice, u_map):
    u_slice = [u_map[name] for name in u_names]
    return plot_q_or_v_or_u(robot_output, 'u', u_names, u_slice, time_slice,
                            ylabel='Efforts (Nm)', title='Select Joint Efforts')


def plot_points_positions(robot_output, time_slice, plant, context, frame_names,
                          pts, dims):
    dim_map = ['_x', '_y', '_z']
    data_dict = {'t': robot_output['t_x']}
    legend_entries = {}
    for name in frame_names:
        frame = plant.GetBodyByName(name).body_frame()
        pt = pts[name]
        data_dict[name] = make_point_positions_from_q(robot_output['q'],
                                                      plant, context, frame, pt)
        legend_entries[name] = [name + dim_map[dim] for dim in dims[name]]

    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        data_dict,
        't',
        time_slice,
        frame_names,
        dims,
        legend_entries,
        {'title': 'Running',
         'xlabel': 'time (s)',
         'ylabel': 'foot vertical position (m)'}, ps)

    import matplotlib
    legend_elements = [matplotlib.patches.Patch(facecolor=ps.cmap(0), alpha=0.3, label='Left Stance (LS)'),
                       matplotlib.patches.Patch(facecolor=ps.cmap(4), alpha=0.3, label='Left Liftoff (LF)'),
                       matplotlib.patches.Patch(facecolor=ps.cmap(2), alpha=0.3, label='Right Stance (RS)'),
                       matplotlib.patches.Patch(facecolor=ps.cmap(6), alpha=0.3, label='Right Liftoff (RF)')]
    legend = plt.legend(legend_elements, ['Left Stance (LS)',
                                          'Left Liftoff (LF)',
                                          'Right Stance (RS)',
                                          'Right Liftoff (RF)'], loc=1)
    plt.gca().add_artist(legend)

    return ps


def plot_tracking_costs(osc_debug, time_slice):
    ps = plot_styler.PlotStyler()
    data_dict = \
        {key: val for key, val in osc_debug['tracking_cost'].items()}
    data_dict['t_osc'] = osc_debug['t_osc']

    plotting_utils.make_plot(
        data_dict,
        't_osc',
        time_slice,
        osc_debug['tracking_cost'].keys(),
        {},
        {key: [key] for key in osc_debug['tracking_cost'].keys()},
        {'xlabel': 'Time',
         'ylabel': 'Cost',
         'title': 'tracking_costs'}, ps)
    return ps


def plot_general_osc_tracking_data(traj_name, deriv, dim, data, time_slice):
    ps = plot_styler.PlotStyler()
    keys = [key for key in data.keys() if key != 't']
    plotting_utils.make_plot(
        data,
        't',
        time_slice,
        keys,
        {},
        {key: [key] for key in keys},
        {'xlabel': 'Time',
         'ylabel': '',
         'title': f'{traj_name} {deriv} tracking {dim}'}, ps)
    return ps


def plot_osc_tracking_data(osc_debug, traj, dim, deriv, time_slice):
    tracking_data = osc_debug['osc_debug_tracking_datas'][traj]
    data = {}
    if deriv == 'pos':
        data['y_des'] = tracking_data.y_des[:, dim]
        data['y'] = tracking_data.y[:, dim]
        data['error_y'] = tracking_data.error_y[:, dim]
    elif deriv == 'vel':
        data['ydot_des'] = tracking_data.ydot_des[:, dim]
        data['ydot'] = tracking_data.ydot[:, dim]
        data['error_ydot'] = tracking_data.ydot_des[:, dim] - tracking_data.ydot[:, dim]
        data['projected_error_ydot'] = tracking_data.error_ydot[:, dim]
    elif deriv == 'accel':
        data['yddot_des'] = tracking_data.yddot_des[:, dim]
        data['yddot_command'] = tracking_data.yddot_command[:, dim]
        data['yddot_command_sol'] = tracking_data.yddot_command_sol[:, dim]

    data['t'] = tracking_data.t
    return plot_general_osc_tracking_data(traj, deriv, dim, data, time_slice)


def plot_qp_costs(osc_debug, time_slice):
    # cost_keys = ['input_cost', 'acceleration_cost',
    #              'soft_constraint_cost']
    # ps = plot_styler.PlotStyler()
    # plotting_utils.make_plot(
    #     osc_debug,
    #     't_osc',
    #     time_slice,
    #     cost_keys,
    #     {},
    #     {key: [key] for key in cost_keys},
    #     {'xlabel': 'Time',
    #      'ylabel': 'Cost',
    #      'title': 'OSC QP Costs'}, ps)
    # return ps
    ps = plot_styler.PlotStyler()
    regularization_cost = osc_debug['regularization_costs'].regularization_costs
    data_dict = \
        {key: val for key, val in regularization_cost.items()}
    data_dict['t_osc'] = osc_debug['t_osc']

    plotting_utils.make_plot(
        data_dict,
        't_osc',
        time_slice,
        regularization_cost.keys(),
        {},
        {key: [key] for key in regularization_cost.keys()},
        {'xlabel': 'Time',
         'ylabel': 'Cost',
         'title': 'regularization_costs'}, ps)
    return ps


def plot_qp_solve_time(osc_debug, time_slice):
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        ['qp_solve_time'],
        {},
        {},
        {'xlabel': 'Timestamp',
         'ylabel': 'Solve Time ',
         'title': 'OSC QP Solve Time'}, ps)
    return ps


def plot_lambda_c_sol(osc_debug, time_slice, lambda_slice):
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        ['lambda_c_sol'],
        {'lambda_c_sol': lambda_slice},
        {'lambda_c_sol': ['lambda_c_' + i for i in
                          plotting_utils.slice_to_string_list(lambda_slice)]},
        {'xlabel': 'time',
         'ylabel': 'lambda',
         'title': 'OSC contact force solution'}, ps)
    return ps


def plot_epsilon_sol(osc_debug, time_slice, epsilon_slice):
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        ['epsilon_sol'],
        {'epsilon_sol': epsilon_slice},
        {'epsilon_sol': ['epsilon_sol' + i for i in
                         plotting_utils.slice_to_string_list(epsilon_slice)]},
        {'xlabel': 'time',
         'ylabel': 'epsilon',
         'title': 'OSC soft constraint epsilon sol'}, ps)
    return ps


def add_fsm_to_plot(ps, fsm_time, fsm_signal, fsm_state_names = []):
    ax = ps.fig.axes[0]
    ymin, ymax = ax.get_ylim()


    # uses default color map
    legend_elements = []
    for i in np.unique(fsm_signal):
        ax.fill_between(fsm_time, ymin, ymax, where=(fsm_signal == i), color=ps.cmap(2*i), alpha=0.2)
        if len(fsm_state_names) == np.unique(fsm_signal).shape[0]:
            legend_elements.append(Patch(facecolor=ps.cmap(2*i), alpha=0.3, label=fsm_state_names[i]))

    # if len(legend_elements) > 0:
    #     legend = ax.legend(legend_elements, fsm_state_names, loc=4)
        # ps.add_legend(legend, loc=4)
        # ax.add_artist(legend)
        # ax.add_artist(legend)
        # ax.relim()
