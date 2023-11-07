import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from pydrake.all import JacobianWrtVariable, JointActuatorIndex, JointIndex

from bindings.pydairlib.common import plot_styler, plotting_utils
from bindings.pydairlib.analysis.osc_debug import lcmt_osc_tracking_data_t, \
    osc_tracking_cost, osc_regularlization_tracking_cost
from bindings.pydairlib.multibody import MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
    CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap

OSC_DERIV_UNITS = {'pos': 'Position $(m)$',
                   'vel': 'Velocity $(m/s)$',
                   'accel': 'Acceleration $(m/s^2)$'}


def make_name_to_mbp_maps(plant):
    return MakeNameToPositionsMap(plant), \
           MakeNameToVelocitiesMap(plant), \
           MakeNameToActuatorsMap(plant)


def make_mbp_name_vectors(plant):
    x_names = CreateStateNameVectorFromMap(plant)
    u_names = CreateActuatorNameVectorFromMap(plant)
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

    pos_map = MakeNameToPositionsMap(plant)
    vel_map = MakeNameToVelocitiesMap(plant)
    act_map = MakeNameToActuatorsMap(plant)

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

    act_map = MakeNameToActuatorsMap(plant)
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


def make_point_velocities_from_qv(
    q, v, plant, context, frame, pt_on_frame, frame_measured_in=None,
    frame_expressed_in=None):
    if frame_measured_in is None:
        frame_measured_in = plant.world_frame()
    if frame_expressed_in is None:
        frame_expressed_in = plant.world_frame()

    vel = np.zeros((v.shape[0], 3))
    for i, generalized_pos_vel in enumerate(zip(q, v)):
        plant.SetPositions(context, generalized_pos_vel[0])
        J = plant.CalcJacobianTranslationalVelocity(context,
                                                    JacobianWrtVariable.kV,
                                                    frame, pt_on_frame,
                                                    frame_measured_in,
                                                    frame_expressed_in)
        vel[i] = J @ generalized_pos_vel[1]

    return vel


def get_floating_base_velocity_in_body_frame(
    robot_output, plant, context, fb_frame):
    vel = np.zeros((robot_output['q'].shape[0], 3))
    for i, (q, v) in enumerate(zip(robot_output['q'], robot_output['v'])):
        plant.SetPositions(context, q)
        plant.SetVelocities(context, v)
        vel[i] = fb_frame.CalcSpatialVelocity(
            context, plant.world_frame(), fb_frame).translational()

    return vel


def process_osc_channel(data):
    t_osc = []
    if hasattr(data[0], 'regularization_cost_names'):
        regularization_costs = osc_regularlization_tracking_cost(
            data[0].regularization_cost_names)
    else:
        regularization_costs = osc_regularlization_tracking_cost(
            ['input_cost', 'acceleration_cost', 'soft_constraint_cost'])
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
        if hasattr(msg, 'regularization_cost_names'):
            regularization_costs.append(msg.regularization_cost_names,
                                        msg.regularization_costs)
        else:
            regularization_cost_names = ['input_cost', 'acceleration_cost',
                                         'soft_constraint_cost']
            regularization_cost_list = [msg.input_cost, msg.acceleration_cost,
                                        msg.soft_constraint_cost]
            regularization_costs.append(regularization_cost_names,
                                        regularization_cost_list)
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
                    lcmt_osc_tracking_data_t(name=tracking_data.name)
            osc_debug_tracking_datas[tracking_data.name].append(
                tracking_data, msg.utime / 1e6)

        fsm.append(msg.fsm_state)

    tracking_cost_handler = osc_tracking_cost(osc_debug_tracking_datas.keys())
    for msg in data:
        if hasattr(msg, 'tracking_costs'):
            tracking_cost_handler.append(msg.tracking_data_names,
                                         msg.tracking_costs)
        else:
            tracking_cost_handler.append(msg.tracking_data_names,
                                         msg.tracking_cost)
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


def process_contact_channel(data):
    t_contact_info = []
    contact_forces = [[], [], [], []]  # Allocate space for all 4 point contacts
    contact_info_locs = [[], [], [], []]
    for msg in data:
        t_contact_info.append(msg.timestamp / 1e6)
        num_left_contacts = 0
        num_right_contacts = 0
        for i in range(msg.num_point_pair_contacts):
            if "toe_left" in msg.point_pair_contact_info[i].body2_name:
                if (num_left_contacts >= 2):
                    continue
                contact_info_locs[num_left_contacts].append(
                    msg.point_pair_contact_info[i].contact_point)
                contact_forces[num_left_contacts].append(
                    msg.point_pair_contact_info[i].contact_force)
                num_left_contacts += 1
            elif "toe_right" in msg.point_pair_contact_info[i].body2_name:
                if (num_right_contacts >= 2):
                    continue
                contact_info_locs[2 + num_right_contacts].append(
                    msg.point_pair_contact_info[i].contact_point)
                contact_forces[2 + num_right_contacts].append(
                    msg.point_pair_contact_info[i].contact_force)
                num_right_contacts += 1
        while num_left_contacts != 2:
            contact_forces[num_left_contacts].append((0.0, 0.0, 0.0))
            contact_info_locs[num_left_contacts].append((0.0, 0.0, 0.0))
            num_left_contacts += 1
        while num_right_contacts != 2:
            contact_forces[2 + num_right_contacts].append((0.0, 0.0, 0.0))
            contact_info_locs[2 + num_right_contacts].append((0.0, 0.0,
                                                              0.0))
            num_right_contacts += 1

    t_contact_info = np.array(t_contact_info)
    contact_forces = np.array(contact_forces)
    contact_info_locs = np.array(contact_info_locs)

    for i in range(contact_info_locs.shape[1]):
        # Swap front and rear contacts if necessary
        # Order will be front contact in index 1
        if contact_info_locs[0, i, 0] > contact_info_locs[1, i, 0]:
            contact_forces[[0, 1], i, :] = contact_forces[[1, 0], i, :]
            contact_info_locs[[0, 1], i, :] = contact_info_locs[[1, 0], i, :]
        if contact_info_locs[2, i, 0] > contact_info_locs[3, i, 0]:
            contact_forces[[2, 3], i, :] = contact_forces[[3, 2], i, :]
            contact_info_locs[[2, 3], i, :] = contact_info_locs[[3, 2], i, :]
    return {'t_lambda': t_contact_info,
            'lambda_c': contact_forces,
            'p_lambda_c': contact_info_locs}

def process_c3_channel(data):
    t = []
    states = []  # Allocate space for all 4 point contacts
    contact_forces = []
    inputs = []
    for msg in data:
        t.append(msg.timestamp / 1e6)
        states.append(msg.c3_solution.x_sol)
        contact_forces.append(msg.c3_solution.lambda_sol)
        inputs.append(msg.u_sol.lambda_sol)

    t_contact_info = np.array(states)
    contact_forces = np.array(contact_forces)
    contact_info_locs = np.array(inputs)

    return {'t_c3': t,
            'x_c3': states,
            'lambda_c3': contact_forces,
            'u_c3': inputs,}


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


def load_force_channels(data, contact_force_channel):
    contact_info = process_contact_channel(data[contact_force_channel])
    return contact_info

def load_c3_debug(data, c3_debug_channel):
    c3_debug = process_c3_channel(data[c3_debug_channel])
    return c3_debug

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


def plot_u_cmd(robot_input, key, x_names, x_slice, time_slice, ylabel=None,
               title=None):
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
                            title='Measured Joint Efforts')


def plot_commanded_efforts(robot_input, u_names, time_slice):
    return plot_u_cmd(robot_input, 'u', u_names, slice(len(u_names)),
                      time_slice, ylabel='Efforts (Nm)',
                      title='Commanded Joint Efforts')


def plot_measured_efforts_by_name(robot_output, u_names, time_slice, u_map):
    u_slice = [u_map[name] for name in u_names]
    return plot_q_or_v_or_u(robot_output, 'u', u_names, u_slice, time_slice,
                            ylabel='Efforts (Nm)', title='Select Joint Efforts')


def plot_points_positions(robot_output, time_slice, plant, context, frame_names,
                          pts, dims, ps=None):
    if ps is None:
        ps = plot_styler.PlotStyler()

    dim_map = ['_x', '_y', '_z']
    data_dict = {'t': robot_output['t_x']}
    legend_entries = {}
    for name in frame_names:
        frame = plant.GetBodyByName(name).body_frame()
        pt = pts[name]
        data_dict[name] = make_point_positions_from_q(robot_output['q'],
                                                      plant, context, frame, pt)
        legend_entries[name] = [name + dim_map[dim] for dim in dims[name]]

    plotting_utils.make_plot(
        data_dict,
        't',
        time_slice,
        frame_names,
        dims,
        legend_entries,
        {'title': 'Point Positions',
         'xlabel': 'time (s)',
         'ylabel': 'pos (m)'}, ps)

    return ps


def plot_points_velocities(robot_output, time_slice, plant, context, frame_names,
                          pts, dims, ps=None):
    if ps is None:
        ps = plot_styler.PlotStyler()

    dim_map = ['_x', '_y', '_z']
    data_dict = {'t': robot_output['t_x']}
    legend_entries = {}
    for name in frame_names:
        frame = plant.GetBodyByName(name).body_frame()
        pt = pts[name]
        data_dict[name] = make_point_velocities_from_qv(robot_output['q'],
                                                                  robot_output['v'],
                                                                  plant, context,
                                                                  frame, np.zeros(3))
        legend_entries[name] = [name + dim_map[dim] for dim in dims[name]]

    plotting_utils.make_plot(
        data_dict,
        't',
        time_slice,
        frame_names,
        dims,
        legend_entries,
        {'title': 'Point Velocities',
         'xlabel': 'time (s)',
         'ylabel': 'pos (m)'}, ps)

    return ps


def plot_floating_base_body_frame_velocities(robot_output, time_slice, plant,
                                             context, fb_frame_name):
    data_dict = {'t': robot_output['t_x']}
    data_dict['base_vel'] = get_floating_base_velocity_in_body_frame(
        robot_output, plant, context,
        plant.GetBodyByName(fb_frame_name).body_frame())[:, 0]
    legend_entries = {'base_vel': ['forward', 'lateral', 'vertical']}
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        data_dict,
        't',
        time_slice,
        ['base_vel'],
        {},
        legend_entries,
        {'title': 'Floating Base Velocity (Body Frame)',
         'xlabel': 'Time (s)',
         'ylabel': 'Velocity (m/s)'}, ps)
    return ps


def plot_tracking_costs(osc_debug, time_slice):
    ps = plot_styler.PlotStyler()
    data_dict = \
        {key: val for key, val in osc_debug['tracking_cost'].items()}
    data_dict['t_osc'] = osc_debug['t_osc']
    # import pdb; pdb.set_trace()
    # data_dict['t_osc'] = osc_debug['trac']
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
         'ylabel': OSC_DERIV_UNITS[deriv],
         'title': f'{traj_name} {deriv} tracking {dim}'}, ps)
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
         'ylabel': OSC_DERIV_UNITS[deriv],
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
        data['error_ydot'] = tracking_data.ydot_des[:,
                             dim] - tracking_data.ydot[:,
                                    dim]
        data['projected_error_ydot'] = tracking_data.error_ydot[:, dim]
    elif deriv == 'accel':
        data['yddot_des'] = tracking_data.yddot_des[:, dim]
        data['yddot_actual'] = 1000 * np.diff(tracking_data.ydot[:, dim], prepend=[0])
        data['yddot_command'] = tracking_data.yddot_command[:, dim]
        data['yddot_command_sol'] = tracking_data.yddot_command_sol[:, dim]

    data['t'] = tracking_data.t

    return plot_general_osc_tracking_data(traj, deriv, dim, data, time_slice)


def plot_osc_tracking_data_in_space(osc_debug, traj, dims, deriv, time_slice):
    tracking_data = osc_debug['osc_debug_tracking_datas'][traj]
    data = {}
    for dim in dims:
        if deriv == 'pos':
            data['y_des_' + str(dim)] = tracking_data.y_des[:, dim]
            data['y_' + str(dim)] = tracking_data.y[:, dim]
            data['error_y_' + str(dim)] = tracking_data.error_y[:, dim]
        elif deriv == 'vel':
            data['ydot_des_' + str(dim)] = tracking_data.ydot_des[:, dim]
            data['ydot_' + str(dim)] = tracking_data.ydot[:, dim]
            data['error_ydot_' + str(dim)] = tracking_data.ydot_des[:,
                                             dim] - tracking_data.ydot[:, dim]
            data['projected_error_ydot_' + str(dim)] = tracking_data.error_ydot[
                                                       :,
                                                       dim]
        elif deriv == 'accel':
            data['yddot_des_' + str(dim)] = tracking_data.yddot_des[:, dim]
            data['yddot_command_' + str(dim)] = tracking_data.yddot_command[:,
                                                dim]
            data['yddot_command_sol_' + str(
                dim)] = tracking_data.yddot_command_sol[:,
                        dim]

    data['t'] = tracking_data.t
    ps = plot_styler.PlotStyler()
    ps.plot(data['y_des_' + str(0)],
            data['y_des_' + str(2)] - data['y_des_' + str(2)][0],
            xlabel='Forward Position (m)', ylabel='Vertical Position (m)',
            grid=False)
    # ps.tight_layout()
    # ps.axes[0].set_ylim([-0.05, 0.5])
    return ps


def plot_qp_costs(osc_debug, time_slice):
    ps = plot_styler.PlotStyler()
    regularization_cost = osc_debug['regularization_costs'].regularization_costs
    data_dict = \
        {key: val for key, val in regularization_cost.items()}
    data_dict['t_osc'] = osc_debug['t_osc']
    ps = plot_styler.PlotStyler()
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


def plot_qp_solutions(osc_debug, time_slice):
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        ['lambda_c_sol'],
        {},
        {},
        {'xlabel': 'Timestamp',
         'ylabel': 'Solve Time ',
         'title': 'OSC Lambda Solutions'}, ps)
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


def plot_lambda_h_sol(osc_debug, time_slice, lambda_slice):
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        ['lambda_h_sol'],
        {'lambda_h_sol': lambda_slice},
        {'lambda_h_sol': ['lambda_h_' + i for i in
                          plotting_utils.slice_to_string_list(lambda_slice)]},
        {'xlabel': 'time',
         'ylabel': 'lambda',
         'title': 'OSC constraint force solution'}, ps)
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


def add_fsm_to_plot(ps, fsm_time, fsm_signal, fsm_state_names):
    ax = ps.fig.axes[0]
    ymin, ymax = ax.get_ylim()

    # uses default color map
    legend_elements = []
    for i in np.unique(fsm_signal):
        ax.fill_between(fsm_time, ymin, ymax, where=(fsm_signal == i),
                        color=ps.cmap(2 * i), alpha=0.2)
        if fsm_state_names:
            legend_elements.append(
                Patch(facecolor=ps.cmap(2 * i), alpha=0.3,
                      label=fsm_state_names[i]))

    if len(legend_elements) > 0:
        legend = ax.legend(handles=legend_elements, loc=4)
        # ax.add_artist(legend)
        ax.relim()
    ax.relim()


def plot_active_tracking_datas(osc_debug, time_slice, fsm_time, fsm_signal,
                               fsm_state_names):
    ps = plot_styler.PlotStyler()

    ax = ps.fig.axes[0]
    # ymin, ymax = ax.get_ylim()
    ymin, ymax = 0, 1
    tracking_data_names = osc_debug['osc_debug_tracking_datas'].keys()
    # import pdb; pdb.set_trace()
    n_tracking_datas = len(tracking_data_names)
    tracking_data_legend_elements = []

    tracking_data_name_dict = {'pelvis_trans_traj': 'Pelvis Position',
                               'left_ft_traj': 'Left Foot Position',
                               'right_ft_traj': 'Right Foot Position',
                               'pelvis_rot_tracking_data': 'Pelvis Orientation',
                               'left_toe_angle_traj': 'Left Toe Joint Angle',
                               'right_toe_angle_traj': 'Right Toe Joint Angle',
                               'swing_hip_yaw_left_traj': 'Left Hip Yaw Angle',
                               'swing_hip_yaw_right_traj': 'Right Hip Yaw Angle',
                               }

    for i, tracking_data_name in enumerate(tracking_data_names):
        # tracking_data_name = tracking_data_names[i]
        tracking_data = osc_debug['osc_debug_tracking_datas'][
            tracking_data_name]
        ax.fill_between(fsm_time, ymax - (i + 0.25) / n_tracking_datas, ymax -
                        (i + 0.75) / n_tracking_datas,
                        where=tracking_data.is_active.astype(bool)[
                              :fsm_time.shape[0]],
                        color=ps.cmap(2 * i), alpha=0.7)
        tracking_data_legend_elements.append(Patch(facecolor=ps.cmap(2 * i),
                                                   alpha=0.7,
                                                   label=
                                                   tracking_data_name_dict[
                                                       tracking_data_name]))

    # uses default color map
    legend_elements = []
    for i in np.unique(fsm_signal):
        ax.fill_between(fsm_time, ymin, ymax, where=(fsm_signal == i),
                        color=ps.cmap(2 * i), alpha=0.2)
        if fsm_state_names:
            legend_elements.append(
                Patch(facecolor=ps.cmap(2 * i), alpha=0.3,
                      label=fsm_state_names[i]))
    ps.tight_layout()

    if len(legend_elements) > 0:
        legend = ax.legend(handles=legend_elements, loc=4)
        ax.add_artist(legend)
        legend = ax.legend(handles=tracking_data_legend_elements, loc=1)
        ax.add_artist(legend)
        ax.relim()
    ax.set_yticklabels([])
    ax.set_yticks([])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tracking Objective')

    return ps

def generate_joint_limits(plant):
    joint_position_limits_lower = np.zeros(plant.num_positions())
    joint_position_limits_upper = np.zeros(plant.num_positions())
    joint_velocity_limits_lower = np.zeros(plant.num_positions())
    joint_velocity_limits_upper = np.zeros(plant.num_positions())
    joint_actuator_limits_lower = np.zeros(plant.num_positions())
    joint_actuator_limits_upper = np.zeros(plant.num_positions())
    for i in range(plant.num_positions()):
        joint_position_limits_lower[i] = plant.get_joint(JointIndex(i)).position_upper_limits()[0]
        joint_position_limits_upper[i] = plant.get_joint(JointIndex(i)).position_lower_limits()[0]
        joint_velocity_limits_lower[i] = plant.get_joint(JointIndex(i)).velocity_upper_limits()[0]
        joint_velocity_limits_upper[i] = plant.get_joint(JointIndex(i)).velocity_lower_limits()[0]
        joint_actuator_limits_lower[i] = -plant.get_joint_actuator(JointActuatorIndex(i)).effort_limit()
        joint_actuator_limits_upper[i] = plant.get_joint_actuator(JointActuatorIndex(i)).effort_limit()
    franka_joint_position_limit_range = [np.min(joint_position_limits_lower), np.max(joint_position_limits_upper)]
    franka_joint_velocity_limit_range = [np.min(joint_velocity_limits_lower), np.max(joint_velocity_limits_upper)]
    franka_joint_actuator_limit_range = [np.min(joint_actuator_limits_lower), np.max(joint_actuator_limits_upper)]
    return franka_joint_position_limit_range, franka_joint_velocity_limit_range, franka_joint_actuator_limit_range
