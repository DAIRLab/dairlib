import numpy as np
import matplotlib.pyplot as plt

from pydairlib.common import plot_styler, plotting_utils
from osc_debug import lcmt_osc_tracking_data_t, osc_tracking_cost
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


def process_effort_channel(data):
    u = []
    t = []
    for msg in data:
        t.append(msg.utime / 1e6)
        u.append(msg.efforts)

    return {'t_u': np.array(t), 'u': np.array(u)}


def process_osc_channel(data):
    t_osc = []
    input_cost = []
    accel_cost = []
    soft_constraint_cost = []
    qp_solve_time = []
    u_sol = []
    osc_output = []
    fsm = []
    osc_debug_tracking_datas = {}

    for msg in data:
        t_osc.append(msg.utime / 1e6)
        input_cost.append(msg.input_cost)
        accel_cost.append(msg.acceleration_cost)
        soft_constraint_cost.append(msg.soft_constraint_cost)
        qp_solve_time.append(msg.qp_output.solve_time)
        u_sol.append(msg.qp_output.u_sol)
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
        tracking_cost_handler.append(msg.tracking_data_names, msg.tracking_cost)
    tracking_cost = tracking_cost_handler.convertToNP()

    for name in osc_debug_tracking_datas:
        osc_debug_tracking_datas[name].convertToNP()

    return {'t_osc': np.array(t_osc),
            'input_cost': np.array(input_cost),
            'acceleration_cost': np.array(accel_cost),
            'soft_constraint_cost': np.array(soft_constraint_cost),
            'qp_solve_time': np.array(qp_solve_time),
            'osc_output': osc_output,
            'tracking_cost': tracking_cost,
            'osc_debug_tracking_datas': osc_debug_tracking_datas,
            'fsm': np.array(fsm)}


def load_default_channels(data, plant, state_channel, input_channel,
                          osc_debug_channel):
    robot_output = process_state_channel(data[state_channel], plant)
    robot_input = process_effort_channel(data[input_channel])
    osc_debug = process_osc_channel(data[osc_debug_channel])

    return robot_output, robot_input, osc_debug


def load_state_channel(data, plant, state_channel):
    return process_state_channel(data[state_channel], plant)


def load_input_channel(data, input_channel):
    return process_effort_channel(data[input_channel])


def load_osc_channel(data, osc_debug_channel):
    return process_osc_channel(data[osc_debug_channel])


def plot_q_or_v_or_u(
        robot_output, key, x_names, x_slice, time_slice,
        ylabel=None, title=None):
    ps = plot_styler.PlotStyler()
    if ylabel is None:
        ylabel = key
    if title is None:
        title = key

    plotting_utils.make_plot(
        robot_output,                       # data dict
        't_x',                              # time channel
        time_slice,
        [key],                              # key to plot
        {key: x_slice},                     # slice of key to plot
        {key: x_names},                     # legend entries
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


def plot_measured_efforts_by_name(robot_output, u_names, time_slice, u_map):
    u_slice = [u_map[name] for name in u_names]
    return plot_q_or_v_or_u(robot_output, 'u', u_names, u_slice, time_slice,
                            ylabel='Efforts (Nm)', title='Select Joint Efforts')


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


def plot_general_osc_tracking_data(traj_name, deriv, dim, data,
                                   t_osc, time_slice):
    ps = plot_styler.PlotStyler()
    data_dict = {key: val for key, val in data.items()}
    data_dict['t_osc'] = t_osc
    plotting_utils.make_plot(
        data_dict,
        't_osc',
        time_slice,
        data.keys(),
        {},
        {key: [key] for key in data.keys()},
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
        data['error_ydot'] = tracking_data.error_ydot[:, dim]
    elif deriv == 'accel':
        data['yddot_des'] = tracking_data.yddot_des[:, dim]
        data['yddot_command'] = tracking_data.yddot_command[:, dim]
        data['yddot_command_sol'] = tracking_data.yddot_command_sol[:, dim]

    return plot_general_osc_tracking_data(traj, deriv, dim, data,
                                          osc_debug['osc_debug_tracking_datas'][traj].t, time_slice)


def plot_qp_costs(osc_debug, time_slice):
    cost_keys = ['input_cost', 'acceleration_cost',
                 'soft_constraint_cost']
    ps = plot_styler.PlotStyler()
    plotting_utils.make_plot(
        osc_debug,
        't_osc',
        time_slice,
        cost_keys,
        {},
        {key: [key] for key in cost_keys},
        {'xlabel': 'Time',
         'ylabel': 'Cost',
         'title': 'OSC QP Costs'}, ps)
    return ps


def add_fsm_to_plot(ps, fsm_time, fsm_signal, scale=1):
    ps.plot(fsm_time, scale*fsm_signal)