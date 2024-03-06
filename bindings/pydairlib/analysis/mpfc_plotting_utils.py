import numpy as np
import matplotlib.pyplot as plt

from pydairlib.common import plot_styler, plotting_utils
from pydairlib.analysis.mbp_plotting_utils import add_fsm_to_plot


_fsm_state_names = ['Left Stance (LS)',
                    'Right Stance (RS)', ' ',
                    'Double Support Post Left (DSPL)',
                    'Double Support Post Right (DSPR)']


def process_mpfc_debug_data(data):
    n = len(data)
    nmodes = data[0].nmodes

    t_mpc = np.zeros((n,))
    fsm = np.zeros((n,), dtype=int)
    solve_time = np.zeros((n,))
    initial_state = np.zeros((n, 4))
    initial_stance_foot = np.zeros((n, 3))
    desired_velocity = np.zeros((n, 2))
    nominal_first_stance_time = np.zeros((n, 1))
    solution_first_stance_time = np.zeros((n, 1))
    pp = [np.zeros((nmodes, 3))] * n
    xx = [np.zeros((nmodes, 4))] * n

    for i, msg in enumerate(data):
        t_mpc[i] = msg.utime * 1e-6
        fsm[i] = msg.fsm_state
        solve_time[i] = msg.solve_time_us * 1e-6
        initial_state[i] = msg.initial_state
        initial_stance_foot[i] = msg.initial_stance_foot
        desired_velocity[i] = msg.desired_velocity
        nominal_first_stance_time[i][0] = msg.nominal_first_stance_time
        solution_first_stance_time[i][0] = msg.solution_first_stance_time
        pp[i] = np.array(msg.pp)
        xx[i] = np.array(msg.xx)

    return {
        't_mpc': t_mpc,
        'fsm': fsm,
        'solve_time': solve_time,
        'initial_state': initial_state,
        'initial_stance_foot': initial_stance_foot,
        'desired_velocity': desired_velocity,
        'nominal_first_stance_time': nominal_first_stance_time,
        'solution_first_stance_time': solution_first_stance_time,
        'pp': pp,
        'xx': xx,
    }


def calc_next_footstep_in_stance_frame(mpc_data):
    n = len(mpc_data['t_mpc'])
    p1 = np.zeros((n, 3))
    for i, pp in enumerate(mpc_data['pp']):
        p1[i] = pp[1] - pp[0]
    return p1


def plot_initial_state(mpc_data, time_slice=None):
    time_slice = slice(len(mpc_data['t_mpc'])) if time_slice is None else (
        time_slice)
    ps = plot_styler.PlotStyler()

    xslice = slice(4)

    plotting_utils.make_plot(
        mpc_data,
        't_mpc',
        time_slice,
        ['initial_state'],
        {'initial_state': xslice},
        {'initial_state': ['x', 'y', 'Lx', 'Ly']},
        {'xlabel': 'Time (s)',
         'ylabel': 'Initial State',
         'title': 'Initial ALIP State'},
        ps
    )
    add_fsm_to_plot(ps, mpc_data['t_mpc'], mpc_data['fsm'], _fsm_state_names)
    return ps


def plot_timing_solution(mpc_data, time_slice=None):
    time_slice = slice(len(mpc_data['t_mpc'])) if time_slice is None else (
        time_slice)

    ps = plot_styler.PlotStyler()

    plotting_utils.make_plot(
        mpc_data,
        't_mpc',
        time_slice,
        ['nominal_first_stance_time', 'solution_first_stance_time'],
        {'nominal_first_stance_time': slice(1),
         'solution_first_stance_time': slice(1)},
        {'nominal_first_stance_time': ['$T_{nom}$'],
         'solution_first_stance_time': ['$T_{sol}$']},
        {'xlabel': 'Time (s)',
         'ylabel': 'Gait Timing',
         'title': 'Nominal and Solution Gait Timing'},
        ps
    )
    add_fsm_to_plot(ps, mpc_data['t_mpc'], mpc_data['fsm'], _fsm_state_names)
    return ps


def plot_footstep_sol_in_stance_frame(mpc_data, time_slice=None):
    time_slice = slice(len(mpc_data['t_mpc'])) if time_slice is None else (
        time_slice)
    ps = plot_styler.PlotStyler()

    data = {
        't_mpc': mpc_data['t_mpc'],
        'next_footstep_in_stance_frame':
            calc_next_footstep_in_stance_frame(mpc_data)
    }

    plotting_utils.make_plot_of_entire_series(
        data,
        't_mpc',
        {'next_footstep_in_stance_frame': ['px', 'py', 'pz']},
        {'xlabel': 'Time',
         'ylabel': 'Next Footstep Pos',
         'title': 'Next Footstep Solution in the Stance Frame'}, ps
    )

    add_fsm_to_plot(ps, mpc_data['t_mpc'], mpc_data['fsm'], _fsm_state_names)

    return ps


def mpfc_debug_callback(data_dict, channel):
    return process_mpfc_debug_data(data_dict[channel])
