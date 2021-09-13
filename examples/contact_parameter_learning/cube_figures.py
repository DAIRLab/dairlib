import matplotlib
matplotlib.use('Agg')
from cube_sim import CUBE_DATA_DT, CubeSim, BLOCK_HALF_WIDTH, FastLossWeights, load_cube_toss, make_cube_toss_filename
import drake_cube_sim
from json import load
import evaluate_cube_parameters as cube_eval
import os
import glob
import numpy as np
from scipy.stats import gaussian_kde
from matplotlib import pyplot as plt
from plotting_utils import format_sim_name
import sensitivity_analysis as sa
from pydairlib.common.plot_styler import PlotStyler


figure_directory = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/figures/')

ps = PlotStyler()
ps.set_default_styling(directory=figure_directory, figsize=(10,6))

sim_colors = {'Drake' : ps.blue, 'MuJoCo': ps.red, 'Bullet' : ps.yellow}

paper_ids = ['drake_2021_09_11_16_44_10',
           'mujoco_2021_09_12_10_27_10', 
           'bullet_2021_09_12_22_00_10']

# def plot_damping_ratios(ids):
#     stiffness = []
#     zeta = []
#     for id in ids:
#         params, _, _ = cube_eval.load_params_and_logs(id)
#         stiffness.append(params['stiffness'])
#         zeta.append(cube_eval.calc_damping_ratio(params))

#     plt.scatter(stiffness, zeta)
#     plt.show()


def plot_estimated_loss_pdfs(losses):
    training_results = list(losses.keys())
    i = 0
    legend_strs = []
    filename = ''
    for result in training_results:
        loss = np.array(list(losses[result].values()))
        pts = np.linspace(np.min(loss), np.max(loss), 100)
        pdf = gaussian_kde(loss).pdf(pts)
        ps.plot(pts, pdf, color=ps.penn_color_wheel[i])
        legend_strs.append(format_sim_name(result))# + ' $T_{sim}$ = ' + str(148 * int(result.split('_')[-1])) + ' Hz')
        filename += format_sim_name(result) + '_' + result.split('_')[-1] + '_'
        i += 1
    
    filename += '.png'
    plt.ylabel('PDF Estimate')
    plt.xlabel('$e_{q}(\hat{x}_{t}, x_{t}^{*})$')
    plt.xlim((0, 2.0))
    plt.legend(legend_strs)
    ps.save_fig(filename)

def plot_sensitivity_analysis(loss_sweeps, params_range, title=''):
    for key in loss_sweeps:
        plt.figure()
        ps.plot(params_range[key], loss_sweeps[key], color=ps.blue)
        plt.title(f'{title} - {key}')
        if (key == 'pen_allow' or key == 'stiction_tol' or key == 'mu_torsion' or key == 'mu_rolling'):
            plt.xscale('log')


def plot_impulses_list_of_ids(ids, traj_id):
    legend_strs = []
    for id in ids:
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        pair = cube_eval.load_traj_pairs(sim, params, [traj_id])[traj_id]
        impulse = cube_eval.calculate_contact_impulse(pair[1])
        time = CubeSim.make_traj_timestamps(pair[1])
        legend_strs.append(format_sim_name(id))
        ps.plot(time[1:], impulse[:,0], color=sim_colors[format_sim_name(id)])
    plt.xlim((0, time[-1]/4.0))
    plt.xlabel('t (s)')
    plt.ylabel('Friction Force (N)')
    plt.legend(legend_strs)
    ps.save_fig('ContactTangentImpulses.png')
    # plt.show()

def make_training_loss_sensitivity_analysis(ids, params_ranges):
    sweeps = {}
    for id in ids:
        sim_type = id.split('_')[0]
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        test_set = range(550)

        weights = FastLossWeights(
            pos=(1.0/BLOCK_HALF_WIDTH)*np.ones((3,)),
            bullet=(format_sim_name(id) == 'Bullet'))
            
        loss_avg, loss_med = sa.get_sensitivity_analysis(
            sim, weights, params, params_ranges[id], test_set)
        sweeps[id] = {'loss_avg' : loss_avg, 
                      'loss_med' : loss_med}
    return sweeps

def make_pos_rot_sensitivity_analysis(ids, params_ranges):
    sweeps = {}
    for id in ids:
        sim_type = id.split('_')[0]
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        test_set = range(550)
        pos_avg, pos_med, rot_avg, rot_med = \
             sa.get_cube_position_and_rotation_error_sensitivity(
                sim, 
                params, 
                params_ranges[id], 
                test_set)

        sweeps[id] = {'pos_avg' : pos_avg, 
                      'pos_med': pos_med, 
                      'rot_avg': rot_avg, 
                      'rot_med': rot_med}
    return sweeps

def make_stiffness_sensitivity_analysis_figure():
    ids = paper_ids

    params_ranges = {}
    params = {}
    for id in ids:
        param, _, _ = cube_eval.load_params_and_logs(id)
        params[id] = param
        params_ranges[id] = sa.get_stiffness_range(id.split('_')[0], param['stiffness'])
    sweeps = make_training_loss_sensitivity_analysis(ids, params_ranges)

    ## plotting
    legend_strs = []
    for id in ids:
        legend_strs.append(format_sim_name(id))
        k_opt = params[id]['stiffness']
        k_ratio = np.array(params_ranges[id]['stiffness']) / k_opt
        ps.plot(k_ratio, sweeps[id]['loss_avg']['stiffness'], color=sim_colors[format_sim_name(id)])

    plt.xlabel('$\mu / \mu^{*}$')
    plt.legend(legend_strs)
    plt.ylabel('Average $e_{q}$')
    plt.ylim((0, 0.8))
    ps.save_fig('StiffnessSensitivity.png')
    # plt.show()
      

def make_friction_sensitivity_analysis_figure():
    ids = paper_ids

    mu_keys = {ids[0] : 'mu_tangent', ids[1] : 'mu', ids[2] : 'mu_tangent'}

    params_ranges = {}
    params = {}
    for id in ids:
        param, _, _ = cube_eval.load_params_and_logs(id)
        params[id] = param
        params_ranges[id] = sa.get_friction_range(id.split('_')[0], param[mu_keys[id]])
    sweeps = make_training_loss_sensitivity_analysis(ids, params_ranges)

    ## plotting
    legend_strs = []

    for id in ids:
        legend_strs.append(format_sim_name(id))
        k_opt = params[id][mu_keys[id]]
        k_ratio = np.array(params_ranges[id][mu_keys[id]]) / k_opt
        ps.plot(k_ratio, sweeps[id]['loss_avg'][mu_keys[id]], color=sim_colors[format_sim_name(id)])

    plt.xlabel('$\mu / \mu^{*}$')
    plt.legend(legend_strs)
    plt.ylabel('Average $e_{q}$')
    plt.ylim((0, 0.8))
    ps.save_fig('FrictionSensitivity.png')


def make_damping_sensitivity_analysis_figure():
    ids = paper_ids

    mu_keys = {ids[0] : 'dissipation', ids[1] : 'damping', ids[2] : 'damping'}

    params_ranges = {}
    params = {}
    for id in ids:
        param, _, _ = cube_eval.load_params_and_logs(id)
        params[id] = param
        params_ranges[id] = sa.get_damping_range(id.split('_')[0], param[mu_keys[id]])
    sweeps = make_training_loss_sensitivity_analysis(ids, params_ranges)

    ## plotting
    legend_strs = []

    for id in ids:
        legend_strs.append(format_sim_name(id))
        k_opt = params[id][mu_keys[id]]
        k_ratio = np.array(params_ranges[id][mu_keys[id]]) / k_opt
        ps.plot(k_ratio, sweeps[id]['loss_avg'][mu_keys[id]], color=sim_colors[format_sim_name(id)])

    plt.xlabel('$b / b^{*}$')
    plt.legend(legend_strs)
    plt.ylabel('Average $e_{q}$')
    plt.ylim((0, 0.8))
    ps.save_fig('DampingSensitivity.png')
    # plt.show()


def make_estimated_pdf_figure():
    ids = paper_ids
    _, e_q, _, _, _ = cube_eval.load_list_of_results(
        ids, cube_eval.pos_rot_loss, eval_all_traj=True)

    plot_estimated_loss_pdfs(e_q)


# def plot_error_vs_time(ids, traj_id):
#     legend_strs = []
#     for id in ids:
#         legend_strs.append(format_sim_name(id))
#         sim = cube_eval.get_eval_sim(id)
#         params, _, _ = cube_eval.load_params_and_logs(id)
#         pair = cube_eval.load_traj_pairs(sim, params, [traj_id])[traj_id]
#         error = cube_eval.calc_error_between_trajectories(pair)['velocity_error']
#         time = CubeSim.make_traj_timestamps(pair[0])
#         ps.plot(time, error, color=sim_colors[legend_strs[id]])
#         i+=1
#     plt.legend(ids)
#     plt.show()

# def make_mujoco_damping_ratio_figure():
#     id_paths = glob.glob('examples/contact_parameter_learning/learned_parameters/cube/mujoco_2021_0*_10.json')
#     ids = [os.path.basename(id_path).split('.')[0] for id_path in id_paths]
#     plot_damping_ratios(ids)

# def make_bullet_damping_ratio_figure():
#     id_paths = glob.glob('examples/contact_parameter_learning/learned_parameters/cube/bullet_2021_0*_10.json')
#     ids = [os.path.basename(id_path).split('.')[0] for id_path in id_paths]
#     plot_damping_ratios(ids)

# def make_error_vs_time_plot():
#     ids = ['drake_2021_09_11_16_44_10',
#            'mujoco_2021_09_12_10_27_10', 
#            'bullet_2021_09_11_14_27_10']
#     traj = 164
#     plot_error_vs_time(ids, traj)

def make_contact_impulse_plot():
    ids = paper_ids
    traj_id  = 361
    plot_impulses_list_of_ids(ids, traj_id)

def visualize_cube_initial_condition():
    sim = drake_cube_sim.DrakeCubeSim(visualize=True, substeps=10)
    sim.init_sim(drake_cube_sim.default_drake_contact_params)
    cube_data = load_cube_toss(
            make_cube_toss_filename(cube_eval.cube_data_folder, 69))
    sim.set_initial_condition(cube_data[0])
    sim.sim_step(CUBE_DATA_DT)
    sim.sim_step(CUBE_DATA_DT)
    sim.sim_step(CUBE_DATA_DT)

if __name__ == '__main__':
    # make_estimated_pdf_figure()
    # make_friction_sensitivity_analysis_figure()
    # make_damping_sensitivity_analysis_figure()
    make_stiffness_sensitivity_analysis_figure()
    # make_error_vs_time_plot()
    # make_contact_impulse_plot()
    # visualize_cube_initial_condition()