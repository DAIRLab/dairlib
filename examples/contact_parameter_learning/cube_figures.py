import matplotlib
# matplotlib.use('Agg')
import evaluate_cube_parameters as cube_eval
from random import choice
from json import load
from random import sample
import os
import glob
import numpy as np
from scipy.stats import gaussian_kde
from matplotlib import pyplot as plt
from matplotlib import colors as colors
from plotting_utils import format_sim_name
import sensitivity_analysis as sa
import drake_cube_sim
from cube_sim import CUBE_DATA_DT, CubeSim, BLOCK_HALF_WIDTH, FastLossWeights, load_cube_toss, make_cube_toss_filename
from pydairlib.common.plot_styler import PlotStyler
import sys


figure_directory = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/figures/')

ps = PlotStyler()
ps.set_default_styling(directory=figure_directory, figsize=(8,6))

sim_colors = {'Drake': ps.blue, 'MuJoCo': ps.red, 'Bullet' : ps.yellow}

paper_ids = ['drake_2022_02_21_15_19_10',
             'mujoco_2022_02_23_15_29_10',
             'bullet_2022_02_22_00_36_10']

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
    styles = ['-', '-', '--']
    for result in training_results:
        loss = np.array(list(losses[result].values()))
        pts = np.linspace(np.min(loss), np.max(loss), 100)
        pdf = gaussian_kde(loss).pdf(pts)
        ps.plot(pts, pdf, linestyle=styles[i], color=ps.penn_color_wheel[i])
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


def make_bounces_histogram(ids):
    legend_strs = []
    bounces = {}
    for id in ids:
        corner_z = np.zeros((550,))
        corner_z_sim = np.zeros((550,))
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        traj_pairs = cube_eval.load_traj_pairs(sim, params, range(550))
        for i in range(len(traj_pairs)):
            sdf = cube_eval.calculate_sdf_trajectory(traj_pairs[i][0])
            start = np.argwhere(sdf <= 0)[0, 0]
            sdf = sdf[start:]
            sdf_sim = cube_eval.calculate_sdf_trajectory(traj_pairs[i][1])
            start = np.argwhere(sdf_sim <= 0)[0, 0]
            sdf_sim = sdf_sim[start:]
            apex_t = np.argmax(sdf)
            apex_t_sim = np.argmax(sdf_sim)
            corner_z[i] = 100 * sdf[apex_t]/(2*BLOCK_HALF_WIDTH)
            corner_z_sim[i] = 100 * sdf_sim[apex_t_sim] / (2*BLOCK_HALF_WIDTH)
        bounces[id] = corner_z_sim
        bounces["real"] = corner_z

    for id in ids:
        plt.hist(bounces[id], 100, histtype=u'step',
                 label=format_sim_name(id))

    plt.hist(bounces["real"], 100,  histtype=u'step',label="Real Cube")
    plt.xlabel('Bounce Height (\% Cube Width)')
    plt.ylabel('Count')
    plt.legend()
    ps.save_fig('CubeBounceHistogram.png')


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


def make_damping_ratio_sensitivity_analysis(ids, params_ranges):
    sweeps = {}
    for id in ids:
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        test_set = range(550)
        weights = FastLossWeights(
            pos=(1.0/BLOCK_HALF_WIDTH)*np.ones((3,)),
            bullet=(format_sim_name(id) == 'Bullet'))
            
        loss_avg, loss_med = sa.get_dr_sensitivity_analysis(
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


def make_gridded_sensitivity_analysis_figure(id, damp_key):
    # optimal_params, _, _ = cube_eval.load_params_and_logs(id)
    # stiffness_range = sa.get_stiffness_range(id.split('_')[0],
    #                                         optimal_params['stiffness'], discretization_n=11)
    # damping_range = sa.get_damping_range(id.split('_')[0],
    #                                      optimal_params[damp_key], discretization_n=11)
    # params_range = {'stiffness': stiffness_range['stiffness'],
    #                 damp_key: damping_range[damp_key]}
    # weights = FastLossWeights(
    #     pos=(1.0/BLOCK_HALF_WIDTH)*np.ones((3,)),
    #     bullet=(format_sim_name(id) == 'Bullet'))
    # sensitivity_analysis = \
    #     sa.get_gridded_stiffness_damping_sensitivity_analysis(
    #         cube_eval.get_eval_sim(id), weights, optimal_params,
    #         params_range, range(550))
    #
    # X = sensitivity_analysis['stiffness'] / optimal_params['stiffness']
    # Y = sensitivity_analysis[damp_key] / optimal_params[damp_key]
    # Z = sensitivity_analysis['loss_avgs']


    # np.save('cube_X_' + format_sim_name(id) + '_grid', X)
    # np.save('cube_Y_' + format_sim_name(id) + '_grid', Y)
    # np.save('cube_Z_' + format_sim_name(id) + '_grid', Z)
    cmaps = {'Drake': 'GnBu',
             'MuJoCo': 'Reds',
             'Bullet': 'YlOrBr'}

    X = np.load('cube_X_' + format_sim_name(id) + '_grid.npy')
    Y = np.load('cube_Y_' + format_sim_name(id) + '_grid.npy')
    Z = np.load('cube_Z_' + format_sim_name(id) + '_grid.npy')

    # levels = np.linspace(0.27, 0.4, 50)
    cmap = plt.cm.get_cmap(cmaps[format_sim_name(id)])
    cmap = cmap.reversed()
    plt.contourf(X, Y, Z, 20, cmap=cmap)
    if format_sim_name(id) == 'Bullet':
        plt.colorbar(extend='max', label='Average Error $(e_{q})$', format='%.2f')
    else:
        plt.colorbar(extend='max', format='%.2f')
    plt.xscale('log')
    plt.yscale('log')
    frame = plt.gca()

    frame.axes.get_xaxis().set_ticks([0.1, 0.3, 1.0, 3.1, 10])
    frame.axes.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    frame.axes.get_yaxis().set_ticks([0.1, 0.3, 1.0, 3.1, 10])
    frame.axes.get_yaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    plt.xlim((X[0, 0], X[-1, -1]))
    plt.ylim((Y[0, 0], Y[-1, -1]))
    plt.xlabel('Normalized Stiffness $k / k^{*}$')
    if format_sim_name(id) == 'Drake':
        plt.ylabel('Normalised Damping $b / b^{*}$')
    plt.title(f'{format_sim_name(id)} Sensitivity (Cube)')

    ps.save_fig(format_sim_name(id) + '_cube_grid')


def make_damping_ratio_sensitivity_analysis_figure():
    ids = paper_ids[1:]

    params_ranges = {}
    params = {}
    for id in ids:
        param, _, _ = cube_eval.load_params_and_logs(id)
        params[id] = param
        params_ranges[id] = sa.get_damping_ratio_range(id.split('_')[0], param['stiffness'], param['damping'])
    sweeps = make_damping_ratio_sensitivity_analysis(ids, params_ranges)

    ## plotting
    legend_strs = []
    
    for id in ids:
        legend_strs.append(format_sim_name(id))
        k_opt = params[id]['stiffness']
        k_ratio = np.array(params_ranges[id]['stiffness']) / k_opt
        ps.plot(k_ratio, sweeps[id]['loss_avg']['stiffness'], color=sim_colors[format_sim_name(id)])
    plt.title('Cube Stiffness Sensitivity - Constant $\zeta$')
    plt.xlabel('$k / k^{*}$')
    plt.xscale('log')
    frame = plt.gca()
    frame.axes.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ps.set_figsize((10,6))
    plt.legend(legend_strs)
    plt.ylabel('Average $e_{q}$')
    plt.ylim((0, 0.8))
    frame.axes.get_yaxis().set_ticks([0.1, 0.3, 0.5, 0.7])
    plt.xlim((k_ratio[0], k_ratio[-1]))
    ps.save_fig('StiffnessSensitivityDampingRatio.png')
    # plt.show()


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
    plt.title('Cube Stiffness Sensitivity')
    plt.xlabel('$k / k^{*}$')
    plt.xscale('log')
    frame = plt.gca()
    frame.axes.get_xaxis().set_ticks([0.01, 0.03, 0.1, 0.3, 1.0, 3.1, 10])
    frame.axes.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ps.set_figsize((8,6))
    plt.legend(legend_strs)
    plt.ylabel('Average $e_{q}$')
    plt.ylim((0, 0.8))
    plt.xlim((k_ratio[0], k_ratio[-1]))
    ps.save_fig('StiffnessSensitivity.png')
    # plt.show()
      

def make_friction_sensitivity_analysis_figure():
    ids = paper_ids

    mu_keys = {ids[0]: 'mu', ids[1]: 'mu_tangent', ids[2]: 'mu_tangent'}

    params_ranges = {}
    params = {}
    for id in ids:
        param, _, _ = cube_eval.load_params_and_logs(id)
        params[id] = param
        params_ranges[id] = sa.get_friction_range(id.split('_')[0], param[mu_keys[id]])
    sweeps = make_training_loss_sensitivity_analysis(ids, params_ranges)

    ## plotting
    legend_strs = []
    ps.set_figsize((8, 6))
    for id in ids:
        legend_strs.append(format_sim_name(id))
        k_opt = params[id][mu_keys[id]]
        k_ratio = np.array(params_ranges[id][mu_keys[id]]) / k_opt
        ps.plot(k_ratio, sweeps[id]['loss_avg'][mu_keys[id]], color=sim_colors[format_sim_name(id)])
    plt.title('Cube Friction Sensitivity')
    plt.xlabel('Normalized Friction Coeff $\mu / \mu^{*}$')
    plt.xscale('log', base=2)
    plt.legend(legend_strs)
    frame = plt.gca()
    frame.axes.get_xaxis().set_ticks([0.5, 0.7, 1.0, 1.4, 2.0])
    frame.axes.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    
    plt.ylabel('Average Error ($e_{q}$)')
    plt.ylim((0, 0.8))
    plt.xlim((k_ratio[0], k_ratio[-1]))
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

    plt.title('Cube Damping Sensitivity')
    plt.xlabel('$b / b^{*}$')
    plt.xscale('log')
    ps.set_figsize((8,6))
    # plt.legend(legend_strs)
    # plt.ylabel('Average $e_{q}$')
    frame = plt.gca()
    frame.axes.get_xaxis().set_ticks([0.1, 0.3, 1.0, 3.1, 10])
    frame.axes.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())

    plt.ylim((0, 0.8))
    plt.xlim((k_ratio[0], k_ratio[-1]))

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
    traj_id  = 259
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

def make_stroboscopic_figure(toss_id):
    learning_result = paper_ids[1]
    eval_sim = cube_eval.get_eval_sim(learning_result)
    params, _, _ = cube_eval.load_params_and_logs(learning_result)
    cube_eval.visualize_learned_params(params, eval_sim, toss_id)

def make_quad_cube_video(toss_id):
    ids = paper_ids
    cube_datas = [load_cube_toss(make_cube_toss_filename(cube_eval.cube_data_folder, toss_id))]
    initial_state = cube_datas[0][0].ravel()
    for id in ids:
        eval_sim = cube_eval.get_eval_sim(id)
        param, _, _ = cube_eval.load_params_and_logs(id)
        eval_sim.init_sim(param)
        sim_data = eval_sim.get_sim_traj_initial_state(
            initial_state, 
            cube_datas[0].shape[0], 
            CUBE_DATA_DT)
        cube_datas.append(sim_data)
    vis_sim = drake_cube_sim.DrakeCubeSim(visualize=True)
    vis_sim.visualize_four_cubes(cube_datas, 0.05)


def make_mujoco_comparison_video():
    make_quad_cube_video(361)

def make_inelastic_traj_video():
    id = 'drake_2021_09_11_16_44_10'
    params, _, _ = cube_eval.load_params_and_logs(id)
    eval_sim = cube_eval.get_eval_sim(id)
    cube_eval.visualize_learned_params(params, eval_sim, 193)

def make_elastic_traj_video():
    id = 'drake_2021_09_11_16_44_10'
    params, _, _ = cube_eval.load_params_and_logs(id)
    eval_sim = cube_eval.get_eval_sim(id)
    cube_eval.visualize_learned_params(params, eval_sim, 313)

def quick_video():
    id = paper_ids[2]
    trajs = [51, 59, 237, 351, 412]
    traj = trajs[4]
    params, _, _ = cube_eval.load_params_and_logs(id)
    eval_sim = cube_eval.get_eval_sim(id)
    cube_eval.visualize_learned_params(params, eval_sim, traj)

if __name__ == '__main__':
    damping_keys = ['dissipation', 'damping', 'damping']
    sim_choice = int(sys.argv[1])
    make_gridded_sensitivity_analysis_figure(
        paper_ids[sim_choice], damping_keys[sim_choice])
    # make_bounces_histogram(paper_ids)
    # plt.show()
    # make_estimated_pdf_figure()
    # make_friction_sensitivity_analysis_figure()
    # make_damping_sensitivity_analysis_figure()
    # make_stiffness_sensitivity_analysis_figure()
    # make_error_vs_time_plot()
    # make_contact_impulse_plot()
    # visualize_cube_initial_condition()
    # make_damping_ratio_sensitivity_analysis_figure()
    # make_inelastic_traj_video()
    # make_elastic_traj_video()
    # quick_video()