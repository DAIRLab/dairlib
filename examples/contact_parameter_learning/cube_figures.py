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

def plot_damping_ratios(ids):
    stiffness = []
    zeta = []
    for id in ids:
        params, _, _ = cube_eval.load_params_and_logs(id)
        stiffness.append(params['stiffness'])
        zeta.append(cube_eval.calc_damping_ratio(params))

    plt.scatter(stiffness, zeta)
    plt.show()


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
    
    filename += '.pdf'
    plt.ylabel('PDF Estimate')
    plt.xlabel('$e_{tot}(\hat{x}_{t}, x_{t}^{*})$')
    plt.legend(legend_strs)
    ps.save_fig(filename)

def plot_sensitivity_analysis(loss_sweeps, params_range, title=''):
    for key in loss_sweeps:
        plt.figure()
        ps.plot(params_range[key], loss_sweeps[key], color=ps.blue)
        plt.title(f'{title} - {key}')
        if (key == 'pen_allow' or key == 'stiction_tol' or key == 'mu_torsion' or key == 'mu_rolling'):
            plt.xscale('log')

def make_pos_rot_sensitivity_analysis(ids, params_ranges):
    sweeps = {}
    for id in ids:
        sim_type = id.split('_')[0]
        sim = cube_eval.get_eval_sim(id)
        params, _, _ = cube_eval.load_params_and_logs(id)
        pos_avg, pos_med, rot_avg, rot_med = \
             sa.get_cube_position_and_rotation_error_sensitivity(
                sim, 
                params, 
                params_ranges[id], 
                range(550))

        sweeps[id] = {'pos_avg' : pos_avg, 
                      'pos_med': pos_med, 
                      'rot_avg': rot_avg, 
                      'rot_med': rot_med}
    return sweeps

def make_stiffness_sensitivity_analysis_figure():
    ids = ['mujoco_2021_08_31_13_59_10',
           'drake_2021_08_31_11_32_10', 
           'bullet_2021_08_31_12_16_10']
    params_ranges = {}
    for id in ids:
        params_ranges[id] = sa.get_stiffness_range(id.split('_')[0])

    sweeps = make_pos_rot_sensitivity_analysis(ids, params_ranges)

    ## plotting
    for id in ids:
        ps.plot(params_ranges[id]['stiffness'], sweeps[id]['pos_rot']['stiffness'])

    plt.show()
      

def make_estimated_pdf_figure():
    ids = ['mujoco_2021_08_31_13_59_10',
           'drake_2021_08_31_11_32_10', 
           'bullet_2021_08_31_12_16_10']
    _, mse, _, _, _ = cube_eval.load_list_of_results(
        ids, cube_eval.mse_loss)

    plot_estimated_loss_pdfs(mse)


def make_mujoco_damping_ratio_figure():
    id_paths = glob.glob('examples/contact_parameter_learning/learned_parameters/cube/mujoco_2021_0*_10.json')
    ids = [os.path.basename(id_path).split('.')[0] for id_path in id_paths]
    plot_damping_ratios(ids)

def make_bullet_damping_ratio_figure():
    id_paths = glob.glob('examples/contact_parameter_learning/learned_parameters/cube/bullet_2021_0*_10.json')
    ids = [os.path.basename(id_path).split('.')[0] for id_path in id_paths]
    plot_damping_ratios(ids)

if __name__ == '__main__':
    #make_estimated_pdf_figure()
    make_stiffness_sensitivity_analysis_figure()
