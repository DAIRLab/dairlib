import evaluate_cube_parameters as cube_eval
import os
import numpy as np
from scipy.stats import gaussian_kde
from matplotlib import pyplot as plt
from plotting_utils import format_sim_name
from pydairlib.common.plot_styler import PlotStyler


figure_directory = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/figures/')

ps = PlotStyler()
ps.set_default_styling(directory=figure_directory, figsize=(10,6))


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
    plt.xlabel('$e(\hat{x}_{t}, x_{t}^{*})$')
    plt.legend(legend_strs)
    ps.save_fig(filename)

def make_estimated_pdf_figure():
    ids = ['mujoco_2021_08_31_13_59_10',
           'drake_2021_08_31_11_32_10', 
           'bullet_2021_08_31_12_16_10']
    _, mse, _, _, _ = cube_eval.load_list_of_results(
        ids, cube_eval.mse_loss)

    plot_estimated_loss_pdfs(mse)

if __name__ == '__main__':
    make_estimated_pdf_figure()

