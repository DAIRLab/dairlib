import pickle
import time
from bindings.pydairlib.common.plot_styler import PlotStyler
from matplotlib import pyplot as plt
import numpy as np
from plotting_utils import format_sim_name
from scipy.stats import gaussian_kde
from cassie_loss_plotter import *
from sensitivity_analysis import *

def main():
  global ps
  global figure_directory
  global figure_data_directory
  global data_directory
  global sim_data_directory
  global x_datatypes
  global kinematics_calculator
  global date_prefix
  global saved_data

  data_directory = '/home/yangwill/Documents/research/projects/invariant_impacts/data/'
  sim_data_directory = '/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data/'
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/sensitivity_analysis/'
  figure_data_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/figure_data/'
  date_prefix = time.strftime("%Y_%m_%d_%H")
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory, figsize=[8, 6])
  with open("x_datatypes", "rb") as fp:
    x_datatypes = pickle.load(fp)

  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_09_15.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_10_11.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_10_13.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_10_14_test.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_10_21_combined.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_11_12.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_11_20.pkl'}
  # saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_12_12.pkl'}
  saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_13_00.pkl'}

  # make_stiffness_sensivity_analysis_figure(use_saved_data=True)
  make_stiffness_sensivity_analysis_figure(use_saved_data=True, save_figs=True)
  # make_stiffness_sensivity_analysis_figure()

  # make_estimated_pdf_figure()

  return

def plot_estimated_loss_pdfs(losses):
  training_results = list(losses.keys())
  i = 0
  legend_strs = []
  filename = ''
  for result in training_results:
    # loss = np.array(list(losses[result].values()))
    loss = losses[result]
    # loss *= 1/np.average(loss)
    pts = np.linspace(np.min(loss), np.max(loss), 100)
    pdf = gaussian_kde(loss).pdf(pts)
    ps.plot(pts, pdf, color=ps.penn_color_wheel[i])
    legend_strs.append(format_sim_name(result))# + ' $T_{sim}$ = ' + str(148 * int(result.split('_')[-1])) + ' Hz')
    # filename += format_sim_name(result) + '_' + result.split('_')[-1] + '_'
    filename += format_sim_name(result) + '_'
    i += 1

  filename += 'estimated_pdf'
  plt.ylabel('PDF Estimate')
  plt.xlabel('$e_{tot}(\hat{x}_{t}, x_{t}^{*})$')
  plt.legend(legend_strs)
  ps.save_fig(filename)

def make_estimated_pdf_figure():
  ids = ['mujoco_2021_09_11_18_training_2500',
         'drake_2021_09_10_17_training_2500']

  drake_losses = np.load(figure_data_directory + 'drake_losses.npy')
  mujoco_losses = np.load(figure_data_directory + 'mujoco_losses.npy')
  losses = {}
  losses['mujoco'] = mujoco_losses
  losses['drake'] = drake_losses
  plot_estimated_loss_pdfs(losses)

def get_sensitivity_analysis_results(ids, param_ranges):
  sweeps = {}
  for id in ids:
    sim_type = id.split('_')[0]
    eval_sim = get_cassie_sim(id)
    params_range = get_cassie_params_range(sim_type)
    params = eval_sim.load_params(id).value
    avg, med = get_sensitivity_analysis(
      eval_sim,
      None,
      params,
      params_range,
      None,
      plant='cassie')

    sweeps[id] = {'avg': avg,
                  'med': med}
  return sweeps



def make_stiffness_sensivity_analysis_figure(use_saved_data=False, save_figs=False):
  # ids = ['drake_2021_09_08_16_training_5000',
  #        'mujoco_2021_09_08_17_training_5000']
  ids = ['drake_2021_09_10_17_training_2500',
         'mujoco_2021_09_12_22_training_2500']
  # ids = ['mujoco_2021_09_09_15_training_5000',
  #        'drake_2021_09_09_15_training_5000']
  # ids = ['drake_2021_09_09_15_training_5000']

  params_ranges = {}
  for id in ids:
    params_ranges[id] = get_cassie_params_range(id.split('_')[0])

  save_data_file = figure_data_directory + date_prefix + '.pkl'

  sweeps = {}
  if use_saved_data:
    with open(saved_data['sensitivity_analysis'], 'rb') as f:
      sweeps = pickle.load(f)
  else:
    print('saving to: ' + save_data_file)
    sweeps = get_sensitivity_analysis_results(ids, params_ranges)
    with open(save_data_file, 'wb') as f:
      pickle.dump(sweeps, f, pickle.HIGHEST_PROTOCOL)

  figures = {}

  for id in ids:
    sim_type = id.split('_')[0]
    color = ps.penn_color_wheel[0]
    if sim_type == 'drake':
      color = ps.penn_color_wheel[0]
    elif sim_type == 'mujoco':
      color = ps.penn_color_wheel[1]

    for param in params_ranges[id].keys():
      param_name = param
      if 'mu' in param:
        param_name = 'mu'
      # plt.figure(id)
      # plt.figure(param)
      # plt.figure(sim_type + '_' + param)
      figures[param_name] = plt.figure(param_name)
      figures[param_name].gca()
      ps.plot(params_ranges[id][param], sweeps[id]['avg'][param], xlabel=param, ylabel='Loss ', color=color, ylim=[0, 70], xlim=[0, params_ranges[id][param][-1]])
      # ps.plot(params_ranges[id]['stiffness'], sweeps[id]['avg']['stiffness'], xlabel=param, ylabel='Loss ', color=color)
      # ps.plot(params_ranges[id][param], sweeps[id]['med'][param])
      if save_figs and sim_type == 'mujoco':
        ps.save_fig('both' + '_' + param_name + '_avg')
        # ps.save_fig(sim_type + '_' + param + '_avg')
  ps.show_fig()


if __name__ == '__main__':
  main()
