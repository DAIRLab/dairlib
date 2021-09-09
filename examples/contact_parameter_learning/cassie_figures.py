import pickle
import time
from bindings.pydairlib.common.plot_styler import PlotStyler
from matplotlib import pyplot as plt

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
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/mujoco_to_real_comparison/'
  figure_data_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/figures/figure_data/'
  date_prefix = time.strftime("%Y_%m_%d_%H")
  ps = PlotStyler()
  ps.set_default_styling(directory=figure_directory)
  with open("x_datatypes", "rb") as fp:
    x_datatypes = pickle.load(fp)

  saved_data = {'sensitivity_analysis' : figure_data_directory + '2021_09_09_13.pkl'}

  make_stiffness_sensivity_analysis_figure(use_saved_data=True)

  return


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

def make_stiffness_sensivity_analysis_figure(use_saved_data=False):
  ids = ['drake_2021_09_08_16_training_5000',
         'mujoco_2021_09_08_17_training_5000']
  params_ranges = {}
  for id in ids:
    params_ranges[id] = get_cassie_params_range(id.split('_')[0])

  if use_saved_data:
    with open(saved_data['sensitivity_analysis'], 'rb') as f:
      sweeps = pickle.load(f)
  else:
    sweeps = get_sensitivity_analysis_results(ids, params_ranges)
    with open(figure_data_directory + date_prefix + '.pkl', 'wb') as f:
      pickle.dump(sweeps, f, pickle.HIGHEST_PROTOCOL)


  for id in ids:
    ps.plot(params_ranges[id]['stiffness'], sweeps[id]['avg']['stiffness'])
    ps.plot(params_ranges[id]['stiffness'], sweeps[id]['med']['stiffness'])

  ps.show_fig()

if __name__ == '__main__':
  main()
