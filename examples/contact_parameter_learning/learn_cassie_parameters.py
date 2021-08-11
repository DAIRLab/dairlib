import nevergrad as ng
import drake_cassie_sim
import os
import pickle
from json import dump, load
import cassie_loss_utils
import numpy as np
import plot_styler
import matplotlib.pyplot as plt

sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5)
loss_over_time = []
pen_allow_over_time = []
log_num = '15'
budget = 2500

def get_drake_loss(params):
  sim_id = sim.run(params, log_num)
  loss = sim.compute_loss(log_num, sim_id)
  print('loss:' + str(loss))
  loss_over_time.append(loss)
  pen_allow_over_time.append(params['pen_allow'])
  return loss

def learn_drake_cassie_params():
  # loss_weights_file = 'default_loss_weights'
  # sim = drake_cassie_sim.DrakeCassieSim()
  # sim.run(sim.default_drake_contact_params, '27')

  optimization_param = ng.p.Dict(
    mu_static = ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
    pen_allow=ng.p.Log(lower=5e-6, upper=5e-2),
    stiction_tol=ng.p.Log(lower=1e-4, upper=1e-2)
  )

  optimization_param.value=sim.default_drake_contact_params
  # optimization_param.value={
  #   "pen_allow": 1e-5}
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  params = optimizer.minimize(get_drake_loss)
  loss = np.array(loss_over_time)
  pen_allow = np.array(pen_allow_over_time)
  np.save(sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget), pen_allow)
  sim.save_params(params, log_num + '_optimized_params_' + str(budget))

def learn_mujoco_cassie_params():
  # loss_weights_file = 'default_loss_weights'
  # sim = drake_cassie_sim.DrakeCassieSim()
  # sim.run(sim.default_drake_contact_params, '27')

  optimization_param = ng.p.Dict(
    mu_static = ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
    pen_allow=ng.p.Log(lower=5e-6, upper=5e-2),
    stiction_tol=ng.p.Log(lower=1e-4, upper=1e-2)
  )

  optimization_param.value=sim.default_drake_contact_params
  # optimization_param.value={
  #   "pen_allow": 1e-5}
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  params = optimizer.minimize(get_drake_loss)
  loss = np.array(loss_over_time)
  pen_allow = np.array(pen_allow_over_time)
  np.save(sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget), pen_allow)
  sim.save_params(params, log_num + '_optimized_params_' + str(budget))

def plot_loss_trajectory():
  loss_t = np.load(sim.params_folder + log_num + '_loss_trajectory_' + str(budget) + '.npy')
  pen_allow_t = np.load(sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget) + '.npy')
  sim.ps.scatter(pen_allow_t, loss_t, xlabel='penetration_allowance (m)', ylabel='loss')
  plt.show()

def print_drake_cassie_params():
  # optimal_params = sim.load_params('optimized_params')
  optimal_params = sim.load_params(log_num + '_optimized_params_' + str(budget))
  sim_id = sim.run(optimal_params.value, log_num)
  loss = sim.compute_loss(log_num, sim_id, plot=False)
  print(loss)
  import pdb; pdb.set_trace()

if (__name__ == '__main__'):
  # sim.run(sim.default_drake_contact_params, log_num)
  # plot_loss_trajectory()
  print_drake_cassie_params()
  # learn_drake_cassie_params()
  #learn_drake_params()