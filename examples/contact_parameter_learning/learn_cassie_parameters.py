import nevergrad as ng
import drake_cassie_sim
import mujoco_cassie_sim
import os
import pickle
from json import dump, load
import cassie_loss_utils
import numpy as np
import plot_styler
import matplotlib.pyplot as plt
from random import sample, choice

SIM_ERROR_LOSS = 100

drake_sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5, loss_filename='pos_loss_weights')
mujoco_sim = mujoco_cassie_sim.LearningMujocoCassieSim(loss_filename='pos_loss_weights')
loss_over_time = []
pen_allow_over_time = []
log_num = '15_24'
budget = 5000
# budget = 25000

all_logs = drake_sim.log_nums_real
# num_train = int(0.8 * len(all_logs))
# training_idxs = sample(all_logs, num_train)
training_idxs = ['15']
test_idxs = [idx for idx in all_logs if not (idx in training_idxs)]

ps = plot_styler.PlotStyler()
ps.set_default_styling(
  directory='/home/yangwill/Documents/research/projects/impact_uncertainty/figures/learning_parameters')


def get_drake_loss(params, log_num=None):
  if (log_num == None):
    log_num = choice(training_idxs)
  print(log_num)
  sim_id = drake_sim.run(params, log_num)
  loss = drake_sim.compute_loss(log_num, sim_id, params)
  loss_over_time.append(loss)
  # pen_allow_over_time.append(params['pen_allow'])
  print('loss:' + str(loss))
  return loss


def get_mujoco_loss(params, log_num=None):
  if (log_num == None):
    log_num = choice(training_idxs)
  print(log_num)
  sim_id = mujoco_sim.run(params, log_num)
  loss = mujoco_sim.compute_loss(log_num, sim_id)
  loss_over_time.append(loss)
  pen_allow_over_time.append(params['timeconst'])
  print('loss:' + str(loss))
  return loss


def learn_drake_cassie_params():
  optimization_param = ng.p.Dict(
    mu_static=ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio=ng.p.Scalar(lower=0.001, upper=1.0),
    stiffness=ng.p.Log(lower=1e3, upper=1e8),
    dissipation=ng.p.Scalar(lower=0.1, upper=1),
    stiction_tol=ng.p.Log(lower=1e-4, upper=1e-2),
    vel_offset=ng.p.Array(shape=(3,)).set_bounds(lower=-1, upper=1),
    z_offset=ng.p.Array(shape=(1,)).set_bounds(lower=-0.05, upper=0.05)
    # vel_offset=ng.p.Array(shape=(len(all_logs) * 3,)).set_bounds(lower=-1, upper=1),
    # z_offset=ng.p.Array(shape=(len(all_logs),)).set_bounds(lower=-0.05, upper=0.05)
  )

  optimization_param.value = drake_sim.default_drake_contact_params
  # optimization_param.value=drake_sim.load_params('15' + '_optimized_params_5000').value
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  params = optimizer.minimize(get_drake_loss)
  loss = np.array(loss_over_time)
  pen_allow = np.array(pen_allow_over_time)
  np.save(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(drake_sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget), pen_allow)
  drake_sim.save_params(params, log_num + '_optimized_params_' + str(budget))


def learn_mujoco_cassie_params():
  optimization_param = ng.p.Dict(
    timeconst=ng.p.Log(lower=1e-4, upper=1e-2),
    dampratio=ng.p.Scalar(lower=1e-2, upper=1e1),
    ground_mu_tangent=ng.p.Scalar(lower=0.01, upper=1.0),
    mu_torsion=ng.p.Scalar(lower=0.00001, upper=1.0),
    mu_rolling=ng.p.Log(lower=0.000001, upper=0.01)
  )

  optimization_param.value = mujoco_sim.default_mujoco_contact_params
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  params = optimizer.minimize(get_mujoco_loss)
  loss = np.array(loss_over_time)
  pen_allow = np.array(pen_allow_over_time)
  np.save(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(drake_sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget), pen_allow)
  drake_sim.save_params(params, log_num + '_optimized_params_' + str(budget))


def plot_loss_trajectory():
  loss_t = np.load(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget) + '.npy')
  pen_allow_t = np.load(drake_sim.params_folder + log_num + '_pen_allow_trajectory_' + str(budget) + '.npy')
  # ps.scatter(pen_allow_t, loss_t, xlabel='penetration_allowance (m)', ylabel='loss')
  ps.plot(np.arange(0, loss_t.shape[0]), loss_t, xlabel='iter', ylabel='loss')
  plt.show()


def print_drake_cassie_params(single_log_num):
  # optimal_params = drake_sim.load_params(log_num + '_optimized_params_' + str(budget))
  # optimal_params = drake_sim.load_params('all' + '_optimized_params_' + str(budget))
  optimal_params = drake_sim.load_params('15_24' + '_optimized_params_' + str(budget))

  sim_id = drake_sim.run(optimal_params.value, single_log_num)
  loss = drake_sim.compute_loss(single_log_num, sim_id, optimal_params.value, plot=True)
  import pdb; pdb.set_trace()
  print(loss)
  z_offsets = optimal_params.value['z_offset']
  vel_offsets = optimal_params.value['vel_offset']
  # np.save(drake_sim.params_folder + log_num + '_z_offset_' + str(budget), z_offsets)
  # np.save(drake_sim.params_folder + log_num + '_vel_offset_' + str(budget), vel_offsets)
  return loss


def print_mujoco_cassie_params(single_log_num):
  # optimal_params = drake_sim.load_params(log_num + '_optimized_params_' + str(budget))
  optimal_params = mujoco_sim.load_params('all' + '_optimized_params_' + str(budget))
  sim_id = mujoco_sim.run(optimal_params.value, single_log_num)
  loss = mujoco_sim.compute_loss(single_log_num, sim_id)
  print(loss)
  # z_offsets = optimal_params.value['z_offset']
  # vel_offsets = optimal_params.value['vel_offset']
  # np.save(drake_sim.params_folder + log_num + '_z_offset_' + str(budget), z_offsets)
  # np.save(drake_sim.params_folder + log_num + '_vel_offset_' + str(budget), vel_offsets)
  return loss


def plot_per_log_loss_drake():
  log_nums = []
  losses = []
  for log_num_ in all_logs:
    print(log_num_)
    losses.append(print_drake_cassie_params(log_num_))
    log_nums.append((log_num_))
  ps.plot(log_nums, losses, xlabel='log number', ylabel='best loss')
  plt.show()


def plot_per_log_loss_mujoco():
  log_nums = []
  losses = []
  for log_num_ in all_logs:
    print(log_num_)
    losses.append(print_mujoco_cassie_params(log_num_))
    log_nums.append((log_num_))
  ps.plot(log_nums, losses, xlabel='log number', ylabel='best loss')
  plt.show()


if (__name__ == '__main__'):
  learn_drake_cassie_params()
  # learn_mujoco_cassie_params()
  # plot_per_log_loss_drake()
  # plot_per_log_loss_mujoco()
  # print_drake_cassie_params('15')
  # print_mujoco_cassie_params()
  # plot_loss_trajectory()
