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
import time
import sys

SIM_ERROR_LOSS = 200

drake_sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5, loss_filename='2021_08_27_weights')
mujoco_sim = mujoco_cassie_sim.LearningMujocoCassieSim(loss_filename='2021_08_27_weights')
# drake_sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5, loss_filename=time.strftime("%Y_%m_%d") + '_weights')
# mujoco_sim = mujoco_cassie_sim.LearningMujocoCassieSim(loss_filename=time.strftime("%Y_%m_%d") + '_weights')
loss_over_time = []
stiffness_over_time = []
params_over_time = []
log_num = 'training'
budget = 5000
# budget = 5000
# budget = 25000

batch_size = 5
all_logs = drake_sim.log_nums_real
num_train = int(0.9 * len(all_logs))
training_idxs = sample(all_logs, num_train)
# training_idxs = ['14']
# training_idxs = [log_num]
test_idxs = [idx for idx in all_logs if not (idx in training_idxs)]

ps = plot_styler.PlotStyler()
ps.set_default_styling(
  directory='/home/yangwill/Documents/research/projects/impact_uncertainty/figures/learning_parameters')


def get_drake_loss(params, log_num=None, plot=False):
  if (log_num == None):
    log_num = choice(training_idxs)
  # print(log_num)
  sim_id = drake_sim.run(params, log_num)
  if sim_id == '-1':
    print('initial state was infeasible')
    loss = SIM_ERROR_LOSS
  else:
    loss = drake_sim.compute_loss(log_num, sim_id, params, plot)
  loss_over_time.append(loss)
  stiffness_over_time.append(params['stiffness'])
  # print('loss:' + str(loss))
  return loss

def get_drake_loss_mp(params):
  loss_sum = 0
  for i in range(batch_size):
    loss_sum += get_drake_loss(params)
  print(loss_sum / batch_size)

  params_over_time.append(params)
  loss_over_time.append(loss_sum / batch_size)
  return loss_sum / batch_size

def get_mujoco_loss(params, log_num=None):
  if (log_num == None):
    log_num = choice(training_idxs)
  print(log_num)
  sim_id = mujoco_sim.run(params, log_num)
  loss = mujoco_sim.compute_loss(log_num, sim_id)
  loss_over_time.append(loss)
  stiffness_over_time.append(params['timeconst'])
  print('loss:' + str(loss))
  return loss


def print_loss_weights(loss_filename):
  new_loss_filename = time.strftime("%Y_%m_%d") + '_weights'
  loss_weights = cassie_loss_utils.CassieLoss(loss_filename)
  loss_weights.weights.vel[17, 17] = 0
  loss_weights.weights.vel[15, 15] = 0
  loss_weights.weights.omega = 5 * np.eye(3)
  loss_weights.weights.impulse_weight = 1e-7
  loss_weights.weights.save(new_loss_filename)
  loss_weights.print_weights()

def learn_drake_cassie_params(batch=False):
  optimization_param = ng.p.Dict(
    mu_static=ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio=ng.p.Scalar(lower=0.001, upper=1.0),
    stiffness=ng.p.Log(lower=1e3, upper=1e8),
    dissipation=ng.p.Scalar(lower=0.1, upper=1),
    # stiction_tol=ng.p.Log(lower=1e-4, upper=1e-2),
    # vel_offset=ng.p.Array(shape=(3,)).set_bounds(lower=-0.5, upper=0.5),
    # z_offset=ng.p.Array(shape=(1,)).set_bounds(lower=-0.05, upper=0.05)
    # vel_offset=ng.p.Array(shape=(len(all_logs) * 3,)).set_bounds(lower=-1, upper=1),
    # z_offset=ng.p.Array(shape=(len(all_logs),)).set_bounds(lower=-0.05, upper=0.05)
  )

  optimization_param.value = drake_sim.default_drake_contact_params
  # optimization_param.value=drake_sim.load_params('15' + '_optimized_params_5000').value
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  if batch:
    params = optimizer.minimize(get_drake_loss_mp)
  else:
    params = optimizer.minimize(get_drake_loss)
  loss = np.array(loss_over_time)
  stiffness = np.array(stiffness_over_time)
  np.save(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(drake_sim.params_folder + log_num + '_stiffness_trajectory_' + str(budget), stiffness)
  drake_sim.save_params(params, '_training_' + str(budget))
  # drake_sim.save_params(params, log_num + '_x_offsets_' + str(budget))
  print('optimal params:')
  print(params)


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
  stiffness = np.array(stiffness_over_time)
  np.save(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget), loss)
  np.save(drake_sim.params_folder + log_num + '_stiffness_trajectory_' + str(budget), stiffness)
  mujoco_sim.save_params(params, log_num + '_optimized_params_' + str(budget))


def plot_loss_trajectory():
  loss_t = np.load(drake_sim.params_folder + log_num + '_loss_trajectory_' + str(budget) + '.npy')
  stiffness_t = np.load(drake_sim.params_folder + log_num + '_stiffness_trajectory_' + str(budget) + '.npy')
  # ps.scatter(stiffness_t, loss_t, xlabel='penetration_allowance (m)', ylabel='loss')
  ps.plot(np.arange(0, loss_t.shape[0]), loss_t, xlabel='iter', ylabel='loss')
  plt.show()


def print_drake_cassie_params(single_log_num, plot=False):
  # optimal_params = drake_sim.load_params(log_num + '_optimized_params_' + str(budget))
  # optimal_params = drake_sim.load_params('all' + '_optimized_params_' + str(budget))
  # optimal_params = drake_sim.load_params('old_params/' + log_num + '_optimized_params_' + str(budget))
  # optimal_params = drake_sim.load_params('drake_2021_08_25_11_45_training_15000')
  optimal_params = drake_sim.load_params('drake_2021_08_31_15_training_15000').value
  # optimal_params = drake_sim.load_params('drake_2021_08_28_22_26' + single_log_num + '_x_offsets_5000').value

  loss = get_drake_loss(optimal_params, single_log_num, plot)
  # loss = get_drake_loss(drake_sim.default_drake_contact_params, single_log_num, plot)
  print(single_log_num)
  # import pdb; pdb.set_trace()
  # z_offset = optimal_params['z_offset']
  # vel_offset = optimal_params['vel_offset']
  stiffness = optimal_params['stiffness']
  dissipation = optimal_params['dissipation']
  # print('z_offset')
  # print(z_offset)
  # print('vel_offset')
  # print(vel_offset)
  print('stiffness')
  print(stiffness)
  print('dissipation')
  print(dissipation)
  # np.save(drake_sim.params_folder + log_num + '_z_offset_' + str(budget), z_offset)
  # np.save(drake_sim.params_folder + log_num + '_vel_offset_' + str(budget), vel_offset)
  # return z_offset, vel_offset
  return


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


def save_x_offsets():
  z_offsets = {}
  vel_offsets = {}
  for i in all_logs:
    log_num = i
    # z_offset, vel_offset = print_drake_cassie_params(log_num)
    print_drake_cassie_params(log_num)
    # z_offsets[log_num] = z_offset
    # vel_offsets[log_num] = vel_offset
  with open(drake_sim.params_folder + 'optimized_z_offsets.pkl', 'wb') as file:
    pickle.dump(z_offsets, file, protocol=pickle.HIGHEST_PROTOCOL)
  with open(drake_sim.params_folder + 'optimized_vel_offsets.pkl', 'wb') as file:
    pickle.dump(vel_offsets, file, protocol=pickle.HIGHEST_PROTOCOL)

def print_params():
  global log_num
  for i in all_logs:
    print(i)
    log_num = i
    print_drake_cassie_params(log_num)


def learn_x_offsets():
  global training_idxs
  global log_num
  for i in all_logs:
    print(i)
    log_num = i
    training_idxs = [log_num]
    learn_drake_cassie_params()

if (__name__ == '__main__'):
  # print_loss_weights('pos_loss_weights')
  # learn_x_offsets()
  # save_x_offsets()
  # print_params()
  # import pdb; pdb.set_trace()
  # learn_drake_cassie_params()
  # learn_drake_cassie_params(batch=True)
  # print_drake_cassie_params(i)
  # learn_mujoco_cassie_params()
  # plot_per_log_loss_drake()
  # plot_per_log_loss_mujoco()
  # print_mujoco_cassie_params()
  # log_num = '33'
  print_drake_cassie_params(str(sys.argv[1]), True)
  # plot_loss_trajectory()
  pass