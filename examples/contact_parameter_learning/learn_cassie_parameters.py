import nevergrad as ng
import drake_cassie_sim
import os
import pickle
from json import dump, load
import cassie_loss_utils
import numpy as np

sim = drake_cassie_sim.DrakeCassieSim()
loss_over_time = []

def get_drake_loss(params):
  log_num = '27'
  sim_id = sim.run(params, log_num)
  loss = sim.compute_loss(log_num, sim_id)
  print('loss:' + str(loss))
  loss_over_time.append(loss)
  # print(loss_over_time)
  return loss

def learn_drake_cassie_params():
  # loss_weights_file = 'default_loss_weights'
  sim = drake_cassie_sim.DrakeCassieSim()
  # sim.run(sim.default_drake_contact_params, '27')

  optimization_param = ng.p.Dict(
    mu_static = ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
    pen_allow=ng.p.Log(lower=1e-5, upper=1e-1),
    stiction_tol=ng.p.Log(lower=1e-4, upper=1e-1)
  )

  budget = 1000
  optimization_param.value=sim.default_drake_contact_params
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=budget)
  params = optimizer.minimize(get_drake_loss)
  loss = np.array(loss_over_time)
  np.save(sim.params_folder + 'loss_trajectory_' + str(budget), loss)
  sim.save_params(params, "optimized_params_" + str(budget))

def print_drake_cassie_params():
  optimal_params = sim.load_params('optimized_params')
  import pdb; pdb.set_trace()

if (__name__ == '__main__'):
  print_drake_cassie_params()
  # learn_drake_cassie_params()
  #learn_drake_params()