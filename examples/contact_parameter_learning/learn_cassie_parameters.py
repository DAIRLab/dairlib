import nevergrad as ng
import drake_cassie_sim
import os
import pickle
from json import dump, load
import cassie_loss_utils



def get_drake_loss(params):
  log_num = '27'
  sim = drake_cassie_sim.DrakeCassieSim()
  sim_id = sim.run(params, log_num)
  loss = sim.compute_loss(log_num, sim_id)

  return

def learn_drake_cassie_params():
  # loss_weights_file = 'default_loss_weights'
  sim = drake_cassie_sim.DrakeCassieSim()
  # sim.run(sim.default_drake_contact_params, '27')

  optimization_param = ng.p.Dict(
    mu_static = ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
    pen_allow=ng.p.Log(lower=1e-6, upper=1e-1),
    stiction_tol=ng.p.Log(lower=1e-6, upper=1e-1)
  )

  optimization_param.value=sim.default_drake_contact_params
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=10000)
  params = optimizer.minimize(get_drake_loss)
  sim.save_params(params, "optimized_params")

if (__name__ == '__main__'):
  learn_drake_cassie_params()
  #learn_drake_params()