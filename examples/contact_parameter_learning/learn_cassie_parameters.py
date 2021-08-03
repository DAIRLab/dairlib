import nevergrad as ng
import drake_cassie_sim
import os
from json import dump, load


def get_drake_loss(params):
  sim = drake_cassie_sim.DrakeCassieSim()
  sim_id = sim.run()
  loss = sim.compute_loss(log_num, sim_id)

  return

def learn_drake_cassie_params():
  sim = drake_cassie_sim.DrakeCassieSim()
  # sim.run(sim.default_drake_contact_params, '27')

  optimization_param = ng.p.Dict(
    mu_static = ng.p.Scalar(lower=0.001, upper=1.0),
    mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
    pen_allow=ng.p.Log(lower=1e-10, upper=1e-1),
    stiction_tol=ng.p.Log(lower=1e-6, upper=1e-1)
  )

  optimization_param.value=default_drake_contact_params
  optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=10000)
  params = optimizer.minimize(get_drake_loss)
  save_params('drake', 33, params.value)

if (__name__ == '__main__'):
  learn_drake_cassie_params()
  #learn_drake_params()