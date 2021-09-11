import drake_cassie_sim
import sys

if __name__ == '__main__':
  sim = drake_cassie_sim.DrakeCassieSim(drake_sim_dt=8e-5)
  params = sim.default_drake_contact_params
  # sim.sim_time = 0.005
  sim_id = sim.run(params, sys.argv[1])
