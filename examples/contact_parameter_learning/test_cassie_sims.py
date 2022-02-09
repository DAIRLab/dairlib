import drake_cassie_sim_v2
import mujoco_cassie_sim_v2
import sys
import matplotlib.pyplot as plt


def test_sim(sim):
  params = sim.default_params
  dataset_num = '15'
  sim.make(params, dataset_num)
  rollout = sim.advance_to(0.0495)
  sim.free_sim()
  rollout.plot_positions()
  sim.hardware_traj.plot_positions()
  import pdb; pdb.set_trace()
  plt.show()

def test_mujoco_sim():
  sim = mujoco_cassie_sim_v2.MuJoCoCassieSim()
  test_sim(sim)

def test_drake_sim():
  sim = drake_cassie_sim_v2.DrakeCassieSim()
  test_sim(sim)

if __name__ == '__main__':
  # test_drake_sim()
  test_mujoco_sim()
