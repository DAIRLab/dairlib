import drake_cassie_sim_v2
import sys
import matplotlib.pyplot as plt


def test_mujoco_sim():
  sim = mujoco_cassie_sim.MuJoCoCassieSim()
  params = sim.default_params
  dataset_num = '15'
  sim.make(params, dataset_num)

if __name__ == '__main__':
  sim = drake_cassie_sim_v2.DrakeCassieSim()

  params = sim.default_params
  dataset_num = '15'
  sim.make(params, dataset_num)
  rollout = sim.advance_to(0.0495)
  rollout.plot_positions()
  sim.hardware_traj.plot_positions()
  # rollout.plot_efforts()
  # sim.hardware_traj.plot_efforts()
  plt.show()

