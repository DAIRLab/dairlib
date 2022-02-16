import drake_cassie_sim_v2
import mujoco_cassie_sim_v2
import isaac_cassie_sim
import bullet_cassie_sim
import sys
import matplotlib.pyplot as plt
import numpy as np


def test_all_trajs(sim):
  params = sim.default_params
  hardware_traj_num = '15'
  sim.make(params, hardware_traj_num)
  hardware_trajs = np.arange(0, 29)
  hardware_traj_nums = ["%.2d" % i for i in hardware_trajs]
  for i in hardware_traj_nums:
    print(i)
    sim.reset(i)
    rollout = sim.advance_to(0.0495)

def test_sim(sim):
  params = sim.default_params
  hardware_traj_num = '15'
  sim.make(params, hardware_traj_num)
  rollout = sim.advance_to(0.0495)
  # sim.free_sim()

  # rollout.plot_positions()
  # sim.hardware_traj.plot_positions()

  rollout.plot_velocities()
  sim.hardware_traj.plot_velocities()

  # rollout.plot_efforts()
  # sim.hardware_traj.plot_efforts()
  plt.show()


def test_mujoco_sim():
  # sim = mujoco_cassie_sim_v2.MuJoCoCassieSim()
  sim = mujoco_cassie_sim_v2.MuJoCoCassieSim(visualize=True)
  test_sim(sim)
  # test_all_trajs(sim)

def test_drake_sim():
  sim = drake_cassie_sim_v2.DrakeCassieSim()
  test_sim(sim)
  # test_all_trajs(sim)

def test_isaac_sim():
  # sim = isaac_cassie_sim.IsaacCassieSim()
  sim = isaac_cassie_sim.IsaacCassieSim(visualize=True)
  # test_sim(sim)
  test_all_trajs(sim)

def test_bullet_sim():
  sim = bullet_cassie_sim.BulletCassieSim(visualize=True)
  test_sim(sim)

if __name__ == '__main__':
  # test_drake_sim()
  # test_mujoco_sim()
  # test_isaac_sim()
  test_bullet_sim()
