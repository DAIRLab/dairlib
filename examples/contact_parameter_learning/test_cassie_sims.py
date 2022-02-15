import drake_cassie_sim_v2
import mujoco_cassie_sim_v2
import isaac_cassie_sim
# import bullet_cassie_sim
import sys
import matplotlib.pyplot as plt


def test_sim(sim):
  params = sim.default_params
  hardware_traj_num = '15'
  sim.make(params, hardware_traj_num)
  rollout = sim.advance_to(0.0495)
  # sim.free_sim()
  rollout.plot_positions()
  sim.hardware_traj.plot_positions()
  # sim.reset(hardware_traj_num)
  # rollout = sim.advance_to(0.0495)
  # sim.hardware_traj.plot_positions()
  # rollout.plot_efforts()
  plt.show()
  # import pdb; pdb.set_trace()

def test_mujoco_sim():
  sim = mujoco_cassie_sim_v2.MuJoCoCassieSim()
  test_sim(sim)

def test_drake_sim():
  sim = drake_cassie_sim_v2.DrakeCassieSim()
  test_sim(sim)

def test_isaac_sim():
  sim = isaac_cassie_sim.IsaacCassieSim()
  test_sim(sim)

# def test_bullet_sim():
#   bullet_cassie_sim.test_sim()
  # sim = bullet_cassie_sim.BulletCassieSim()
  # test_sim(sim)

if __name__ == '__main__':
  # test_drake_sim()
  # test_bullet_sim()
  # test_mujoco_sim()
  test_isaac_sim()
