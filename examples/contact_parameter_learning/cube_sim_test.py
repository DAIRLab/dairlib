import numpy as np
import mujoco_cube_sim
import drake_cube_sim
import bullet_cube_sim
import cube_sim
from timeit import default_timer as timer
from learn_cube_parameters import cube_data_folder, drake_data_folder, \
     mujoco_data_folder, bullet_data_folder, num_trials

test_state = np.array([ 0.18629883,  0.02622872,  1.89283257, -0.52503014,  0.39360754,
       -0.29753734, -0.67794127,  0.01438053,  1.29095332, -0.21252927,
        1.46313532, -4.85439428,  9.86961928]).reshape(-1,1)

def test_drake():
    drake_sim = drake_cube_sim.DrakeCubeSim(visualize=False, substeps=10)
    contact_params = drake_cube_sim.default_drake_contact_params
    drake_sim.init_sim(contact_params)
    state_traj = drake_sim.get_sim_traj_initial_state(test_state, 300, cube_sim.CUBE_DATA_DT)
    return state_traj

def test_mujoco():
    mujoco_sim = mujoco_cube_sim.MujocoCubeSim(visualize=False)
    contact_params = mujoco_cube_sim.default_mujoco_contact_params
    mujoco_sim.init_sim(contact_params)
    state_traj = mujoco_sim.get_sim_traj_initial_state(test_state, 300, cube_sim.CUBE_DATA_DT)
    return state_traj

def test_bullet():
    bullet_sim = bullet_cube_sim.BulletCubeSim(visualize=False)
    contact_params = bullet_cube_sim.default_bullet_contact_params
    bullet_sim.init_sim(contact_params)
    state_traj = bullet_sim.get_sim_traj_initial_state(test_state, 300, cube_sim.CUBE_DATA_DT)
    return state_traj

def make_mujoco_sim_tests():
    sim = mujoco_cube_sim.MujocoCubeSim(visualize=False)
    cube_sim.make_simulated_traj_data_for_training(mujoco_cube_sim.default_mujoco_contact_params, 
    range(num_trials), cube_data_folder, mujoco_data_folder, sim)

def make_drake_sim_tests():
    sim = drake_cube_sim.DrakeCubeSim(visualize=False)
    cube_sim.make_simulated_traj_data_for_training(drake_cube_sim.default_drake_contact_params, 
    range(num_trials), cube_data_folder, drake_data_folder, sim)

def make_bullet_sim_tests():
    sim = bullet_cube_sim.BulletCubeSim(visualize=False, substeps=10)
    cube_sim.make_simulated_traj_data_for_training(bullet_cube_sim.default_bullet_contact_params, 
    range(num_trials), cube_data_folder, bullet_data_folder, sim)

if (__name__ == "__main__"):
    # sim = drake_cube_sim.DrakeCubeSim(visualize=True)
    # loss = cube_sim.calculate_cubesim_loss(drake_cube_sim.default_drake_contact_params, 100, cube_data_folder, sim, cube_sim.LossWeights())
    # traj1 = test_drake()
    # traj2 = test_bullet()
    # vis_sim = drake_cube_sim.DrakeCubeSim()
    # vis_sim.visualize_two_cubes(traj1, traj2, 0.2)

    for i in range(100000):
        test_drake()

    # # Profiling quaternion losses
    # w = cube_sim.FastLossWeights(bullet=True)
    # t1 = np.random.rand(100,4)
    # t2 = np.random.rand(100,4)
    # tmp1 = np.linalg.norm(t1, axis=-1)
    # tmp2 =  np.linalg.norm(t2, axis=-1)
    #
    # for i in range(4):
    #     t1[:,i] /= tmp1
    #     t2[:,i] /= tmp2
    #
    # start = timer()
    # e1 = w.CalcQuatLoss(t1, t2)
    # stop = timer()
    # print(stop-start)
    #
    # print()
    #
    # start = timer()
    # e2 = w.FastQuatLoss(t1, t2)
    # stop = timer()
    # print(stop-start)