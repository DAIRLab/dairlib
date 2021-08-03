import numpy as np
import mujoco_cube_sim
import drake_cube_sim
import cube_sim

test_state = np.array([ 0.18629883,  0.02622872,  1.89283257, -0.52503014,  0.39360754,
       -0.29753734, -0.67794127,  0.01438053,  1.29095332, -0.21252927,
        1.46313532, -4.85439428,  9.86961928]).reshape(-1,1)

def test_drake():
    drake_sim = drake_cube_sim.DrakeCubeSim(visualize=True)
    contact_params = drake_cube_sim.default_drake_contact_params
    drake_sim.init_sim(contact_params)
    state_traj = drake_sim.get_sim_traj_initial_state(test_state, 2000, cube_sim.CUBE_DATA_DT)
    return state_traj

def test_mujoco():
    mujoco_sim = mujoco_cube_sim.MujocoCubeSim(visualize=True)
    contact_params = mujoco_cube_sim.default_mujoco_contact_params
    mujoco_sim.init_sim(contact_params)
    state_traj = mujoco_sim.get_sim_traj_initial_state(test_state, 300, cube_sim.CUBE_DATA_DT)
    return state_traj

if (__name__ == "__main__"):
    data = test_mujoco()
    print(data)