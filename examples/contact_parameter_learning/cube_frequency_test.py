import cube_sim
import drake_cube_sim
import numpy as np
import matplotlib.pyplot as plt

test_state = np.array([ 0.18629883,  0.02622872,  0.483257, -0.52503014,  0.39360754,
       -0.29753734, -0.67794127,  0.01438053,  1.29095332, -0.21252927,
        1.46313532, -4.85439428,  1.86961928])

mse_loss = cube_sim.LossWeights()
position_loss = cube_sim.LossWeights(vel=np.zeros((3,)), omega=np.zeros((3,)), quat=cube_sim.BLOCK_HALF_WIDTH)

w_n = np.sqrt(drake_cube_sim.default_drake_contact_params['stiffness'] / cube_sim.CUBE_MASS)
print(f'w_n: {w_n}')

dts = np.logspace(np.log(0.5/w_n), np.log(2/w_n), 50).tolist()

continuous_contact_sim = drake_cube_sim.DrakeCubeSim(drake_sim_dt=0)
continuous_contact_sim.init_sim(drake_cube_sim.default_drake_contact_params)

# get 1 second trajectory with conitnuous time steps
continuous_timestep_traj = continuous_contact_sim.get_sim_traj_initial_state(test_state, int(cube_sim.CUBE_DATA_HZ), cube_sim.CUBE_DATA_DT)

mse_losses = []
position_losses = []
dts = sorted(dts)
for dt in sorted(dts):
    print(f'simulating with dt = {dt}')
    sim = drake_cube_sim.DrakeCubeSim(drake_sim_dt=dt)
    sim.init_sim(drake_cube_sim.default_drake_contact_params)
    sim_traj = sim.get_sim_traj_initial_state(test_state, int(cube_sim.CUBE_DATA_HZ), cube_sim.CUBE_DATA_DT)
    mse_losses.append(mse_loss.CalculateLoss(continuous_timestep_traj, sim_traj))
    position_losses.append(position_loss.CalculateLoss(continuous_timestep_traj, sim_traj))

plt.plot(dts, mse_losses)
plt.plot(dts, position_losses)
plt.xscale('log')
plt.legend(['mse loss', 'position loss'])
plt.show()