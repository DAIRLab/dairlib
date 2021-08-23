import cube_sim
import drake_cube_sim
import numpy as np

test_state = np.array([ 0.18629883,  0.02622872,  0.483257, -0.52503014,  0.39360754,
       -0.29753734, -0.67794127,  0.01438053,  1.29095332, -0.21252927,
        1.46313532, -4.85439428,  1.86961928])

mse_loss = cube_sim.LossWeights()
position_loss = cube_sim.LossWeights(vel=np.zeros((3,)), omega=np.zeros((3,)), quat=cube_sim.BLOCK_HALF_WIDTH)

w_n = np.sqrt(drake_cube_sim.default_drake_contact_params['stiffness'] / cube_sim.CUBE_MASS)
print(w_n)