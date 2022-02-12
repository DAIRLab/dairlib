import numpy as np

from drake_to_mujoco_converter import DrakeToMujocoConverter

if __name__ == '__main__':
    converter = DrakeToMujocoConverter()

    qpos_init_mujoco = np.array([0, 0, 1.01, 1, 0, 0, 0,
                                 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
                                 -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
                                 -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
                                 -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968])
    qpos_init_drake = np.array(
        [1, 0, 0, 0, 0, 0, 1.01, 0.0045, 0, 0.4973, -1.1997, 0, 1.4267, 0, -1.5968, 0.0045, 0, 0.4973, -1.1997, 0,
         1.4267, 0, -1.5968])

    qvel_init_mujoco = np.zeros(32)
    qvel_init_drake = np.zeros(22)
    x_init_drake = np.hstack((qpos_init_drake, qvel_init_drake))
    # converter.visualize_state_upper(x_init_drake)
    # converter.visualize_state_lower(x_init_drake)
    converter.visualize_entire_leg(x_init_drake)
