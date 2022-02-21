from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, CUBE_DATA_DT, load_cube_toss
import numpy as np
import os
import pybullet as p

cube_urdf_path = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/urdf/cube.urdf')
plane_urdf_path = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/urdf/plane.urdf')

default_bullet_contact_params = {"stiffness": 550,
                                 "damping": 19.0,
                                 "mu_tangent": 0.39}


class BulletCubeSim(CubeSim):

    def __init__(self, visualize=False, substeps=1):
        self.visualize = visualize
        if (self.visualize):
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        self.substeps = substeps

    def init_sim(self, params):
        p.resetSimulation(physicsClientId=self.client_id)
        p.setGravity(0,0,-9.81, physicsClientId=self.client_id)
        self.cube_id = p.loadURDF(cube_urdf_path, physicsClientId=self.client_id)
        self.plane_id = p.loadURDF(plane_urdf_path, useFixedBase=True,
            physicsClientId=self.client_id)
        p.changeDynamics(self.plane_id, -1, 
                            lateralFriction=params['mu_tangent'], 
                            contactStiffness=params['stiffness'],
                            contactDamping=params['damping'], 
                            physicsClientId=self.client_id)

        p.setTimeStep(CUBE_DATA_DT / self.substeps, physicsClientId=self.client_id)
        
    def set_initial_condition(self, state):
        initial_state = self.reexpress_state_local_to_global_omega(state)

        q = initial_state[CUBE_DATA_QUATERNION_SLICE]
        q_conv = np.array([q[1], q[2], q[3], q[0]]).ravel()

        p.resetBasePositionAndOrientation(self.cube_id, 
            posObj=initial_state[CUBE_DATA_POSITION_SLICE], 
            ornObj=q_conv, physicsClientId=self.client_id)
        p.resetBaseVelocity(self.cube_id, 
                            linearVelocity=initial_state[CUBE_DATA_VELOCITY_SLICE],
                            angularVelocity=initial_state[CUBE_DATA_OMEGA_SLICE], 
                            physicsClientId=self.client_id)

    def sim_step(self, dt):
        data_arr = np.zeros((1,13))
        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.cube_id, 
            physicsClientId=self.client_id)
        cube_vel, cube_omega = p.getBaseVelocity(self.cube_id, 
            physicsClientId=self.client_id)

        q_conv = np.array([cube_quat[3], cube_quat[0], cube_quat[1], cube_quat[2]])

        data_arr[0, CUBE_DATA_POSITION_SLICE] = cube_pos
        data_arr[0, CUBE_DATA_QUATERNION_SLICE] = q_conv
        data_arr[0, CUBE_DATA_VELOCITY_SLICE] = cube_vel
        data_arr[0, CUBE_DATA_OMEGA_SLICE] = cube_omega

        for i in range(self.substeps):
            p.stepSimulation(physicsClientId=self.client_id)
        data_arr[0] = self.reexpress_state_global_to_local_omega(data_arr)
        return data_arr

    def __del__(self):
        try:
            p.disconnect(physicsClientId=self.client_id)
        except:
            pass