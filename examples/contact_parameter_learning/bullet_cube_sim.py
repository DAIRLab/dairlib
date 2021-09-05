from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, CUBE_DATA_DT, load_cube_toss
import numpy as np
import os
import pybullet as p

cube_urdf_path = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/urdf/cube.urdf')
plane_urdf_path =  os.path.join(os.getcwd(), 'examples/contact_parameter_learning/urdf/plane.urdf')

default_bullet_contact_params = {"stiffness" : 2000, 
                                 "damping" : 36.02, 
                                 "restitution": 0.125,
                                 "mu_tangent" : 0.18, 
                                 "mu_torsion" : 0.005, 
                                 "mu_rolling" : 0.0001}


class BulletCubeSim(CubeSim):

    def __init__(self, visualize=False, substeps=1):
        self.visualize = visualize
        if (self.visualize):
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.substeps = substeps

    def init_sim(self, params):
        p.setGravity(0,0,-9.81)
        self.cube_id = p.loadURDF(cube_urdf_path)
        self.plane_id = p.loadURDF(plane_urdf_path, useFixedBase=True)
        p.changeDynamics(self.plane_id, -1, 
                            lateralFriction=params['mu_tangent'], 
                            spinningFriction=params['mu_torsion'],
                            rollingFriction=params['mu_rolling'],
                            restitution=params['restitution'], 
                            contactStiffness=params['stiffness'],
                            contactDamping=params['damping'])

        p.setTimeStep(CUBE_DATA_DT / self.substeps)
        
    def set_initial_condition(self, state):

        initial_state = self.reexpress_state_local_to_global_omega(state)

        q = initial_state[CUBE_DATA_QUATERNION_SLICE]
        q_conv = np.array([q[1], q[2], q[3], q[0]]).ravel()

        p.resetBasePositionAndOrientation(self.cube_id, posObj=initial_state[CUBE_DATA_POSITION_SLICE], ornObj=q_conv)
        p.resetBaseVelocity(self.cube_id, 
                            linearVelocity=initial_state[CUBE_DATA_VELOCITY_SLICE],
                            angularVelocity=initial_state[CUBE_DATA_OMEGA_SLICE])

    def sim_step(self, dt):
        data_arr = np.zeros((1,13))
        cube_pos, cube_quat = p.getBasePositionAndOrientation(self.cube_id)
        cube_vel, cube_omega = p.getBaseVelocity(self.cube_id)

        q_conv = np.array([cube_quat[3], cube_quat[0], cube_quat[1], cube_quat[2]])

        data_arr[0, CUBE_DATA_POSITION_SLICE] = cube_pos
        data_arr[0, CUBE_DATA_QUATERNION_SLICE] = q_conv
        data_arr[0, CUBE_DATA_VELOCITY_SLICE] = cube_vel
        data_arr[0, CUBE_DATA_OMEGA_SLICE] = cube_omega

        for i in range(self.substeps):
            p.stepSimulation()
        data_arr[0] = self.reexpress_state_global_to_local_omega(data_arr)
        return data_arr

    def __del__(self):
        p.disconnect()
