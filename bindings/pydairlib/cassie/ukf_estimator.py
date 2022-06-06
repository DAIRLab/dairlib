import sys
import numpy as np
import matplotlib.pyplot as plt
import pydairlib
import scipy.linalg
import scipy.io
from scipy.spatial.transform import Rotation
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import kinematic
from pydrake.multibody.tree import JacobianWrtVariable

spring_stiffness = np.zeros(23,)

class UKF:
    def __init__(self,init_mean, init_cov, R, Q):
        """
        Initialize the states and covariance matrices.

        The initial states are in the form of [q, v, lambda_c, C],
        where the q[:4] are orientation of pelvis expressed in quaternion form.

        R is the dynamics noise in order of [motor noise, lambda_c noise, lambda_h noise]
        """
        self.mean = init_mean
        self.cov = init_cov
        # Dynamics uncertainty
        self.R = R
        # Observation uncertainty
        self.Q = Q

        # Dim of q 
        self.nq = 23
        # Dim of v, the C should also in the dimension
        self.nv = 22
        # Dim of lambda_c 
        self.nlc = 12
        # Dim of damping ratio
        self.nd = 16
        # Total dim
        self.n = self.nq + self.nv + self.nlc + self.nd

        # Number of points for the foot to contacts for the ground
        self.num_contacts = 4

        # Initialize for drake
        self.builder = DiagramBuilder()
        self.drake_sim_dt = 5e-5
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        addCassieMultibody(self.plant, self.scene_graph, True, 
                            "examples/Cassie/urdf/cassie_v2.urdf", True, True)
        self.plant.Finalize()
        self.world = self.plant.world_frame()
        self.context = self.plant.CreateDefaultContext()
        
        left_rod_on_thigh = LeftRodOnThigh(self.plant)
        left_rod_on_heel = LeftRodOnHeel(self.plant)
        self.left_loop = kinematic.DistanceEvaluator(
                        self.plant, left_rod_on_heel[0], left_rod_on_heel[1], left_rod_on_thigh[0],
                        left_rod_on_thigh[1], 0.5012)
        right_rod_on_thigh = RightRodOnThigh(self.plant)
        right_rod_on_heel = RightRodOnHeel(self.plant)
        self.right_loop = kinematic.DistanceEvaluator(
                        self.plant, right_rod_on_heel[0], right_rod_on_heel[1], right_rod_on_thigh[0],
                        right_rod_on_thigh[1], 0.5012)
                    
    def get_sigma_points(self):
        """
        Get 2n+1 sigma points and dynamics noise based on covariance.
        
        return:
            a 2n+1 X n matrix where each represent a sigma point
            2n+1 motor noises
            2n+1 lambda_c_noises
            2n+1 lambda_h_noises
        """
        
        # stack self.cov and noise cov to a whole_cov term

        whole_cov = np.zeros((self.cov.shape[0]+self.R.shape[0],self.cov.shape[0]+self.R.shape[0]))
        whole_cov[0:self.cov.shape[0],0:self.cov.shape[0]] = self.cov
        whole_cov[-self.R.shape[0]:,-self.R.shape[0]:] = self.R

        # Comupte the sqrt of covariance by cholesky
        L = np.linalg.cholesky(whole_cov)
        # diviation for all states
        div = (self.cov.shape[0] + self.R.shape[0])**0.5 * L[:,0:self.cov.shape[0]]
        # motor noise
        motor_noise = (self.cov.shape[0] + self.R.shape[0])**0.5 * \
                        np.vstack((np.zeros(10),
                                whole_cov[:,self.cov.shape[0]:self.cov.shape[0]+10], 
                                -whole_cov[:,self.cov.shape[0]:self.cov.shape[0]+10]))
        # lambda_c random walking
        lambda_c_noise = (self.cov.shape[0] + self.R.shape[0])**0.5 * \
                        np.vstack((np.zeros(self.nlc), 
                        whole_cov[:,self.cov.shape[0]+10:self.cov.shape[0]+10+self.nlc], 
                        -whole_cov[:,self.cov.shape[0]+10:self.cov.shape[0]+10+self.nlc]))

        # Deal with part with quaternion
        # First get deviation as rotation vector
        rot_vectors_div = np.vstack((div[:,:3],-div[:,:3]))
        # Get orientation div into Rotation class
        ori_div = Rotation.from_rotvec(rot_vectors_div)
        # Get the mean of orientation into Rotation class
        ori_mean = Rotation.from_quat(self.mean[:4][[1,2,3,0]]) # The quaternion in drake is represent as scalar first, while the library use scalar last
        # Get the sigma points for quaternion
        ori_sigma_points = (ori_mean * ori_div).as_quat()[:, [3,0,1,2]] # From scalar last to scalar first

        # Deal with other states which are all in vector space
        other_div = np.vstack((div[:,3:], -div[:,3:]))
        other_mean = self.mean[4:]
        other_sigma_points = other_mean + other_div

        # Assemble different parts together
        sigma_points = np.hstack((ori_sigma_points, other_sigma_points))

        # Also include the self.mean into sigma point
        sigma_points = np.vstack((self.mean, sigma_points))

        return sigma_points, motor_noise, lambda_c_noise

    def dynamics_updates(self, sigma_points, u, motor_noise, lambda_c_noise, dt):
        """
        Get the nominal sigma points after dynamics,
        sigma points are in form of [q, v, lambda_c, C]
        paras:
            sigma_points: 2(n+self.u.size+self.nlc)+1 X n, where each row represent a sigma point
            u: (10,) motor toque
            motor_noise: 2(n+self.u.size+self.nlc)+1 X 10, motor noise to be added in
            lambda_c_noise: 2(n+self.u.size+self.nlc)+1 X nlc, the incremental for lambda_c
            dt: scalar, the time diff since last update
        return:
            sigma_points: 2(n+self.u.size+self.nlc)+1 X n sigma_points after dynamics  
        """
        
        q = sigma_points[:,:self.nq]
        v = sigma_points[:,self.nq:self.nq+self.nv]
        lambda_c = sigma_points[:,self.nq+self.nv: self.nq + self.nv + self.nlc]
        C = sigma_points[:,-self.nd:]

        """ 
            The dynamics equation is given by x_{k+1} = x_k + dt * dotx_k.
            The dotx_k is given by [q_dot, v_dot, 0, 0]
            The q_dot is v with additional process for quaternion,
            The v_dot can be compute by other formulation 
            The dot_lambda_c and is hard to model, we alternative model it by some high covariance
            We assume C to be constant
        """

        # Cal the q_next
        # Compute the delta in orientation first
        delta_q_ori = Rotation.from_rotvec(v[:,:3] * dt)
        # Compute the quaternion after dynamics
        q_next_quat = (Rotation.from_quat(q[:,[1,2,3,0]]) * delta_q_ori).as_quat()[:,[3,0,1,2]] # switch the quaternion scalar between last and first 
        # Compute q_next for other position elements
        q_next_other = q[:,4:] + v[:,3:] * dt
        # Assemble the q_next
        q_next = np.hstack((q_next_quat, q_next_other))

        # Cal the v_next
        # Cal vdot
        v_dot = self.cal_vdot(q, v, lambda_c, C, u+motor_noise)
        # Cal v_next
        v_next = v + v_dot * dt

        # Cal lambda_c next
        lambda_c_next = lambda_c + lambda_c_noise * dt

        # Assemble all the stuff
        sigma_points_next = np.hstack((q_next, v_next, lambda_c_next, C))

        return sigma_points_next

    def cal_vdot(self, q, v, lambda_c, C, u):
        """
        The dynamics is given by:
        M @ v_dot + bias == J_c^T @ lambda_c + J_h^T @ lambda_h + B @ u + dig(C) @ v
        paras:
            q: 2(n+self.u.size+self.nlc)+1 X 23, each row represents a sigma point of the position vector
            v: 2(n+self.u.size+self.nlc)+1 X 22, each row represents a sigma point of the velocity vector
            lambda_c: 2(n+self.u.size+self.nlc)+1 X 12, each row represents a sigma point of the ground force vector
            C: 2(n+self.u.size+self.nlc)+1 X 22, each row presents a sigma point of the damping ratio parameter vector 
            u: 2(n+self.u.size+self.nlc)+1 X 10 the input torque
        returns:
            vdot: 2(n+self.u.size+self.nlc)+1 X 22, where each row represents a sigma point of acceleration
        """

        vdot = np.zeros((q.shape[0], 22))

        q_v = np.hstack((q, v))
        for i in range(q.shape[0]):

            if q.ndim == 1:
                self.plant.SetPositionsAndVelocities(self.context, q_v)
            else:
                self.plant.SetPositionsAndVelocities(self.context, q_v[i,:])

            # get M matrix
            M = self.plant.CalcMassMatrix(self.context)
            M_inv = scipy.linalg.pinv(M)

            # get bias term
            bias = self.plant.CalcBiasTerm(self.context)

            # get the J_c
            J_c = np.zeros((3 * self.num_contacts, self.nv))
            for j in range(self.num_contacts):
                # get the contact point and foot frame
                if j == 0:
                    point_on_foot, foot_frame = LeftToeFront(self.plant) # left toe front
                elif j == 1:
                    point_on_foot, foot_frame = LeftToeRear(self.plant) # left toe back
                elif j == 2:
                    point_on_foot, foot_frame =  RightToeFront(self.plant) # right toe front
                elif j == 3:
                    point_on_foot, foot_frame = RightToeRear(self.plant) # right toe back
                else:
                    raise ValueError("num_of_contacts should not be more than 4")

                # calculate the J_c for one contact point
                J_c[3*j:3*(j+1), :] = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)

            # get the J_h
            J_h = np.zeros((2, self.nv))
            J_h[0, :] = self.left_loop.EvalFullJacobian(self.context)
            J_h[1, :] = self.right_loop.EvalFullJacobian(self.context)

            # get the B matrix 
            B = self.plant.MakeActuationMatrix()

            # get the gravity_force
            gravity_force = self.plant.CalcGravityGeneralizedForces(self.context)

            # Calculate the J_h_dot_times_v for constraint force later 
            J_h_dot_times_v = np.zeros(2,)
            J_h_dot_times_v[0] = self.left_loop.EvalFullJacobianDotTimesV(self.context)
            J_h_dot_times_v[1] = self.right_loop.EvalFullJacobianDotTimesV(self.context)

            # compute the v_dot
            if q.ndim == 1:
                # calculate the constraint force 
                lambda_h = -scipy.linalg.pinv(J_h @ M_inv @ J_h.T) @ (J_h @ M_inv @ (-bias + gravity_force + J_c.T @ lambda_c + B @ u -(spring_stiffness * q)[1:] - np.hstack((np.zeros(self.nv - self.nd), C))* v) - J_h_dot_times_v)
                # calculate the vdot 
                vdot = M_inv @ (-bias + gravity_force + J_c.T @ lambda_c + J_h.T @ lambda_h + B @ u -(spring_stiffness * q)[1:] - np.hstack((np.zeros(self.nv - self.nd), C)) * v)
            else:
                lambda_h = -scipy.linalg.pinv(J_h @ M_inv @ J_h.T) @ (J_h @ M_inv @ (-bias + gravity_force + J_c.T @ lambda_c[i,:] + B @ u[i,:] -(spring_stiffness * q[i,:])[1:] - np.hstack((np.zeros(self.nv - self.nd), C[i,:]))*v[i,:]) - J_h_dot_times_v)
                vdot[i,:] = M_inv @ (-bias + gravity_force + J_c.T @ lambda_c[i,:] + J_h.T @ lambda_h + B @ u[i,:] -(spring_stiffness * q[i,:])[1:] - np.hstack((np.zeros(self.nv-self.nd), C[i,:])) * v[i,:])

        return vdot

    def cal_diviation_vectors_from_sigma_points(self, sigma_points_quat, nominal_quat_mean):
        """
        Given sigma_points_quat and nominal mean calculate the divation of each sigma points from the nominal mean
        
        paras:
            sigma_point_quat: 2n+1 X 4, quaternion part of each sigma point
            nominal_quat_mean: (4,), the nominal mean of current sigma_points

        return:
            error vector in (2n+1, 3), the deviation of each sigma_point from the nominal mean in rotation vector form
        """

        error_rot_vecs = (Rotation.from_quat(nominal_quat_mean[[1,2,3,0]]).inv() * Rotation.from_quat(sigma_points_quat[:,[1,2,3,0]])).as_rotvec() # From scalar first to scalar last

        return error_rot_vecs

    def cal_mean_of_sigma_points(self, sigma_points, initial_guess_of_quaternion=Rotation.identity().as_quat()[[3,0,1,2]], threshold = 1e-5):
        """
        Calculate the mean of sigma_points.
        
        Calculate the mean of quaternion is tricky.
        I solve it by start with some nominal mean and calculate the error vector based on deviation.
        Then, correct the nominal mean by the error vector until meet some threshold.

        paras:
            sigma_points: 2n+1 X n, where each row represent a sigma point
            initial_guess_of_quaternion: initial guess of the mean of quaternion
            threshold: the stop criterion for error vector

        returns:
            (2n+1,) vector represents the mean of sigma points 
        """

        # Calculation for the mean of quaternion
        nominal_quat_mean = initial_guess_of_quaternion
        sigma_points_quat = sigma_points[:,:4]
        diviation_vectors = self.cal_diviation_vectors_from_sigma_points(sigma_points_quat, nominal_quat_mean)
        error_rot_vector = np.mean(diviation_vectors, axis=0)
        while np.linalg.norm(error_rot_vector) > threshold:
            nominal_quat_mean = (Rotation.from_quat(nominal_quat_mean[[1,2,3,0]]) * Rotation.from_rotvec(error_rot_vector)).as_quat()[[3,0,1,2]]
            diviation_vectors = self.cal_diviation_vectors_from_sigma_points(sigma_points_quat, nominal_quat_mean)
            error_rot_vector = np.mean(diviation_vectors, axis=0)
        
        # Calculate the mean of other part
        other_mean = np.mean(sigma_points[:,4:], axis=0)

        # Assemble mean
        mean = np.hstack((nominal_quat_mean, other_mean))

        return mean

    def cal_spread_of_sigma_points(self, sigma_points):
        """
        Calculate the spread of sigma_points. Basically sigma_points - mean, with additional process for quaternion
        paras:
            sigma_points: 2n+1 X n, where each row represents a sigma point
        returns:
            2n+1 X n-1 : spread of sigma points by subtracting it's mean. The orientation is expressed in rot_vec rather than quaternion
        """
        # Compute the mean of sigma points
        mean = self.cal_mean_of_sigma_points(sigma_points, initial_guess_of_quaternion=self.mean[0:4]) # Use the self.mean quaternion as a initial guess of quaternion

        # Compute the spread of quaternion for sigma points
        mean_quat = mean[:4]
        rot_vec_spread = self.cal_diviation_vectors_from_sigma_points(sigma_points[:,:4], mean_quat)

        # Compute other parts
        other_parts_spread = sigma_points[:,4:] - mean[4:]

        # Assemble them together
        spread_of_sigma_points = np.hstack((rot_vec_spread, other_parts_spread))

        return spread_of_sigma_points 

    def cal_cov_of_sigma_points(self, sigma_points):
        """
        Cacluate the cov of sigma points, the orientation is expressed as rot_vec rather than quaternion
        """
        spread_of_sigma_points = self.cal_spread_of_sigma_points(sigma_points)

        cov = np.cov(spread_of_sigma_points, rowvar=False, bias=True)

        return cov

    def cal_expected_obs_of_sigma_points(self, sigma_points, u, motor_noise):
        """
        Calculate the expected observation given sigma_points.
        
        The sensors are imu at main body and encoder for each joints
        The observation thus goes [accelerometer reading of main body, gyro reading of body, encoder of each joint ]
        """

        q = sigma_points[:,:self.nq]
        v = sigma_points[:,self.nq:self.nq+self.nv]
        lambda_c = sigma_points[:,self.nq+self.nv: self.nq + self.nv + self.nlc]
        C = sigma_points[:,-self.nd:]
        
        # Deal with IMU part
        """
        The accelerometer read the acceleration of main body plus the gravity expressed in body fixed frame.
        
        The accelerometer reading is given by R^T @ (vdot_in_world_frame + g_in_world_frame)
        """
        # Calculate the accelerometer readings
        g_in_world_frame = [0, 0, -9.8]
        vdot = self.cal_vdot(q, v, lambda_c, C, u+motor_noise)
        acc_of_body = vdot[:, 3:6]
        # print("acc_of_body", acc_of_body)
        # print("cov of acc", np.cov(acc_of_body, rowvar=False, bias=True))
        rot_matrice = Rotation.from_quat(q[:,:4][:,[1,2,3,0]]).as_matrix()
        total_acc_in_world_frame = g_in_world_frame + acc_of_body

        accelerometer_readings = (np.moveaxis(rot_matrice, 1, -1) @ total_acc_in_world_frame[:,:,None]).squeeze() # moveaxis change the R to R.T
        # print("accelerometer_readings", accelerometer_readings)
        # print("accelerometer_readings cov", np.cov(accelerometer_readings, rowvar=False, bias=True))
        # import pdb; pdb.set_trace()
        # Calculate the gyro readings
        gyro_readings = v[:,:3]

        # Deal with encoder
        encoder_reading = q[:,7:]

        # Assemble all reading together
        exp_obs = np.hstack((accelerometer_readings, gyro_readings, encoder_reading))

        return exp_obs

    def cal_cov_xy(self, states, obs):
        """
        Calculate the covariance between the states and expected observations
        paras:
            states: 2n+1 X n matrix, where each row represents individual state. The first 4 entries represent the orientation in quaternion\
            obs: (22,) vector of observation
        """
        spread_of_states = self.cal_spread_of_sigma_points(states)
        spread_of_obs = obs - np.mean(obs, axis=0)

        cov = (spread_of_states.T @ spread_of_obs)/spread_of_states.shape[0]

        return cov
    
    def update(self, u, obs, dt):
        """
        Get one time-step updates after get inputs and observation
        paras:
            u: (10,) vector represent the input torque
            obs: (22,) vector represent the sensor observation. The observation should in form of [body imu, encoding reading of q except body orientation and position]
            dt: scalar for diff in time
        """
        
        print("cov of q", np.diag(self.cov)[:self.nv])
        print("cov of v", np.diag(self.cov)[self.nv:self.nv*2])
        print("max uncertainty of q", np.max(np.abs(np.diag(self.cov)[:self.nv])))
        print("max uncertainty of v", np.max(np.abs(np.diag(self.cov)[self.nv:self.nv*2])))
        # print("cov of lambda_C", np.diag(self.cov)[self.nv*2:self.nv*2+self.nlc])
        # print("cov of damping", np.diag(self.cov)[-self.nd:])

        # Sample the sigma points
        sigma_points, motor_noise, lambda_c_noise = self.get_sigma_points()
        # expected_observations = self.cal_expected_obs_of_sigma_points(sigma_points, u, motor_noise)
        # expected_observations_mean = np.mean(expected_observations, axis=0)
        # expected_observations_cov = np.cov(expected_observations, rowvar=False, bias=True)

        # print("cov of observation", np.diag(expected_observations_cov))

        # Dynamics process for sigma points
        sigma_points = self.dynamics_updates(sigma_points, u, motor_noise, lambda_c_noise, dt)
        # Update the mean and covariance after dynamics
        self.mean = self.cal_mean_of_sigma_points(sigma_points, self.mean[:4])
        self.cov = self.cal_cov_of_sigma_points(sigma_points)

        # Cal expected observation of sigma_points
        expected_observations = self.cal_expected_obs_of_sigma_points(sigma_points, u, motor_noise)
        expected_observations_mean = np.mean(expected_observations, axis=0)
        expected_observations_cov = np.cov(expected_observations, rowvar=False, bias=True)

        # Cal the kalman gains
        # Cal the cov we needed
        cov_yy = expected_observations_cov + self.Q
        cov_xy = self.cal_cov_xy(sigma_points, expected_observations)
        # Cal Kalman gain
        kalman_gain = cov_xy @ scipy.linalg.pinv(cov_yy)

        import pdb; pdb.set_trace()
    
        # Incorporate the observation
        # Compute the correction term for dynamics process
        innovation = obs - expected_observations_mean
        correction_term = kalman_gain @ innovation # For the correction term the orientation is represented as rotation vector
        # Update the mean
        # Correct the quaternion part
        self.mean[:4] = Rotation.from_rotvec(Rotation.from_quat(self.mean[:4][[1,2,3,0]]).as_rotvec() + correction_term[:3]).as_quat()[[3,0,1,2]] # Convert the quatertion convention between libarary and drake
        # Correct the other part
        self.mean[4:] = self.mean[4:] + correction_term[3:]
        # Update the covariance
        self.cov = self.cov - kalman_gain @ cov_yy @ kalman_gain.T

        return self.mean

def interploation(t,t1,t2,v1,v2):
    ratio = (t - t1)/(t2-t1)
    v = v1 + ratio * (v2 -v1)
    return v

def processing_raw_data(raw_data, R, Q):
    """
    Process the data given raw_data and noise. 
    Time was synchronized to t_x by interpolation to other value that do not use t_x.
    return a dict contained processed data
    addtional infos for returned data:
    keys in dict:
        u: the ground truth input to the robot plus fake noise.
        obs: the observation [imu acc, gyro, encoder reading] plus fake noise, 
            the imu acc is obtained by acc_gt which is taken by finite difference of v
        lambda_c_gt: ground truth contact force by interpolation, order should be [left front, left rear, right front, right rear] TODO make sure this
        lambda_h_guess: the lambda_h solved by osc optimization, should be a very rough value of what it should be
    """
    # Initialization
    start_time = 6.5
    end_time = 10

    # processing robot output
    robot_output = raw_data['robot_output']
    t_robot_output = robot_output["t_x"][0][0][0]
    start_index = np.argwhere(t_robot_output > start_time)[0][0]
    end_index = np.argwhere(t_robot_output < end_time)[-1][0]
    t_robot_output = t_robot_output[start_index:end_index]
    q = robot_output['q'][0][0][start_index:end_index,:]
    v = robot_output['v'][0][0][start_index:end_index,:]
    u = robot_output['u'][0][0][start_index:end_index,:]
    # obtained acc by finite diff
    acc = (robot_output['v'][0][0][start_index+1:end_index+1,:] - robot_output['v'][0][0][start_index-1:end_index-1,:])\
        /(robot_output["t_x"][0][0][0][start_index+1:end_index+1] - robot_output["t_x"][0][0][0][start_index-1:end_index-1])[:,None]
    
    # processing contact force
    contact_output = raw_data['contact_output']
    t_contact = contact_output['t_lambda'][0][0][0]
    start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
    end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
    t_contact = t_contact[start_index: end_index]
    contact_force = contact_output['lambda_c'][0][0]
    left_toe_front = contact_force[0,start_index:end_index,:]
    left_toe_rear = contact_force[1,start_index:end_index,:]
    right_toe_front = contact_force[2,start_index:end_index,:]
    right_toe_rear = contact_force[3,start_index:end_index,:]
    contact_force_processed = []
    pointer = 0
    for t in t_robot_output:
        while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
            pointer+=1
        left_toe_front_interpolated = interploation(t, t_contact[pointer], t_contact[pointer+1], 
                                                    left_toe_front[pointer,:], left_toe_front[pointer+1,:])
        left_toe_rear_interpolated = interploation(t, t_contact[pointer], t_contact[pointer+1],
                                                    left_toe_rear[pointer,:], left_toe_rear[pointer+1,:])
        right_toe_front_interpolated = interploation(t, t_contact[pointer], t_contact[pointer+1],
                                                    right_toe_front[pointer,:], right_toe_front[pointer+1,:])
        right_toe_rear_interpolated = interploation(t, t_contact[pointer], t_contact[pointer+1],
                                                    right_toe_rear[pointer,:], right_toe_rear[pointer+1,:])
        joined_forces = np.hstack((left_toe_front_interpolated, left_toe_rear_interpolated, right_toe_front_interpolated, right_toe_rear_interpolated))
        contact_force_processed.append(joined_forces)
    contact_force_processed = np.array(contact_force_processed)

    # processing osc output
    osc_output = raw_data['osc_output']
    t_osc = osc_output['t_osc'][0][0][0]
    start_index = np.argwhere(t_osc > start_time - 0.01)[0][0] # make sure constraint force period range cover the output states
    end_index = np.argwhere(t_osc < end_time + 0.01)[-1][0]
    t_osc = t_osc[start_index:end_index]
    lambda_h_sol = osc_output['lambda_h_sol'][0][0][start_index:end_index,0:2] # only the first two contribute to closure force
    lambda_h_guess = []
    pointer = 0
    for t in t_robot_output:
        while not (t_osc[pointer] <= t and t_osc[pointer+1] >=t):
            pointer+=1
        lambda_h_interpolation = interploation(t, t_osc[pointer], t_osc[pointer+1],
                                                lambda_h_sol[pointer,:], lambda_h_sol[pointer+1,:])
        lambda_h_guess.append(lambda_h_interpolation)
    lambda_h_guess = np.array(lambda_h_guess)

    # Get damping ratio
    damping_ratio = raw_data['damping_ratio'][0]

    # Get spring stiffness
    global spring_stiffness 
    spring_stiffness = raw_data['spring_stiffness'][0]

    # construct observation by adding noise to ground truth 
    accelerometer_gt = acc[:,3:6]
    gyro_gt = v[:,0:3]
    encoder_gt = q[:,7:]
    obs_gt = np.hstack((accelerometer_gt, gyro_gt, encoder_gt))
    n = obs_gt.shape[0]
    noise = np.random.multivariate_normal(np.zeros(Q.shape[0]),Q,n)
    obs_processed = obs_gt + noise

    # plt.plot(acc[:,3],label="x")
    # plt.plot(acc[:,4],label="y")
    # plt.plot(acc[:,5],label="z")
    # plt.ylim(-20,20)
    # plt.legend()
    # plt.show()

    # construct noisy input
    n = u.shape[0]
    noise = np.random.multivariate_normal(np.zeros(R.shape[0]),R,n)
    u_processed = u + noise

    processed_data = {
        't':t_robot_output,
        'u':u_processed,
        'obs':obs_processed,
        'lambda_c_gt':contact_force_processed,
        'lambda_h_guess':lambda_h_guess,
        'q_gt':q,
        'v_gt':v,
        'acc_gt':acc,
        'obs_gt':obs_gt,
        'u_gt':u,
        'damping_ratio':damping_ratio
    }

    return processed_data

def main():
    np.random.seed(0)
    data_path = "log/20220530_1.mat"
    raw_data = scipy.io.loadmat(data_path)
    
    # dynamics noise
    R_motor = np.eye(10) * 1
    # observation noise
    Q = np.eye(22) * 0.1
    # initial noise
    init_cov = np.eye(72) * 0.1

    # processing raw data
    print("Begin processing raw data for test")
    processed_data = processing_raw_data(raw_data, R_motor, Q)
    t = processed_data['t'];u = processed_data['u'];obs = processed_data['obs']
    lambda_c_gt = processed_data['lambda_c_gt'];lambda_h_guess = processed_data['lambda_h_guess'];q_gt = processed_data['q_gt']
    v_gt = processed_data['v_gt'];acc_gt = processed_data['acc_gt'];obs_gt = processed_data['obs_gt']
    u_gt = processed_data['u_gt'];damping_ratio = processed_data['damping_ratio']
    print("Finish processing data")

    # get initial value 
    init_mean = np.hstack((
                        q_gt[0,:],
                        v_gt[0,:],
                        lambda_c_gt[0,:],
                        damping_ratio[6:]
                        ))
    noise = np.random.multivariate_normal(np.zeros(init_mean.shape[0]-1), init_cov)
    # init_mean[4:] += noise[3:]
    # init_mean[:4] = (Rotation.from_quat(init_mean[:4][[1,2,3,0]]) * Rotation.from_rotvec(noise[0:3])).as_quat()[[3,0,1,2]]

    # initial robot plant
    builder = DiagramBuilder()
    drake_sim_dt = 5e-5
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, drake_sim_dt)
    addCassieMultibody(plant, scene_graph, True, 
                        "examples/Cassie/urdf/cassie_v2.urdf", True, True)
    plant.Finalize()
    vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
    vel_map_inverse = {value:key for (key, value) in vel_map.items()}
    pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
    pos_map_inverse = {value:key for (key, value) in pos_map.items()}

    # initial ukf
    R_lambda_c = np.eye(12) * 200
    R = np.zeros((22,22))
    R[0:10,0:10] = R_motor
    R[10:22,10:22] = R_lambda_c
    estimator = UKF(init_mean, init_cov, R, Q)

    # acc_est = []
    # acc_est_with_lambda_c_noise = []
    # interested_range = range(1000,3000)
    # for i in interested_range:
    #     noise = np.random.multivariate_normal(np.zeros(12), np.eye(12)*200)
    #     acc_est.append(estimator.cal_vdot(q_gt[i,:],v_gt[i,:],lambda_c_gt[i,:], damping_ratio[6:], u_gt[i,:]))
        # acc_est_with_lambda_c_noise.append(estimator.cal_vdot(q_gt[i,:],v_gt[i,:],lambda_c_gt[i,:] + noise, damping_ratio[6:], u_gt[i,:]))
    # acc_est = np.array(acc_est)
    # acc_est_with_lambda_c_noise = np.array(acc_est_with_lambda_c_noise)
    
    # for index in range(22):
    #     path = "bindings/pydairlib/cassie/ukf_experienments/dynamics_check/" + vel_map_inverse[index] + ".png" 
    #     plt.clf()
    #     plt.plot(t[interested_range], acc_est[:,index],'r',label="est")
    #     # plt.plot(t[interested_range], acc_est_with_lambda_c_noise[:, index], label="est_with_lambda_c_noise")
    #     plt.plot(t[interested_range], acc_gt[interested_range,index],'g', label="ground_truth")
    #     plt.xlabel("time(s)")
    #     plt.ylabel("derivative")
    #     plt.title(vel_map_inverse[index])
    #     plt.ylim(-500,500)
    #     plt.savefig(path)
    # import pdb; pdb.set_trace()

    est = [init_mean]

    # update the belief
    for i in range(1,t.shape[0]):
        states = estimator.update(u_gt[i],obs_gt[i],t[i]-t[i-1])
        est.append(states)

if __name__ == "__main__":
    main()