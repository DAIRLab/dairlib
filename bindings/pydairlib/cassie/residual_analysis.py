import sys
import tqdm
import numpy as np
import matplotlib.pyplot as plt
import pydairlib
import scipy.linalg
import scipy.io
import cvxpy as cp
from scipy.spatial.transform import Rotation
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import kinematic
from pydrake.multibody.tree import JacobianWrtVariable, MultibodyForces

class CassieSystem():
    def __init__(self):
        # Initialize for drake
        self.builder = DiagramBuilder()
        self.drake_sim_dt = 5e-5
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        addCassieMultibody(self.plant, self.scene_graph, True, 
                            "examples/Cassie/urdf/cassie_v2.urdf", True, True)
        self.plant.Finalize()
        self.world = self.plant.world_frame()
        self.base_frame = self.plant.GetBodyByName("pelvis").body_frame()
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
        # Initial other parameters
        self.K = np.zeros((22,23))
        self.C = np.zeros((22,22))

    def get_spring_stiffness(self, K):
        self.K = K
    
    def get_damping(self, C):
        self.C = C

    def calc_vdot(self, t, q, v, u, lambda_c_gt, lambda_c_position ,is_soft_constraints=False):
        """
            The unknown term are \dot v, contact force \lambda_c_active and constraints forces \lambda_h.

            M * \dot v + bias + Cv + Kq == Bu + gravity + J_c_active^T*\lambda_c + J_h^T*\lambda_h     (1)
            \dot J_h * v + J_h * \dot v == 0    (2)
            \dot J_c_active * v + J_c_active * \dot v == 0 (3)

            ==>

            [M, -J_c_active^T, -J_h^T      [ \dot v             [ Bu + gravity - bias - Cv -Kq
            J_h, 0, 0           *     \lambda_c     =      -\dot J_h * v
            J_c_active, 0, 0          \lambda_h            -\dot J_c_active * v 
            ]                       ]                     ]
        """
        # Set configuration
        self.plant.SetPositionsAndVelocities(self.context, np.hstack((q,v)))

        # Get mass matrix
        M = self.plant.CalcMassMatrix(self.context)

        # Get bias term
        bias = self.plant.CalcBiasTerm(self.context)

        # Get the B matrix
        B = self.plant.MakeActuationMatrix()

        # Get the gravity term    Set optional for solving soft constraints optimiazation problem for calc_vdot

        gravity = self.plant.CalcGravityGeneralizedForces(self.context)

        # Get the general damping and spring force 
        damping_and_spring_force = MultibodyForces(self.plant)
        self.plant.CalcForceElementsContribution(self.context, damping_and_spring_force)
        damping_and_spring_force = damping_and_spring_force.generalized_forces()

        # Get the J_h term
        J_h = np.zeros((2, 22))
        J_h[0, :] = self.left_loop.EvalFullJacobian(self.context)
        J_h[1, :] = self.right_loop.EvalFullJacobian(self.context)

        # Get the JdotTimesV term
        J_h_dot_times_v = np.zeros(2,)
        J_h_dot_times_v[0] = self.left_loop.EvalFullJacobianDotTimesV(self.context)
        J_h_dot_times_v[1] = self.right_loop.EvalFullJacobianDotTimesV(self.context)

        # Get the J_c_active term
        # For simplicity test, now directly give the lambda_c ground truth from simulation to determine if contact happen
        J_c_active, J_c_active_dot_v, num_contact_unknown, get_force_at_point_matrix = self.get_J_c_ative_and_J_c_active_dot_v(lambda_c_gt, lambda_c_position)
        z_direction_selection_matrix = np.zeros((4,12))
        z_direction_selection_matrix[0,2] = 1;  z_direction_selection_matrix[1,5] = 1
        z_direction_selection_matrix[2,8] = 1;  z_direction_selection_matrix[3,11] = 1

        # Construct Linear Equation Matrices
        # Solving the exact linear equations:
        if num_contact_unknown > 0:
            A = np.vstack((
                    np.hstack((M, -J_c_active.T, -J_h.T)),
                    np.hstack((J_h, np.zeros((2,num_contact_unknown)),np.zeros((2, 2)))),
                    np.hstack((J_c_active, np.zeros((num_contact_unknown, num_contact_unknown)), np.zeros((num_contact_unknown,2))))
            ))

            b = np.hstack((
                B @ u + gravity - bias + damping_and_spring_force,
                -J_h_dot_times_v,
                -J_c_active_dot_v
            ))
        else:
            A = np.vstack((
                np.hstack((M, -J_h.T)),
                np.hstack((J_h, np.zeros((2, 2)))),
                ))

            b = np.hstack((
                B @ u + gravity - bias + damping_and_spring_force,
                -J_h_dot_times_v,
            ))

        if not is_soft_constraints or num_contact_unknown == 0:
            # Solve minimum norm solution, since J_c may be not full row rank
            solution = A.T @ np.linalg.inv(A @ A.T) @ b
        else:
            solution = cp.Variable(22+num_contact_unknown+2)
            epislon = cp.Variable(22+num_contact_unknown+2)
            obj = cp.Minimize(cp.norm(epislon))
            constraints = [A @ solution == b + epislon]
            constraints.append(z_direction_selection_matrix @ get_force_at_point_matrix @ solution[22:22+num_contact_unknown] >= 0)
            prob = cp.Problem(obj, constraints)
            prob.solve()
            solution = solution.value

        v_dot = solution[:22]
        if num_contact_unknown > 0:
            lambda_c = get_force_at_point_matrix @ solution[22:22+num_contact_unknown]
        else:
            lambda_c = np.zeros(12,)
        lambda_h = solution[-2:]

        # if num_contact_unknown > 0:
        #     lambda_c_z_direction = z_direction_selection_matrix @ get_force_at_point_matrix @ solution[22:22+num_contact_unknown]
        #     if not np.all(lambda_c_z_direction >= 0):
        #         print("minmum norm solution:",lambda_c_z_direction[:,None])
        #         if scipy.linalg.null_space(A).shape[1] > 0:
        #             null_space_z_direction = z_direction_selection_matrix @ get_force_at_point_matrix @ scipy.linalg.null_space(A)[22:22+num_contact_unknown]
        #             print("null space:", null_space_z_direction)
        #             c = cp.Variable(null_space_z_direction.shape[1])
        #             obj = cp.Minimize(0)
        #             constraints = [lambda_c_z_direction + null_space_z_direction @ c >= 0]
        #             prob = cp.Problem(obj, constraints)
        #             prob.solve()
        #             if prob.status != cp.OPTIMAL:
        #                 import pdb; pdb.set_trace()
        #                 pass
        #         else:
        #             import pdb; pdb.set_trace()
        #             pass

        return v_dot, lambda_c, lambda_h

    def get_J_c_ative_and_J_c_active_dot_v(self, lambda_c_gt, lambda_c_position):
        J_c_active = None; J_c_active_dot_v = None; num_contact_unknown = 0; 
        left_front_index = None; left_rear_index = None;
        right_front_index = None; right_rear_index = None;
        if np.linalg.norm(lambda_c_gt[:3]) > 0:
            _, foot_frame = LeftToeFront(self.plant)
            point_on_foot = lambda_c_position[:3]
            J_c_left_front = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            J_c_left_front_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
            J_c_active = J_c_left_front
            J_c_active_dot_v = J_c_left_front_dot_v
            left_front_index = num_contact_unknown
            num_contact_unknown += 3
        if np.linalg.norm(lambda_c_gt[3:6]) > 0:
            _, foot_frame = LeftToeRear(self.plant)
            point_on_foot = lambda_c_position[3:6]
            J_c_left_rear = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            J_c_left_rear_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
            if J_c_active is None:
                J_c_active = J_c_left_rear
                J_c_active_dot_v = J_c_left_rear_dot_v
            else:
                J_c_active = np.vstack((J_c_active, J_c_left_rear))
                J_c_active_dot_v = np.hstack((J_c_active_dot_v, J_c_left_rear_dot_v))
            left_rear_index = num_contact_unknown
            num_contact_unknown += 3
        if np.linalg.norm(lambda_c_gt[6:9]) > 0:
            _, foot_frame = RightToeFront(self.plant)
            point_on_foot = lambda_c_position[6:9]
            J_c_right_front = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            J_c_right_front_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
            if J_c_active is None:
                J_c_active = J_c_right_front
                J_c_active_dot_v = J_c_right_front_dot_v
            else:
                J_c_active = np.vstack((J_c_active, J_c_right_front))
                J_c_active_dot_v = np.hstack((J_c_active_dot_v, J_c_right_front_dot_v))
            right_front_index = num_contact_unknown
            num_contact_unknown += 3
        if np.linalg.norm(lambda_c_gt[9:12]) > 0:
            _, foot_frame = RightToeRear(self.plant)
            point_on_foot = lambda_c_position[9:12]
            J_c_right_rear = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
            J_c_right_rear_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
            if J_c_active is None:
                J_c_active = J_c_right_rear
                J_c_active_dot_v = J_c_right_rear_dot_v
            else:
                J_c_active = np.vstack((J_c_active, J_c_right_rear))
                J_c_active_dot_v = np.hstack((J_c_active_dot_v, J_c_right_rear_dot_v))
            right_rear_index = num_contact_unknown
            num_contact_unknown += 3

        get_force_at_point_matrix = np.zeros((12, num_contact_unknown))
        if left_front_index is not None:
            get_force_at_point_matrix[0,left_front_index] = 1
            get_force_at_point_matrix[1,left_front_index+1] = 1
            get_force_at_point_matrix[2,left_front_index+2] = 1
        if left_rear_index is not None:
            get_force_at_point_matrix[3,left_rear_index] = 1
            get_force_at_point_matrix[4,left_rear_index+1] = 1
            get_force_at_point_matrix[5,left_rear_index+2] = 1
        if right_front_index is not None:
            get_force_at_point_matrix[6,right_front_index] = 1
            get_force_at_point_matrix[7,right_front_index+1] = 1
            get_force_at_point_matrix[8,right_front_index+2] = 1
        if right_rear_index is not None:
            get_force_at_point_matrix[9,right_rear_index] = 1
            get_force_at_point_matrix[10,right_rear_index+1] = 1
            get_force_at_point_matrix[11,right_rear_index+2] = 1

        return J_c_active, J_c_active_dot_v, num_contact_unknown, get_force_at_point_matrix      

    def make_position_map(self):

        pos_map = pydairlib.multibody.makeNameToPositionsMap(self.plant)
        pos_map_inverse = {value:key for (key, value) in pos_map.items()}
        
        return pos_map, pos_map_inverse
    
    def make_velocity_map(self):

        vel_map = pydairlib.multibody.makeNameToVelocitiesMap(self.plant)
        vel_map_inverse = {value:key for (key, value) in vel_map.items()}

        return vel_map, vel_map_inverse

class CaaiseSystemTest():
    def __init__(self, data_path, start_time, end_time):
        
        self.data_path = data_path
        self.start_time = start_time
        self.end_time = end_time
        
        self.cassie = CassieSystem()
        
        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.lambda_c_map, self.lambda_c_map_inverse = self.make_lambda_c_map()

    def make_lambda_c_map(self):
        
        lambda_c_map = {"left_front_x":0, "left_front_y":1, "left_front_z":2,
                            "left_rear_x":3, "left_rear_y":4, "left_rear_z":5,
                            "right_front_x":6, "right_front_y":7, "right_front_z":8,
                            "right_rear_x":9, "right_rear_y":10, "right_rear_z":11
                        }
        lambda_c_map_inverse = {value:key for (key,value) in lambda_c_map.items()}
        
        return lambda_c_map, lambda_c_map_inverse

    def simulation_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = self.process_sim_data(raw_data, self.start_time, self.end_time)
        t = process_data["t"]; q = process_data["q"]; v = process_data["v"]; v_dot_gt = process_data["v_dot"]; 
        u = process_data["u"]; lambda_c_gt = process_data["lambda_c"]; lambda_c_position = process_data["lambda_c_position"]
        damping_ratio = process_data["damping_ratio"]; spring_stiffness = process_data["spring_stiffness"]

        self.cassie.get_damping(damping_ratio)
        self.cassie.get_spring_stiffness(spring_stiffness)

        t_list = []
        v_dot_est_list = []
        lambda_c_est_list = []
        lambda_h_est_list = []
        v_dot_gt_list = []
        lambda_c_gt_list = []

        for i in range(t.shape[0]):
            t_list.append(t[i])
            v_dot_est, lambda_c_est, lambda_h_est= self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], lambda_c_gt[i,:], lambda_c_position[i,:], is_soft_constraints=False)
            v_dot_est_list.append(v_dot_est)
            lambda_c_est_list.append(lambda_c_est)
            lambda_h_est_list.append(lambda_h_est)
            v_dot_gt_list.append(v_dot_gt[i,:])
            lambda_c_gt_list.append(lambda_c_gt[i,:])
        
        t_list = np.array(t_list)
        v_dot_est_list = np.array(v_dot_est_list)
        lambda_c_est_list = np.array(lambda_c_est_list)
        lambda_h_est_list = np.array(lambda_h_est_list)
        lambda_c_gt_list = np.array(lambda_c_gt_list)
        v_dot_gt_list = np.array(v_dot_gt_list)

        # Save v_dot pictures
        for i in range(22):
            plt.cla()
            plt.plot(t_list, v_dot_gt_list[:,i], 'g', label='gt')
            plt.plot(t_list, v_dot_est_list[:,i], 'r', label='est')
            plt.legend()
            plt.title(self.vel_map_inverse[i])
            plt.ylim(-100,100)
            plt.savefig("bindings/pydairlib/cassie/residual_analysis/simulation_test/v_dot/{}.png".format(self.vel_map_inverse[i]))
        
        # Save lambda_c
        for i in range(12):
            plt.cla()
            plt.plot(t_list, lambda_c_est_list[:,i], 'r', label='est')
            plt.plot(t_list, lambda_c_gt_list[:,i], 'g', label='gt')
            plt.legend()
            plt.title(self.lambda_c_map_inverse[i])
            plt.ylim(-200,200)
            plt.savefig("bindings/pydairlib/cassie/residual_analysis/simulation_test/lambda_c/{}.png".format(self.lambda_c_map_inverse[i]))
        
        import pdb; pdb.set_trace()

    def interpolation(self, t,t1,t2,v1,v2):
        ratio = (t - t1)/(t2-t1)
        v = v1 + ratio * (v2 -v1)
        return v

    def process_sim_data(self, raw_data, start_time, end_time):

        # processing robot output
        robot_output = raw_data['robot_output']
        t_robot_output = robot_output["t_x"][0][0][0]
        start_index = np.argwhere(t_robot_output > start_time)[0][0]
        end_index = np.argwhere(t_robot_output < end_time)[-1][0]
        t_robot_output = t_robot_output[start_index:end_index]
        q = robot_output['q'][0][0][start_index:end_index,:]
        v = robot_output['v'][0][0][start_index:end_index,:]
        u = robot_output['u'][0][0][start_index:end_index,:]
        # obtained v_dot by finite diff
        smooth_window = 10
        v_dot = (robot_output['v'][0][0][start_index+smooth_window:end_index+smooth_window,:] - robot_output['v'][0][0][start_index-smooth_window:end_index-smooth_window,:])\
            /(robot_output["t_x"][0][0][0][start_index+smooth_window:end_index+smooth_window] - robot_output["t_x"][0][0][0][start_index-smooth_window:end_index-smooth_window])[:,None]
        
        # processing contact force
        contact_output = raw_data['contact_output']
        t_contact = contact_output['t_lambda'][0][0][0]
        start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
        end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
        t_contact = t_contact[start_index: end_index]
        contact_force = contact_output['lambda_c'][0][0]
        contact_position = contact_output['p_lambda_c'][0][0]
        left_toe_front_force = contact_force[0,start_index:end_index,:]
        left_toe_front_position = contact_position[0,start_index:end_index,:]
        left_toe_rear_force = contact_force[1,start_index:end_index,:]
        left_toe_rear_position = contact_position[1,start_index:end_index,:]
        right_toe_front_force = contact_force[2,start_index:end_index,:]
        right_toe_front_position = contact_position[2,start_index:end_index,:]
        right_toe_rear_force = contact_force[3,start_index:end_index,:]
        right_toe_rear_position = contact_position[3,start_index:end_index,:]
        contact_force_processed = []
        contact_position_processed = []
        pointer = 0
        for t in t_robot_output:
            while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
                pointer+=1
            left_toe_front_force_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1], 
                                                        left_toe_front_force[pointer,:], left_toe_front_force[pointer+1,:])
            left_toe_front_position_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        left_toe_front_position[pointer,:], left_toe_front_position[pointer+1,:])
            left_toe_rear_force_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        left_toe_rear_force[pointer,:], left_toe_rear_force[pointer+1,:])
            left_toe_rear_position_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        left_toe_rear_position[pointer,:], left_toe_rear_position[pointer+1,:])
            right_toe_front_force_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        right_toe_front_force[pointer,:], right_toe_front_force[pointer+1,:])
            right_toe_front_position_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        right_toe_front_position[pointer,:], right_toe_front_position[pointer+1,:])
            right_toe_rear_force_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        right_toe_rear_force[pointer,:], right_toe_rear_force[pointer+1,:])
            right_toe_rear_position_interpolated = self.interpolation(t, t_contact[pointer], t_contact[pointer+1],
                                                        right_toe_rear_position[pointer,:], right_toe_rear_position[pointer+1,:])
            joined_positions = np.hstack((left_toe_front_position_interpolated, left_toe_rear_position_interpolated,
                                        right_toe_front_position_interpolated, right_toe_rear_position_interpolated))
            joined_forces = np.hstack((left_toe_front_force_interpolated, left_toe_rear_force_interpolated, right_toe_front_force_interpolated, right_toe_rear_force_interpolated))
            contact_force_processed.append(joined_forces)
            contact_position_processed.append(joined_positions)
        contact_force_processed = np.array(contact_force_processed)
        contact_position_processed = np.array(contact_position_processed)

        # Get damping ratio
        damping_ratio = raw_data['damping_ratio']

        # Get spring stiffness
        spring_stiffness = raw_data['spring_stiffness']

        processed_data = {
            't':t_robot_output,
            'q':q,
            'v':v,
            'v_dot':v_dot,
            'u':u,
            'lambda_c':contact_force_processed,
            'lambda_c_position':contact_position_processed,
            'damping_ratio':damping_ratio,
            'spring_stiffness':spring_stiffness
        }

        return processed_data

def main():
    simulationDataTester = CaaiseSystemTest(data_path="log/20220530_1.mat", start_time=6.5, end_time=10)
    simulationDataTester.simulation_test()

if __name__ == "__main__":
    main()