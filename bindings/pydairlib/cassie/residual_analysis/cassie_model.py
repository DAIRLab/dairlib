import numpy as np
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import kinematic
from pydrake.multibody.tree import JacobianWrtVariable, MultibodyForces
from pydairlib.multibody import multibody
import pydairlib
import copy

class CassieSystem():
    def __init__(self):

        self.urdf = "examples/Cassie/urdf/cassie_v2.urdf"

        # Initialize for drake
        self.builder = DiagramBuilder()
        self.drake_sim_dt = 5e-5
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        addCassieMultibody(self.plant, self.scene_graph, True, 
                            self.urdf, True, True)
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
        self.spring_offset = np.zeros(23,)
        self.C = np.zeros((22,22))

        # Visualizer
        self.visualizer = multibody.MultiposeVisualizer(self.urdf, 1, '')

        self.pos_map, self.pos_map_inverse = self.make_position_map()
        self.vel_map, self.vel_map_inverse = self.make_velocity_map()
        self.act_map, self.act_map_inverse = self.make_actuator_map()

        self.intermediate_variables = []

    def drawPose(self, q):
        self.visualizer.DrawPoses(q)

    def get_spring_stiffness(self, K):
        self.K = K
    
    def get_damping(self, C):
        self.C = C

    def get_spring_offset(self, offset):
        self.spring_offset = offset

    def calc_vdot(self, t, q, v, u, is_contact=None, lambda_c_gt=None, lambda_c_position=None, v_dot_gt=None, u_osc=None, v_dot_osc=None, spring_mode="changed_stiffness"):
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

        if spring_mode not in ["fixed_spring", "changed_stiffness", "constant_stiffness"]:
            raise ValueError("Spring mode not valid")

        # Set configuration
        self.plant.SetPositionsAndVelocities(self.context, np.hstack((q,v)))

        # Get mass matrix
        M = self.plant.CalcMassMatrix(self.context)

        # Get bias term
        bias = self.plant.CalcBiasTerm(self.context)

        # Get the B matrix
        B = self.plant.MakeActuationMatrix()

        # Get the gravity term

        gravity = self.plant.CalcGravityGeneralizedForces(self.context)

        if spring_mode == "changed_stiffness":

            self.K = np.zeros((22,23))

            if np.array_equal(is_contact, np.array([1,0])):
                self.K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = 1749.1496
                self.K[self.vel_map["ankle_spring_joint_leftdot"],self.pos_map["ankle_spring_joint_left"]] = 1026.3388
                self.K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = 883.3358
                self.K[self.vel_map["ankle_spring_joint_rightdot"],self.pos_map["ankle_spring_joint_right"]] = 591.9035
                
                self.spring_offset[self.pos_map["knee_joint_left"]] = -10.9850/1749.1496
                self.spring_offset[self.pos_map["ankle_spring_joint_left"]] = 7.9933/1026.3388
                self.spring_offset[self.pos_map["knee_joint_right"]] = 3.1201/883.3358
                self.spring_offset[self.pos_map["ankle_spring_joint_right"]] = 9.5112/591.9035

            elif np.array_equal(is_contact, np.array([0,1])):
                self.K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = 1749.1496
                self.K[self.vel_map["ankle_spring_joint_rightdot"],self.pos_map["ankle_spring_joint_right"]] = 1026.3388
                self.K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = 883.3358
                self.K[self.vel_map["ankle_spring_joint_leftdot"],self.pos_map["ankle_spring_joint_left"]] = 591.9035
                
                self.spring_offset[self.pos_map["knee_joint_right"]] = -10.9850/1749.1496
                self.spring_offset[self.pos_map["ankle_spring_joint_right"]] = 7.9933/1026.3388
                self.spring_offset[self.pos_map["knee_joint_left"]] = 3.1201/883.3358
                self.spring_offset[self.pos_map["ankle_spring_joint_left"]] = 9.5112/591.9035
            
            else:
                self.K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = 808.3982
                self.K[self.vel_map["ankle_spring_joint_leftdot"],self.pos_map["ankle_spring_joint_left"]] = 519.0283
                self.K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = 808.3982
                self.K[self.vel_map["ankle_spring_joint_rightdot"],self.pos_map["ankle_spring_joint_right"]] = 519.0283
                
                self.spring_offset[self.pos_map["knee_joint_left"]] = 1.5369/808.3982
                self.spring_offset[self.pos_map["ankle_spring_joint_left"]] = 6.0154/519.0283
                self.spring_offset[self.pos_map["knee_joint_right"]] = 1.5369/808.3982
                self.spring_offset[self.pos_map["ankle_spring_joint_right"]] = 6.0154/519.0283
        
        damping_force = -self.C @ v
        spring_force = -self.K @ (q - self.spring_offset)
        damping_and_spring_force = damping_force + spring_force

        # Get the J_h term
        J_h = np.zeros((2, 22))
        J_h[0, :] = self.left_loop.EvalFullJacobian(self.context)
        J_h[1, :] = self.right_loop.EvalFullJacobian(self.context)

        # Get the JdotTimesV term
        J_h_dot_times_v = np.zeros(2,)
        J_h_dot_times_v[0] = self.left_loop.EvalFullJacobianDotTimesV(self.context)
        J_h_dot_times_v[1] = self.right_loop.EvalFullJacobianDotTimesV(self.context)

        # Get the J_c_active term
        J_c_active, J_c_active_dot_v, num_contact_unknown, get_force_at_point_matrix = self.get_J_c_ative_and_J_c_active_dot_v(is_contact=is_contact, lambda_c_gt=lambda_c_gt, lambda_c_position=lambda_c_position)
    
        # Get the J_s term 
        J_s = np.zeros((4, 22))
        J_s[0, self.vel_map["knee_joint_leftdot"]] = 1
        J_s[1, self.vel_map["knee_joint_rightdot"]] = 1
        J_s[2, self.vel_map["ankle_spring_joint_leftdot"]] = 1
        J_s[3, self.vel_map["ankle_spring_joint_rightdot"]] = 1

        if spring_mode == "fixed_spring":
            if num_contact_unknown > 0:
                A = np.vstack((
                    np.hstack((M, -J_c_active.T, -J_h.T, -J_s.T)),
                    np.hstack((J_h, np.zeros((2,num_contact_unknown)), np.zeros((2,2)), np.zeros((2,4)) )),
                    np.hstack((J_c_active, np.zeros((num_contact_unknown, num_contact_unknown)), np.zeros((num_contact_unknown,2)), np.zeros((num_contact_unknown, 4)) )),
                    np.hstack((J_s, np.zeros((4, num_contact_unknown)), np.zeros((4,2)), np.zeros((4,4)) ))
                ))

                b = np.hstack((
                    B @ u + gravity - bias + damping_and_spring_force,
                    -J_h_dot_times_v,
                    -J_c_active_dot_v,
                    np.zeros(4)
                ))
            else:
                A = np.vstack((
                    np.hstack((M, -J_h.T ,-J_s.T)),
                    np.hstack((J_h, np.zeros((2, 2)), np.zeros((2,4)))),
                    np.hstack((J_s, np.zeros((4,2)), np.zeros((4,4)) ))
                    ))

                b = np.hstack((
                    B @ u + gravity - bias + damping_and_spring_force,
                    -J_h_dot_times_v,            
                    np.zeros(4)
                ))
        else:
            if num_contact_unknown > 0:
                A = np.vstack((
                    np.hstack((M, -J_c_active.T, -J_h.T, )),
                    np.hstack((J_h, np.zeros((2,num_contact_unknown)),np.zeros((2,2)), )),
                    np.hstack((J_c_active, np.zeros((num_contact_unknown, num_contact_unknown)), np.zeros((num_contact_unknown,2)),  )),
                ))

                b = np.hstack((
                    B @ u + gravity - bias + damping_and_spring_force,
                    -J_h_dot_times_v,
                    -J_c_active_dot_v,
                ))
            else:
                A = np.vstack((
                    np.hstack((M, -J_h.T,)),
                    np.hstack((J_h, np.zeros((2, 2)))),
                    ))

                b = np.hstack((
                    B @ u + gravity - bias + damping_and_spring_force,
                    -J_h_dot_times_v,            
                ))

        solution = A.T @ np.linalg.inv(A @ A.T) @ b

        v_dot = solution[:22]
        if num_contact_unknown > 0:
            lambda_c = get_force_at_point_matrix @ solution[22:22+num_contact_unknown]
        else:
            lambda_c = np.zeros(6,)
        lambda_h = solution[22+num_contact_unknown:24+num_contact_unknown]

        left_toe_mid_point_on_world, right_toe_mid_point_on_world, v_left_toe_mid_in_world, v_right_toe_mid_in_world = self.get_toe_q_and_v_in_world(q, v)

        # save all intermediate variables for later use
        intermediate_variables = {"t":t, "q":q, "v":v, "u":u, "is_contact":is_contact, "lambda_c_gt":lambda_c_gt, "lambda_c_position":lambda_c_position, "v_dot_gt": v_dot_gt, "u_osc":u_osc, "v_dot_osc":v_dot_osc, "spring_mode":spring_mode,
        "M":M, "bias":bias, "B":B, "gravity":gravity, "K":self.K, "C":self.C, "damping_force":damping_force, "spring_force":spring_force, "damping_force":damping_force, "damping_and_spring_force":damping_and_spring_force,
        "J_h": J_h, "J_h_dot_v": J_h_dot_times_v, "J_c_active":J_c_active, "J_c_active_dot_v":J_c_active_dot_v, "J_s":J_s,"num_contact_unknown":num_contact_unknown, "get_force_at_point_matrix":get_force_at_point_matrix, 
        "A":A, "b":b, "solution":solution, "v_dot":v_dot, "lambda_c":lambda_c, "lambda_h":lambda_h, "left_toe_mid_point_on_world":left_toe_mid_point_on_world, "right_toe_mid_point_on_world": right_toe_mid_point_on_world, 
        "v_left_toe_mid_in_world":v_left_toe_mid_in_world, "v_right_toe_mid_in_world":v_right_toe_mid_in_world
        }

        self.intermediate_variables.append(intermediate_variables)
    
        return v_dot, lambda_c, lambda_h

    def get_toe_q_and_v_in_world(self, q, v):

        context = copy.deepcopy(self.context)

        self.plant.SetPositionsAndVelocities(context, np.hstack((q,v)))
        
        left_toe_front_on_foot, left_foot_frame = LeftToeFront(self.plant)
        left_toe_rear_on_foot, left_foot_frame = LeftToeRear(self.plant)
        right_toe_front_on_foot, right_foot_frame = RightToeFront(self.plant)
        right_toe_rear_on_foot, right_foot_frame = RightToeRear(self.plant)

        left_toe_mid_point = (left_toe_front_on_foot + left_toe_rear_on_foot)/2
        right_toe_mid_point = (right_toe_front_on_foot + right_toe_rear_on_foot)/2

        left_toe_mid_point_on_world = self.plant.CalcPointsPositions(context, left_foot_frame, left_toe_mid_point, self.world).squeeze()
        right_toe_mid_point_on_world = self.plant.CalcPointsPositions(context, right_foot_frame, right_toe_mid_point, self.world).squeeze()

        v_left_toe_mid_in_world = self.plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, left_foot_frame, left_toe_mid_point, self.world, self.world)
        v_right_toe_mid_in_world = self.plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, right_foot_frame, right_toe_mid_point, self.world, self.world)

        return left_toe_mid_point_on_world, right_toe_mid_point_on_world, v_left_toe_mid_in_world, v_right_toe_mid_in_world

    def get_J_c_ative_and_J_c_active_dot_v(self, is_contact, lambda_c_gt, lambda_c_position):
        J_c_active = None; J_c_active_dot_v = None; num_contact_unknown = 0
        left_front_index = None; left_rear_index = None
        right_front_index = None; right_rear_index = None
        left_index = None; right_index = None
        if lambda_c_gt is not None:
            if np.linalg.norm(lambda_c_gt[:3]) > 0:
                point_on_foot, foot_frame = LeftToeFront(self.plant)
                if lambda_c_position is not None:
                    point_on_foot_on_world = lambda_c_position[:3]
                    point_on_foot = self.plant.CalcPointsPositions(self.context, self.world, point_on_foot_on_world, foot_frame)
                J_c_left_front = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
                J_c_left_front_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
                J_c_active = J_c_left_front
                J_c_active_dot_v = J_c_left_front_dot_v
                left_front_index = num_contact_unknown
                num_contact_unknown += 3
            if np.linalg.norm(lambda_c_gt[3:6]) > 0:
                point_on_foot, foot_frame = LeftToeRear(self.plant)
                if lambda_c_position is not None:
                    point_on_foot_on_world = lambda_c_position[3:6]
                    point_on_foot = self.plant.CalcPointsPositions(self.context, self.world, point_on_foot_on_world, foot_frame)
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
                point_on_foot, foot_frame = RightToeFront(self.plant)
                if lambda_c_position is not None:
                    point_on_foot_on_world = lambda_c_position[6:9]
                    point_on_foot = self.plant.CalcPointsPositions(self.context, self.world, point_on_foot_on_world, foot_frame)
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
                point_on_foot, foot_frame = RightToeRear(self.plant)
                if lambda_c_position is not None:
                    point_on_foot_on_world = lambda_c_position[9:12]
                    point_on_foot = self.plant.CalcPointsPositions(self.context, self.world, point_on_foot_on_world, foot_frame)
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
        else:
            if is_contact[0] > 0:
                toe_front, foot_frame = LeftToeFront(self.plant)
                toe_rear, foot_frame = LeftToeRear(self.plant)
                point_on_foot = (toe_front + toe_rear)/2
                J_c_left = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
                J_c_left_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
                J_c_active = J_c_left
                J_c_active_dot_v = J_c_left_dot_v
                left_index = num_contact_unknown
                num_contact_unknown += 3
            if is_contact[1] > 0:
                toe_front, foot_frame = RightToeFront(self.plant)
                toe_rear, foot_frame = RightToeRear(self.plant)
                point_on_foot = (toe_front + toe_rear)/2
                J_c_right = self.plant.CalcJacobianTranslationalVelocity(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world)
                J_c_right_dot_v = self.plant.CalcBiasTranslationalAcceleration(self.context, JacobianWrtVariable.kV, foot_frame, point_on_foot, self.world, self.world).squeeze()
                if J_c_active is None:
                    J_c_active = J_c_right
                    J_c_active_dot_v = J_c_right_dot_v    
                else:
                    J_c_active = np.vstack((J_c_active, J_c_right))
                    J_c_active_dot_v = np.hstack((J_c_active_dot_v, J_c_right_dot_v))

                right_index = num_contact_unknown
                num_contact_unknown += 3

            get_force_at_point_matrix = np.zeros((6, num_contact_unknown))
            if left_index is not None:
                get_force_at_point_matrix[0,left_index] = 1
                get_force_at_point_matrix[1,left_index+1] = 1
                get_force_at_point_matrix[2,left_index+2] = 1
            if right_index is not None:
                get_force_at_point_matrix[3, right_index] = 1
                get_force_at_point_matrix[4, right_index+1] = 1
                get_force_at_point_matrix[5, right_index+2] = 1

        return J_c_active, J_c_active_dot_v, num_contact_unknown, get_force_at_point_matrix      

    def make_position_map(self):

        pos_map = pydairlib.multibody.makeNameToPositionsMap(self.plant)
        pos_map_inverse = {value:key for (key, value) in pos_map.items()}
        
        return pos_map, pos_map_inverse
    
    def make_velocity_map(self):

        vel_map = pydairlib.multibody.makeNameToVelocitiesMap(self.plant)
        vel_map_inverse = {value:key for (key, value) in vel_map.items()}

        return vel_map, vel_map_inverse

    def make_actuator_map(self):

        act_map = pydairlib.multibody.makeNameToActuatorsMap(self.plant)
        act_map_inverse = {value:key for (key, value) in act_map.items()}

        return act_map, act_map_inverse