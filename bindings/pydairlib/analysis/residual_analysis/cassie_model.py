import numpy as np
from scipy.spatial.transform import Rotation
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable, JointIndex,ForceElementIndex
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import multibody, kinematic
import pydairlib
import copy
import io
from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

class CassieSystem():
    def __init__(self):

        self.urdf = "examples/Cassie/urdf/cassie_v2.urdf"

        self.piecewice_linear_spring_config = "bindings/pydairlib/analysis/residual_analysis/piecewise_linear_spring.yaml"

        # Initialize for drake
        self.builder = DiagramBuilder()
        self.drake_sim_dt = 5e-5
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, self.drake_sim_dt)
        AddCassieMultibody(self.plant, self.scene_graph, True, 
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

        self.num_pos = self.plant.num_positions()
        self.num_vel = self.plant.num_velocities()
        
        self.pos_map, self.pos_map_inverse = self.make_position_map()
        self.vel_map, self.vel_map_inverse = self.make_velocity_map()
        self.act_map, self.act_map_inverse = self.make_actuator_map()

        # Initial other parameters
        self.piecewice_linear_spring_params = load(io.open(self.piecewice_linear_spring_config,"r"), Loader=Loader)
        self.initial_spring_and_damping()

        # Visualizer
        self.visualizer = multibody.MultiposeVisualizer(self.urdf, 1, '')

        self.intermediate_variables_list = []

    def initial_spring_and_damping(self):
        self.K = np.zeros((self.num_vel,self.num_pos))
        self.spring_offset = np.zeros(self.num_pos,)
        self.C = np.zeros((self.num_vel,self.num_vel))

        for force_element_idx in range(self.plant.num_force_elements()):
            force_element = self.plant.get_force_element(ForceElementIndex(force_element_idx))
            if hasattr(force_element, 'joint') and hasattr(force_element, 'stiffness'):
                self.K[self.vel_map[force_element.joint().name() + 'dot'], self.pos_map[force_element.joint().name()]] = force_element.stiffness()


        for joint_idx in range(self.plant.num_joints()):
            joint = self.plant.get_joint(JointIndex(joint_idx))
            if joint.num_velocities() > 0:
                self.C[self.vel_map[joint.name() + 'dot'], self.vel_map[joint.name() + 'dot']] = joint.damping()
    
    def draw_pose(self, q):
        self.visualizer.DrawPoses(q)

    def set_spring_stiffness(self, K):
        self.K = K
    
    def set_damping(self, C):
        self.C = C

    def set_spring_offset(self, offset):
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
        
            """
            The changed stiffness mode means piecewise linear spring model.

            The spring stiffness and offset are fitted separately for both stance and swing phases.

            The values comes a running log (03_15_22-log_11).
            """

            self.K = np.zeros((self.num_vel,self.num_pos))

            # left leg touch the ground
            if is_contact[0] == 1:
                self.K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = self.piecewice_linear_spring_params["stance"]["knee_spring"]["stiffness"]
                self.K[self.vel_map["ankle_spring_joint_leftdot"],self.pos_map["ankle_spring_joint_left"]] = self.piecewice_linear_spring_params["stance"]["ankle_spring"]["stiffness"]
                self.spring_offset[self.pos_map["knee_joint_left"]] = self.piecewice_linear_spring_params["stance"]["knee_spring"]["offset"]
                self.spring_offset[self.pos_map["ankle_spring_joint_left"]] = self.piecewice_linear_spring_params["stance"]["ankle_spring"]["offset"]
            # left leg in the air
            else:
                self.K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = self.piecewice_linear_spring_params["swing"]["knee_spring"]["stiffness"]
                self.K[self.vel_map["ankle_spring_joint_leftdot"],self.pos_map["ankle_spring_joint_left"]] = self.piecewice_linear_spring_params["swing"]["ankle_spring"]["stiffness"]
                self.spring_offset[self.pos_map["knee_joint_left"]] = self.piecewice_linear_spring_params["swing"]["knee_spring"]["offset"]
                self.spring_offset[self.pos_map["ankle_spring_joint_left"]] = self.piecewice_linear_spring_params["swing"]["ankle_spring"]["offset"]
                
            # right leg in the gorund
            if is_contact[1] == 1:
                self.K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = self.piecewice_linear_spring_params["stance"]["knee_spring"]["stiffness"]
                self.K[self.vel_map["ankle_spring_joint_rightdot"],self.pos_map["ankle_spring_joint_right"]] = self.piecewice_linear_spring_params["stance"]["ankle_spring"]["stiffness"]
                self.spring_offset[self.pos_map["knee_joint_right"]] = self.piecewice_linear_spring_params["stance"]["knee_spring"]["offset"]
                self.spring_offset[self.pos_map["ankle_spring_joint_right"]] = self.piecewice_linear_spring_params["stance"]["ankle_spring"]["offset"]
            # right leg in the air
            else:
                self.K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = self.piecewice_linear_spring_params["swing"]["knee_spring"]["stiffness"]
                self.K[self.vel_map["ankle_spring_joint_rightdot"],self.pos_map["ankle_spring_joint_right"]] = self.piecewice_linear_spring_params["swing"]["ankle_spring"]["stiffness"]
                self.spring_offset[self.pos_map["knee_joint_right"]] = self.piecewice_linear_spring_params["swing"]["knee_spring"]["offset"]
                self.spring_offset[self.pos_map["ankle_spring_joint_right"]] = self.piecewice_linear_spring_params["swing"]["ankle_spring"]["offset"]
            
        damping_force = -self.C @ v
        spring_force = -self.K @ (q - self.spring_offset)
        damping_and_spring_force = damping_force + spring_force

        # Get the J_h term
        J_h = np.zeros((2, self.num_vel))
        J_h[0, :] = self.left_loop.EvalFullJacobian(self.context)
        J_h[1, :] = self.right_loop.EvalFullJacobian(self.context)

        # Get the JdotTimesV term
        J_h_dot_times_v = np.zeros(2,)
        J_h_dot_times_v[0] = self.left_loop.EvalFullJacobianDotTimesV(self.context)
        J_h_dot_times_v[1] = self.right_loop.EvalFullJacobianDotTimesV(self.context)

        # Get the J_c_active term
        J_c_active, J_c_active_dot_v, num_contact_unknown, get_force_at_point_matrix = self.get_J_c_ative_and_J_c_active_dot_v(is_contact=is_contact, lambda_c_gt=lambda_c_gt, lambda_c_position=lambda_c_position)
    
        # Get the J_s term 
        J_s = np.zeros((4, self.num_vel))
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

        v_dot = solution[:self.num_vel]
        if num_contact_unknown > 0:
            lambda_c = get_force_at_point_matrix @ solution[self.num_vel:self.num_vel+num_contact_unknown]
        else:
            lambda_c = np.zeros(6,)
        lambda_h = solution[self.num_vel+num_contact_unknown:24+num_contact_unknown]

        left_toe_mid_point_on_world, right_toe_mid_point_on_world, v_left_toe_mid_in_world, v_right_toe_mid_in_world = self.get_toe_q_and_v_in_world(q, v)

        # save all intermediate variables for later use
        intermediate_variables = {"t":t, "q":q, "v":v, "u":u, "is_contact":is_contact, "lambda_c_gt":lambda_c_gt, "lambda_c_position":lambda_c_position, "v_dot_gt": v_dot_gt, "u_osc":u_osc, "v_dot_osc":v_dot_osc, "spring_mode":spring_mode,
        "M":M, "bias":bias, "B":B, "gravity":gravity, "K":self.K, "C":self.C, "damping_force":damping_force, "spring_force":spring_force, "damping_force":damping_force, "damping_and_spring_force":damping_and_spring_force,
        "J_h": J_h, "J_h_dot_v": J_h_dot_times_v, "J_c_active":J_c_active, "J_c_active_dot_v":J_c_active_dot_v, "J_s":J_s,"num_contact_unknown":num_contact_unknown, "get_force_at_point_matrix":get_force_at_point_matrix, 
        "A":A, "b":b, "solution":solution, "v_dot":v_dot, "lambda_c":lambda_c, "lambda_h":lambda_h, "left_toe_mid_point_on_world":left_toe_mid_point_on_world, "right_toe_mid_point_on_world": right_toe_mid_point_on_world, 
        "v_left_toe_mid_in_world":v_left_toe_mid_in_world, "v_right_toe_mid_in_world":v_right_toe_mid_in_world
        }

        self.intermediate_variables_list.append(intermediate_variables)
    
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

        # The matrix is linear map, 
        # which maps the 3*num_of_contact dimension unknown contact forces,
        # to the 6 dimension order contact forces. 
        # e.g. When only right leg is on the ground, the matrix turn the unknown contact forces at right leg to a vector as [0,0,0,F_right_x, F_right_y, F_right_z].T
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

        pos_map = pydairlib.multibody.MakeNameToPositionsMap(self.plant)
        pos_map_inverse = {value:key for (key, value) in pos_map.items()}
        
        return pos_map, pos_map_inverse
    
    def make_velocity_map(self):

        vel_map = pydairlib.multibody.MakeNameToVelocitiesMap(self.plant)
        vel_map_inverse = {value:key for (key, value) in vel_map.items()}

        return vel_map, vel_map_inverse

    def make_actuator_map(self):

        act_map = pydairlib.multibody.MakeNameToActuatorsMap(self.plant)
        act_map_inverse = {value:key for (key, value) in act_map.items()}

        return act_map, act_map_inverse