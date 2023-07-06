from pydrake.solvers import MathematicalProgram 
from pydrake.solvers import Solve
import matplotlib.pyplot as plt
from matplotlib import patches
import tqdm
from plot_utlis import *
from utils import *
from scipy.spatial.transform import Rotation
import numpy as np

class CassieResidualAnalyzer():

    def __init__(self):
        pass

    def set_num_of_pos(self, num_pos):
        self.num_pos = num_pos
    
    def set_num_of_vel(self, num_vel):
        self.num_vel = num_vel

    def set_pos_map(self, pos_map, pos_map_inverse):
        self.pos_map = pos_map
        self.pos_map_inverse = pos_map_inverse

    def set_vel_map(self, vel_map, vel_map_inverse):
        self.vel_map = vel_map
        self.vel_map_inverse = vel_map_inverse
    
    def set_act_map(self, act_map, act_map_inverse):
        self.act_map = act_map
        self.act_map_inverse = act_map_inverse

    def set_data(self, data):
        self.original_data = data

    def construct_dict_for_v_shape_value(self, v):
        dicts = {}
        for i in range(self.num_vel):
            dicts[self.vel_map_inverse[i]] = v[i]
        return dicts

    def construct_dict_for_q_shape_value(self, q):
        dicts = {}
        for i in range(self.num_pos):
            dicts[self.pos_map_inverse[i]] = q[i]
        return dicts

    def construct_dict_for_spring_force_value(self, spring_force):
        order_dict = {0:"knee_joint_left", 1:"knee_joint_right", 2:"ankle_spring_joint_left", 3:"ankle_spring_joint_right"}
        dicts = {}
        for order in order_dict:
            dicts[order_dict[order]] = spring_force[order]
        return dicts

    def construct_dict_for_u_shape_value(self, u):
        dicts = {}
        for i in range(10):
            dicts[self.act_map_inverse[i]] = u[i]
        return dicts

    def construct_dict_for_left(self, value):
        dicts = {}
        dicts["left"] = value
        return dicts
    
    def construct_dict_for_right(self, value):
        dicts = {}
        dicts["right"] = value
        return dicts

    def fit_best_spring(self):
        """Calculate the what spring can do best"""

        print("Begin fitting the best spring model.")
        
        best_spring_forces = []
        v_dot_of_best_spring_model = []
        v_dot_gts = []
        
        # Selected joints that have encoders in them, such that the corresponding ground truth v_dot are more accurate.
        selected_joints = ["hip_roll_leftdot", "hip_roll_rightdot", "hip_pitch_leftdot", "hip_pitch_rightdot", "hip_yaw_leftdot", "hip_yaw_rightdot",
                        "knee_leftdot", "knee_rightdot", "knee_joint_leftdot", "knee_joint_rightdot", "ankle_joint_leftdot", "ankle_joint_rightdot"]
        selection_matrix = np.zeros((self.num_vel,self.num_vel))
        for selected_joint in selected_joints:
            selection_matrix[self.vel_map[selected_joint], self.vel_map[selected_joint]] = 1

        for i in tqdm.trange(len(self.original_data)):
            
            # Initialize the program
            prog = MathematicalProgram()

            # Definie decision variables
            spring_force = prog.NewContinuousVariables(4)
            v_dot = prog.NewContinuousVariables(self.num_vel)

            # Get the parameters 
            datum = self.original_data[i]
            v_dot_gt = datum["v_dot_gt"]
            num_of_contact = datum["num_contact_unknown"]
            A = datum["A"]
            B = datum["B"]
            J_c_active_dot_v = datum["J_c_active_dot_v"]
            J_h_active_dot_v = datum["J_h_dot_v"]
            gravity = datum["gravity"]
            bias = datum["bias"]
            damping_force = datum["damping_force"]
            J_s = datum["J_s"]
            u = datum["u"]
            
            A_inverse = np.linalg.inv(A)
            
            v_dot_gts.append(v_dot_gt)

            # Define cost
            # The cost is define as L2 norm of "selection_matrix @ (v_dot_gt - v_dot)"
            # Rewrite the function to use AddL2NormCost as |Ax-b|^2
            prog.Add2NormSquaredCost(selection_matrix, selection_matrix@v_dot_gt, v_dot)

            # Add dynamics constraints:

            # The true constraints should be:
            # (A_inverse @ b) [:self.num_vel] == v_dot, 
            # where if num_of_contact > 0:
            # b = np.hstack((
            #         B @ u + gravity - bias + damping_force + J_s.T @ spring_force,
            #         -J_h_active_dot_v,
            #         -J_c_active_dot_v,
            #       ))
            # else: 
            # b = np.hstack((
            #         B @ u + gravity - bias + damping_force + J_s.T @ spring_force,
            #         -J_h_active_dot_v,            
            #     )),
            # A is the corresponding coefficients relate the mass matrix, holomonic and contact constraints and other infos, see cassie_model.py for details

            # The constraint is rewritten to compatible to drake optimization funcion.
            # Separate A_inverse into 
            # [A_inverse_11, A_inverse_12,
            # A_inverse_21, A_inverse_22] and rewrite the constraints as
            # A_inverse 11 @ J_s.T @ spring_force - v_dot = - A_inverse_11 @ (B @ u + gravity - bias +damping_force) + A_inverse_12 @ ( J_h_active_dot_v + J_c_active_dot_v)
            # [A_inverse_11@J_s.T, -I] [spring_force// v_dot] = - A_inverse_11 @ (B @ u + gravity - bias +damping_force) + A_inverse_12 @ ( J_h_active_dot_v + J_c_active_dot_v) 

            A_inverse_11 = A_inverse[:self.num_vel, :self.num_vel]
            A_inverse_12 = A_inverse[:self.num_vel, self.num_vel:]

            if num_of_contact > 0:
                prog.AddLinearEqualityConstraint(
                    np.hstack((A_inverse_11@J_s.T, -np.eye(self.num_vel))),
                    - A_inverse_11 @ (B @ u + gravity - bias + damping_force) + A_inverse_12 @ np.hstack((J_h_active_dot_v,J_c_active_dot_v)),
                    np.hstack((spring_force, v_dot))
                )
            else:
                prog.AddLinearEqualityConstraint(
                    np.hstack((A_inverse_11@J_s.T, -np.eye(self.num_vel))),
                    - A_inverse_11 @ (B @ u + gravity - bias + damping_force) + A_inverse_12 @ J_h_active_dot_v,
                    np.hstack((spring_force, v_dot))
                )

            result = Solve(prog)

            v_dot_of_best_spring_model.append(result.GetSolution(v_dot))
            best_spring_forces.append(result.GetSolution(spring_force))

        best_spring_forces = np.array(best_spring_forces)
        v_dot_gts = np.array(v_dot_gts)
        v_dot_of_best_spring_model = np.array(v_dot_of_best_spring_model)
        residual = get_residual(v_dot_gts, v_dot_of_best_spring_model)

        print("Finish fitting the best spring model.")

        return residual, v_dot_of_best_spring_model, best_spring_forces

    def calc_spring_constant(self, q, best_spring_forces, is_show_value=False):
        
        n = q.shape[0]

        print("begin to fix the spring model:")

        A_knee_left = np.ones((n, 2))
        b_knee_left = np.zeros(n)
        A_knee_right = np.ones((n, 2))
        b_knee_right = np.zeros(n)
        A_ankle_left = np.ones((n, 2))
        b_ankle_left = np.zeros(n)
        A_ankle_right = np.ones((n,2))
        b_ankle_right = np.zeros(n)


        for i in range(n):
            A_knee_left [i,0] = q[i, self.pos_map["knee_joint_left"]]
            b_knee_left[i] = best_spring_forces[i,0]
            A_knee_right[i,0] = q[i, self.pos_map["knee_joint_right"]] 
            b_knee_right[i] = best_spring_forces[i,1]
            A_ankle_left[i,0] = q[i, self.pos_map["ankle_spring_joint_left"]]
            b_ankle_left[i] = best_spring_forces[i,2]
            A_ankle_right[i,0] = q[i, self.pos_map["ankle_spring_joint_right"]]
            b_ankle_right[i] = best_spring_forces[i,3]

        knee_left_sol = np.linalg.inv(A_knee_left.T @ A_knee_left) @ A_knee_left.T @ b_knee_left
        knee_right_sol = np.linalg.inv(A_knee_right.T @ A_knee_right) @ A_knee_right.T @ b_knee_right 
        ankle_left_sol = np.linalg.inv(A_ankle_left.T @ A_ankle_left) @ A_ankle_left.T @ b_ankle_left
        ankle_right_sol = np.linalg.inv(A_ankle_right.T @ A_ankle_right) @ A_ankle_right.T @ b_ankle_right

        K = -np.array([knee_left_sol[0], knee_right_sol[0], ankle_left_sol[0], ankle_right_sol[0]])
        offset = -np.array([knee_left_sol[1]/knee_left_sol[0], knee_right_sol[1]/knee_right_sol[0], ankle_left_sol[1]/ankle_left_sol[0], ankle_right_sol[1]/ankle_right_sol[0]])

        if is_show_value:
            print(f"{'Joint name':>15} {'K':>10} {'Offset':>10}")
            print(f"{'Knee left':<15} {K[0]:>10.2f} {offset[0]:>10.2f}")
            print(f"{'Knee right':<15} {K[1]:>10.2f} {offset[1]:>10.2f}")
            print(f"{'Ankle left':<15} {K[2]:>10.2f} {offset[2]:>10.2f}")
            print(f"{'Ankle left':<15} {K[3]:>10.2f} {offset[3]:>10.2f}")

        return K, offset

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name="residual_for_best_spring_model"):

        threshholds = {"hip_roll_leftdot":100,
                        "hip_roll_rightdot":100,
                        "hip_pitch_leftdot":100,
                        "hip_pitch_rightdot":100,
                        "hip_yaw_leftdot":100,
                        "hip_yaw_rightdot":100}

        residuals = {}

        t = []

        color_corresponding = {"hip_roll_leftdot":'aqua',
                                "hip_roll_rightdot":'blue',
                                "hip_pitch_leftdot":'brown',
                                "hip_pitch_rightdot":'coral',
                                "hip_yaw_leftdot":'silver',
                                "hip_yaw_rightdot":'yellow'}

        start_index = 0
        while self.processed_data[start_index]["t"] < start_time:
            start_index += 1
        end_index = len(self.processed_data)-1
        while self.processed_data[end_index]["t"] > end_time:
            end_index -= 1
    
        for datum in self.processed_data[start_index:end_index]:
            t.append(datum['t'])
            single_residual_info = datum[residual_name]
            for joint_name in single_residual_info:
                if joint_name not in threshholds:
                    continue
                if joint_name in residuals:
                    residuals[joint_name].append(single_residual_info[joint_name])
                else:
                    residuals[joint_name] = [single_residual_info[joint_name]]

        t = np.array(t)

        max_value = None
        min_value = None

        plt.figure()

        for joint_name in residuals:
            residuals[joint_name] = np.array(residuals[joint_name])
            if max_value is None or np.max(residuals[joint_name]) > max_value:
                max_value = np.max(residuals[joint_name])
            if min_value is None or np.min(residuals[joint_name]) < min_value:
                min_value = np.min(residuals[joint_name])
            plt.plot(t, residuals[joint_name], label=joint_name, color=color_corresponding[joint_name])

        rects = []

        for joint_name in residuals:
            indices = np.argwhere(np.abs(residuals[joint_name])>threshholds[joint_name])
            if indices.shape[0] == 0:
                continue
            start_index = indices[0]
            end_index = indices[-1]
            rect = patches.Rectangle((t[start_index], min_value), t[end_index] - t[start_index], max_value-min_value, color=color_corresponding[joint_name], alpha=0.8)
            rects.append({"rect":rect,
                        "start_index":start_index,
                        "end_index":end_index})
        rects.sort(key=lambda x: x["start_index"])
        
        for rect in rects:
            plt.gca().add_patch(rect["rect"])

        plt.xlabel("t")
        plt.ylabel("residuals")
        plt.legend()
        plt.show()

    def calc_contact_mode_changed_time_points(self, is_contact, t, stable_threshold=0.02):

        left_leg_leaving_time_points = []
        left_leg_landing_time_points = []
        right_leg_leaving_time_points = []
        right_leg_landing_time_points = []

        pre_time_point_left_mode = is_contact[0,0]
        pre_time_point_right_mode = is_contact[0,1]

        for i in range(1, is_contact.shape[0]):

            if pre_time_point_left_mode != is_contact[i,0]:
                is_stable = True
                j = i
                while j < t.shape[0]-1 and t[j] < t[i] + stable_threshold:
                    j += 1
                    if is_contact[j,0] != is_contact[i,0]:
                        is_stable = False
                        break
                if is_stable:
                    if pre_time_point_left_mode == 0:
                        left_leg_landing_time_points.append(t[i])
                    else:
                        left_leg_leaving_time_points.append(t[i])
                    pre_time_point_left_mode = is_contact[i,0]

            if pre_time_point_right_mode != is_contact[i,1]:
                is_stable = True
                j = i
                while j < t.shape[0]-1 and t[j] < t[i] + stable_threshold:
                    j += 1
                    if is_contact[j,1] != is_contact[i,1]:
                        is_stable = False
                        break
                if is_stable:
                    if pre_time_point_right_mode == 0:
                        right_leg_landing_time_points.append(t[i])
                    else:
                        right_leg_leaving_time_points.append(t[i])
                    pre_time_point_right_mode = is_contact[i,1]
        
        left_leg_leaving_time_points = np.array(left_leg_leaving_time_points)
        left_leg_landing_time_points = np.array(left_leg_landing_time_points)
        right_leg_leaving_time_points = np.array(right_leg_leaving_time_points)
        right_leg_landing_time_points = np.array(right_leg_landing_time_points)

        return left_leg_leaving_time_points, left_leg_landing_time_points, right_leg_leaving_time_points, right_leg_landing_time_points

    def calc_contact_mode_maintained_time_periods(self, is_contact, t, stable_threshold=0.05):
        
        left_leg_on_the_air_time_periods = []
        left_leg_on_the_ground_time_periods = []
        right_leg_on_the_air_time_periods = []
        right_leg_on_the_ground_time_periods = []

        pre_left_leg_mode = is_contact[0,0]
        pre_left_leg_mode_start_time = t[0]
        pre_right_leg_mode = is_contact[0,1]
        pre_right_leg_mode_start_time = t[0]

        for i in range(1, is_contact.shape[0]):

            if pre_left_leg_mode != is_contact[i,0]:
                if pre_left_leg_mode == 0:
                    left_leg_on_the_air_time_periods.append([pre_left_leg_mode_start_time, t[i-1]])
                else:
                    left_leg_on_the_ground_time_periods.append([pre_left_leg_mode_start_time, t[i-1]])
                pre_left_leg_mode_start_time = t[i]
                pre_left_leg_mode = is_contact[i,0]
            
            if pre_right_leg_mode != is_contact[i,1]:
                if pre_right_leg_mode == 0:
                    right_leg_on_the_air_time_periods.append([pre_right_leg_mode_start_time, t[i-1]])
                else:
                    right_leg_on_the_ground_time_periods.append([pre_right_leg_mode_start_time, t[i-1]])
                pre_right_leg_mode_start_time = t[i]
                pre_right_leg_mode = is_contact[i,1]

        left_leg_on_the_air_time_periods = np.array([[x[0], x[1]] for x in left_leg_on_the_air_time_periods if x[1] - x[0] > stable_threshold])
        left_leg_on_the_ground_time_periods = np.array([[x[0], x[1]] for x in left_leg_on_the_ground_time_periods if x[1] - x[0] > stable_threshold])
        right_leg_on_the_air_time_periods = np.array([[x[0], x[1]] for x in right_leg_on_the_air_time_periods if x[1] - x[0] > stable_threshold])
        right_leg_on_the_ground_time_periods = np.array([[x[0], x[1]] for x in right_leg_on_the_ground_time_periods if x[1] - x[0] > stable_threshold])

        return left_leg_on_the_air_time_periods, left_leg_on_the_ground_time_periods, right_leg_on_the_air_time_periods, right_leg_on_the_ground_time_periods

    def calc_residuals_of_linear_spring_given_time_periods(self, start_time, end_time):
        
        residuals = []

        start_index = 0
        while self.processed_data[start_index]["t"] < start_time:
            start_index += 1
        end_index = len(self.processed_data)-1
        while self.processed_data[end_index]["t"] > end_time:
            end_index -= 1

        for datum in self.processed_data[start_index:end_index]:
            linear_spring_force = -self.fitted_K * (np.array([datum["q"]["knee_joint_left"], 
                                    datum["q"]["knee_joint_right"], 
                                    datum["q"]["ankle_spring_joint_left"], 
                                    datum["q"]["ankle_spring_joint_right"]]) - self.fitted_offset)
            residuals.append(np.array([datum["spring_force_of_best_spring_model"]["knee_joint_left"],
                        datum["spring_force_of_best_spring_model"]["knee_joint_right"],
                        datum["spring_force_of_best_spring_model"]["ankle_spring_joint_left"],
                        datum["spring_force_of_best_spring_model"]["ankle_spring_joint_right"]]) - linear_spring_force)
                    
        residuals = np.array(residuals)

        return residuals

    def calc_residuals_info_at_given_period(self, start_time, end_time, joints_name, residual_name="residual_for_best_spring_model", is_show_freq_plot=False, is_show_numerical_value=False):
        
        residuals = {}

        act_start_time = None
        act_end_time = None

        if joints_name is None:
            joints_name = ["hip_roll_leftdot", "hip_roll_rightdot", "hip_pitch_leftdot", "hip_pitch_rightdot", "hip_yaw_leftdot", "hip_yaw_rightdot",
                        "knee_leftdot", "knee_rightdot", "knee_joint_leftdot", "knee_joint_rightdot", "ankle_joint_leftdot", "ankle_joint_rightdot"]

        start_index = 0
        while self.processed_data[start_index]["t"] < start_time:
            start_index += 1
        end_index = len(self.processed_data)-1
        while self.processed_data[end_index]["t"] > end_time:
            end_index -= 1

        for datum in self.processed_data[start_index:end_index]:
            if datum['t'] < start_time or datum['t'] > end_time:
                continue
            if act_start_time is None or datum["t"] < act_start_time:
                act_start_time = datum["t"]
            if act_end_time is None or datum["t"] > act_end_time:
                act_end_time = datum["t"]
            
            single_residual_info = datum[residual_name]
            for joint_name in single_residual_info:
                if joint_name not in joints_name:
                    continue
                if joint_name in residuals:
                    residuals[joint_name].append(single_residual_info[joint_name])
                else:
                    residuals[joint_name] = [single_residual_info[joint_name]]
        
        for joint_name in residuals:
            residuals[joint_name] = np.array(residuals[joint_name])
        
        if is_show_numerical_value:
            print("time period:{} to {}".format(act_start_time, act_end_time))
            print(f"{'joint_name':<28} {'mean absolute':>13} {'mean':>10} {'max abs':>10} {'freq':>10} {'mag':>10}")
            for joint_name in residuals:
                xf, freq = get_freq_domain(residuals[joint_name])
                print(f"{joint_name:<28} {np.mean(np.abs(residuals[joint_name])):13.2f} {np.mean(residuals[joint_name]):10.2f} {np.max(np.abs(residuals[joint_name])):10.2f} {freq[np.argmax(xf[1:])+1]:10.2f} {np.max(xf[1:]):10.2f}" )

        if is_show_freq_plot:
            for joint_name in residuals:
                xf, freq = get_freq_domain(residuals[joint_name])
                plt.figure()
                plt.plot(freq, xf)
                plt.xlabel("freq")
                plt.ylabel("mag")
                plt.title(joint_name)
            plt.show()

        return residuals

    def process_data(self):
        processed_data = []
        # Load Data
        n = len(self.original_data)
        t = [datum['t'] for datum in self.original_data]; q = [datum['q'] for datum in self.original_data]
        v = [datum['v'] for datum in self.original_data]; v_dot = [datum['v_dot'] for datum in self.original_data]
        v_dot_gt = [datum['v_dot_gt'] for datum in self.original_data]; is_contact = [datum["is_contact"] for datum in self.original_data]
        J_s = [datum["J_s"] for datum in self.original_data]; spring_force = [datum["spring_force"] for datum in self.original_data]
        u = [datum['u'] for datum in self.original_data]

        # Make numpy array
        t = np.array(t); q = np.array(q); v = np.array(v); v_dot = np.array(v_dot); v_dot_gt = np.array(v_dot_gt); is_contact = np.array(is_contact)
        u = np.array(u)

        # Calculate residual for a special spring model 
        residual_specify_spring_model = get_residual(v_dot_gt, v_dot)

        # Calculate residual for best spring 
        residual_of_best_spring_model, v_dot_of_best_spring_model, best_spring_forces = self.fit_best_spring()

        # Calculate fitted linear spring model
        self.fitted_K, self.fitted_offset = self.calc_spring_constant(q, best_spring_forces)

        for i in range(n):
            datum = {}; datum["t"] = t[i]; datum["q"] = self.construct_dict_for_q_shape_value(q[i]); 
            datum["v"] = self.construct_dict_for_v_shape_value(v[i]); datum["v_dot_gt"] = self.construct_dict_for_v_shape_value(v_dot_gt[i])
            datum["u"] = self.construct_dict_for_u_shape_value(u[i])
            datum["residual_for_changed_stiffness_spring_model"] = self.construct_dict_for_v_shape_value(residual_specify_spring_model[i])
            datum["v_dot_best_spring_model"] = self.construct_dict_for_v_shape_value(v_dot_of_best_spring_model[i])
            datum["v_dot_changed_stiffness_spring_model"] = self.construct_dict_for_v_shape_value(v_dot[i])
            datum["residual_for_best_spring_model"] = self.construct_dict_for_v_shape_value(residual_of_best_spring_model[i])
            datum["is_contact"]={}; datum["is_contact"].update(self.construct_dict_for_left(is_contact[i,0])); datum["is_contact"].update(self.construct_dict_for_right(is_contact[i,1]))
            datum["spring_force_of_changed_spring_model"] = self.construct_dict_for_spring_force_value(J_s[i] @ spring_force[i])
            datum["spring_force_of_best_spring_model"] = self.construct_dict_for_spring_force_value(best_spring_forces[i])

            processed_data.append(datum)

        self.processed_data = processed_data

        return processed_data
            