import cvxpy as cp
import tqdm
import matplotlib.pyplot as plt
from utils import *
class DataProcessor():

    def __init__(self):
        pass

    def update_pos_map(self, pos_map, pos_map_inverse):
        self.pos_map = pos_map
        self.pos_map_inverse = pos_map_inverse

    def update_vel_map(self, vel_map, vel_map_inverse):
        self.vel_map = vel_map
        self.vel_map_inverse = vel_map_inverse
    
    def update_act_map(self, act_map, act_map_inverse):
        self.act_map = act_map
        self.act_map_inverse = act_map_inverse

    def get_data(self, data):
        self.original_data = data

    def construct_dict_for_v_shape_value(self, v):
        dicts = {}
        for i in range(22):
            dicts[self.vel_map_inverse[i]] = v[i]
        return dicts

    def construct_dict_for_q_shape_value(self, q):
        dicts = {}
        for i in range(23):
            dicts[self.pos_map_inverse[i]] = q[i]
        return dicts

    def construct_dict_for_spring_force_value(self, spring_force):
        order_dict = {0:"knee_joint_left", 1:"knee_joint_right", 2:"ankle_spring_joint_left", 3:"ankle_spring_joint_right"}
        dicts = {}
        for key in order_dict:
            dicts[order_dict[key]] = spring_force[key]
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
        lambda_h_solved = []
        lambda_c_solved = []
        v_dot_gts = []
        
        selected_joints = ["hip_roll_leftdot", "hip_roll_rightdot", "hip_pitch_leftdot", "hip_pitch_rightdot", "hip_yaw_leftdot", "hip_yaw_rightdot",
                        "knee_leftdot", "knee_rightdot", "knee_joint_leftdot", "knee_joint_rightdot", "ankle_joint_leftdot", "ankle_joint_rightdot"]
        selection_matrix = np.zeros((22,22))
        for selected_joint in selected_joints:
            selection_matrix[self.vel_map[selected_joint], self.vel_map[selected_joint]] = 1

        for i in tqdm.tqdm(range(len(self.original_data))):

            spring_force = cp.Variable(4)
            v_dot = cp.Variable(22)

            datum = self.original_data[i]
            v_dot_gt = datum["v_dot_gt"]; num_of_contact = datum["num_contact_unknown"]; A = datum["A"]; B = datum["B"]
            J_c_active_dot_v = datum["J_c_active_dot_v"]; J_h_active_dot_v = datum["J_h_dot_v"]; gravity = datum["gravity"]
            bias = datum["bias"]; damping_force = datum["damping_force"]; J_s = datum["J_s"]; u = datum["u"]
            A_inverse = np.linalg.inv(A); v_dot_gts.append(v_dot_gt)

            if num_of_contact > 0:
                b = cp.hstack((
                    B @ u + gravity - bias + damping_force + J_s.T @ spring_force,
                    -J_h_active_dot_v,
                    -J_c_active_dot_v,
                ))
            else:
                b = cp.hstack((
                    B @ u + gravity - bias + damping_force + J_s.T @ spring_force,
                    -J_h_active_dot_v,            
                ))

            obj = cp.Minimize( cp.norm(selection_matrix@(v_dot_gt - v_dot)))
            constraints = [(A_inverse @ b)[:22] == v_dot]

            prob = cp.Problem(obj, constraints)
            prob.solve()
            v_dot_of_best_spring_model.append(v_dot.value)
            best_spring_forces.append(spring_force.value)

        best_spring_forces = np.array(best_spring_forces)
        v_dot_gts = np.array(v_dot_gts)
        v_dot_of_best_spring_model = np.array(v_dot_of_best_spring_model)
        residual = get_residual(v_dot_gts, v_dot_of_best_spring_model)

        print("Finish fitting the best spring model.")

        return residual, v_dot_of_best_spring_model, best_spring_forces

    def calc_residuals_info_at_given_period(self, start_time, end_time, joints_name, residual_name, is_show_freq_plot):
        
        residuals = {}

        act_start_time = None
        act_end_time = None

        if joints_name is None:
            joints_name = ["hip_roll_leftdot", "hip_roll_rightdot", "hip_pitch_leftdot", "hip_pitch_rightdot", "hip_yaw_leftdot", "hip_yaw_rightdot",
                        "knee_leftdot", "knee_rightdot", "knee_joint_leftdot", "knee_joint_rightdot", "ankle_joint_leftdot", "ankle_joint_rightdot"]

        for datum in self.processed_data:
            if not (datum['t'] >= start_time and datum['t'] <= end_time):
                continue
            if act_start_time is None or datum["t"] < act_start_time:
                act_start_time = datum["t"]
            if act_end_time is None or datum["t"] > act_end_time:
                act_end_time = datum["t"]
            
            single_residual_info = datum[residual_name]
            for key in single_residual_info:
                if key not in joints_name:
                    continue
                if key in residuals:
                    residuals[key].append(single_residual_info[key])
                else:
                    residuals[key] = [single_residual_info[key]]
        
        print("time period:{} to {}".format(act_start_time, act_end_time))
        print(f"{'joint_name':<28} {'mean absolute':>13} {'mean':>10} {'max abs':>10} {'freq':>10} {'mag:':>10}")
        for key in residuals:
            residuals[key] = np.array(residuals[key])
            xf, freq = get_freq_domain(residuals[key])
            print(f"{key:<28} {np.mean(np.abs(residuals[key])):13.2f} {np.mean(residuals[key]):7.2f} {np.max(np.abs(residuals[key])):10.2f} {freq[np.argmax(xf[1:])]:10.2f} {np.max(xf[1:]):10.2f}" )
            if is_show_freq_plot:
                plt.figure()
                plt.plot(freq, xf)
                plt.xlabel("freq")
                plt.ylabel("mag")
                plt.title(key)

        if is_show_freq_plot:
            plt.show()

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
            