import cvxpy as cp
from matplotlib import patches
import tqdm
import matplotlib.pyplot as plt
from utils import *
from scipy.spatial.transform import Rotation
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

    def fit_spring_damper_model_of_hip_between_main_body(self, start_time, end_time):

        n = len(self.processed_data)
        t = []
        state_dots = []
        state_s = []

        for datum in self.processed_data:
            if not (datum['t'] >= start_time and datum['t'] <= end_time):
                continue
            t.append(datum['t'])
            state_dots.append([datum["v"]["hip_roll_leftdot"], datum["v"]["base_wx"], datum["v"]["hip_roll_rightdot"],
                        datum["v_dot_gt"]["hip_roll_leftdot"] - datum["v_dot_best_spring_model"]["hip_roll_leftdot"], 
                        datum["v_dot_gt"]["base_wx"] - datum["v_dot_best_spring_model"]["base_wx"], 
                        datum["v_dot_gt"]["hip_roll_rightdot"] - datum["v_dot_best_spring_model"]["hip_roll_rightdot"]])
            rot = Rotation.from_quat( np.array([datum["q"]["base_qx"], datum["q"]["base_qy"], datum["q"]["base_qz"], datum["q"]["base_qw"]]))
            state_s.append([datum["q"]["hip_roll_left"], rot.as_rotvec()[0], datum["q"]["hip_roll_right"],
                    datum["v"]["hip_roll_leftdot"], datum["v"]["base_wx"], datum["v"]["hip_roll_rightdot"]])

        state_s = np.array(state_s)
        state_dots = np.array(state_dots)

        print("Begin fit a spring damper model")

        A = cp.Variable((6,6))
        residuals = cp.Variable(6*n)
        """
        A = [0,0,0,1,0,0,
            0,0,0,0,1,0,
            -k1/m1,k1/m1,0,,-c1/m1,c1/m1,0,
            k1/m2,-(k1+k2)/m2,k2/m2,c1/m2,-(c1+c2)/m2,c2/m2,
            0,k2/m3,-k2/m3,0,c3/m3,-c2/m3
        ]
        """
        constraints = [A[:3]==np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]),
                        A[3,2]==0, A[3,5]==0, A[5,0]==0, A[5,3]==0,
                        A[3,0]+A[3,1]==0, A[3,3]+A[3,4]==0,
                        A[4,0]+A[4,1]+A[4,2] == 0, A[4,3]+A[4,4]+A[4,5]==0,
                        A[5,1] + A[5,2] == 0, A[5,4] + A[5,5] == 0]
        
        for i in range(n):
            constraints.append(residuals[i*6:(i+1)*6] == state_dots[i] - A @ state_s[i])

        obj = cp.Minimize(cp.norm2(residuals))
        
        prob = cp.Problem(obj, constraints)
        prob.solve()

        state_dots_est = []

        for i in range(n):
            state_dots_est.append(A.value @ state_s[i])

        state_dots_est = np.array(state_dots_est)

        plt.figure()
        plt.plot(t, state_dots_est[:,0], label='est')
        plt.plot(t, state_dots[:,0], label='gt')
        plt.legend()
        plt.title("left_hip_roll_v")

        plt.figure()
        plt.plot(t, state_dots_est[:,1], label='est')
        plt.plot(t, state_dots[:,1], label='gt')
        plt.legend()
        plt.title("base_wx")

        plt.figure()
        plt.plot(t, state_dots_est[:,2], label='est')
        plt.plot(t, state_dots[:,2], label='gt')
        plt.legend()
        plt.title("right_hip_roll_v")

        plt.figure()
        plt.plot(t, state_dots_est[:,3], label='est')
        plt.plot(t, state_dots[:,3], label='gt')
        plt.legend()
        plt.title("left_hip_roll_v_dot")

        plt.figure()
        plt.plot(t, state_dots_est[:,4], label='est')
        plt.plot(t, state_dots[:,4], label='gt')
        plt.legend()
        plt.title("base_wx_v_dot")

        plt.figure()
        plt.plot(t, state_dots_est[:,5], label='est')
        plt.plot(t, state_dots[:,5], label='gt')
        plt.legend()
        plt.title("right_hip_v_dot")

        plt.show()

        print("Finish fit a spring damper model")

    def fit_spring_damper_model_of_hip_as_beam(self, start_time, end_time, selected_joint="hip_roll_leftdot"):
        """
            Assuming there are spring and damping couple with hip roll, this function is going to how it may look like.
            
            Note, becasue there are too many freedom to choose K, C and mass for the system, the result can mean nothing.
        """

        # initial guess of spring stiffness and damping
        Ks = [10000]
        Cs = [1]
        # max iteration
        max_iter = 5

        tau = []
        t = []

        n = len(self.original_data)

        for i in range(n):
            datum_ori = self.original_data[i]
            datum_processed = self.processed_data[i]

            v_dot_best_spring = [datum_processed["v_dot_best_spring_model"][x] for x in datum_processed["v_dot_best_spring_model"]]

            if not (datum_ori['t'] >= start_time and datum_ori['t'] <= end_time):
                continue
            
            # TODO alternative just choose the residual corresponding the acc?

            tau.append( (datum_ori['M'] @ (datum_ori['v_dot_gt'] - v_dot_best_spring))[self.vel_map[selected_joint]])
            t.append(datum_ori['t'])

        tau = np.array(tau)
        t = np.array(t)

        q_init = np.zeros(n)
        v_init = np.zeros(n)
        qs = [q_init]
        vs = [v_init]

        print("Begin fit spring damping model. Step of signal:{}".format(n))
        
        for i in tqdm.tqdm(range(max_iter)):
            # solve q and v
            q_cp = cp.Variable(n)
            v_cp = cp.Variable(n)
            
            cost = cp.norm(tau + Ks[-1] * q_cp + Cs[-1] * v_cp)
            obj = cp.Minimize(cost)
            
            constraints = []
            for i in range(1,n):
                constraints.append(q_cp[i] == q_cp[i-1] + (t[i] - t[i-1])*v_cp)
            
            prob = cp.Problem(obj, constraints)
            q_cp.value = qs[-1]
            v_cp.value = vs[-1]
            
            prob.solve(warm_start = True)
            
            qs.append(q_cp.value)
            vs.append(v_cp.value)

            # solve K and C
            A = np.hstack((qs[-1].reshape(-1,1), vs[-1].reshape(-1,1)))
            sol = -np.linalg.inv(A.T @ A) @ A.T @ tau            
            Ks.append(sol[0])
            Cs.append(sol[1])
        
        print("Finish fit spring damping model.")
        
        plt.figure()
        plt.plot(Ks, label="K")
        plt.plot(Cs, label="C")
        plt.legend()
        
        for i in range(max_iter):
            plt.figure()
            plt.plot(t, -Ks[i] * qs[i+1] - Cs[i] * vs[i+1], label="Spring and Damping force")
            plt.plot(t, tau, label="Virtual Torque")
            plt.legend()

            plt.plot(t, qs[i+1], label="q")
            plt.plot(t, vs[i+1], label="v")
            plt.legend()
        
        plt.show()

    def calc_spring_constant(self, q, best_spring_forces):
        
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

        print(f"{'Joint name':>15} {'K':>10} {'Offset':>10}")

        print(f"{'Knee left':<15} {knee_left_sol[0]:>10.2f} {knee_left_sol[1]:>10.2f}")
        print(f"{'Knee right':<15} {knee_right_sol[0]:>10.2f} {knee_right_sol[1]:>10.2f}")
        print(f"{'Ankle left':<15} {ankle_left_sol[0]:>10.2f} {ankle_left_sol[1]:>10.2f}")
        print(f"{'Ankle left':<15} {ankle_right_sol[0]:>10.2f} {ankle_right_sol[1]:>10.2f}")

        import pdb; pdb.set_trace()

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name):

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
    
        for datum in self.processed_data:
            if not (datum['t'] >= start_time and datum['t'] <= end_time):
                continue
            t.append(datum['t'])
            single_residual_info = datum[residual_name]
            for key in single_residual_info:
                if key not in threshholds:
                    continue
                if key in residuals:
                    residuals[key].append(single_residual_info[key])
                else:
                    residuals[key] = [single_residual_info[key]]

        t = np.array(t)

        max_value = None
        min_value = None

        plt.figure()

        for key in residuals:
            residuals[key] = np.array(residuals[key])
            if max_value is None or np.max(residuals[key]) > max_value:
                max_value = np.max(residuals[key])
            if min_value is None or np.min(residuals[key]) < min_value:
                min_value = np.min(residuals[key])
            plt.plot(t, residuals[key], label=key, color=color_corresponding[key])

        rects = []

        for key in residuals:
            indices = np.argwhere(np.abs(residuals[key])>threshholds[key])
            if indices.shape[0] == 0:
                continue
            start_index = indices[0]
            end_index = indices[-1]
            rect = patches.Rectangle((t[start_index], min_value), t[end_index] - t[start_index], max_value-min_value, color=color_corresponding[key], alpha=0.8)
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
        print(f"{'joint_name':<28} {'mean absolute':>13} {'mean':>10} {'max abs':>10} {'freq':>10} {'mag':>10}")
        for key in residuals:
            residuals[key] = np.array(residuals[key])
            xf, freq = get_freq_domain(residuals[key])
            print(f"{key:<28} {np.mean(np.abs(residuals[key])):13.2f} {np.mean(residuals[key]):10.2f} {np.max(np.abs(residuals[key])):10.2f} {freq[np.argmax(xf[1:])+1]:10.2f} {np.max(xf[1:]):10.2f}" )
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
            