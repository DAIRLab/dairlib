from math import floor
import sys
import time
import os
import tqdm
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import scipy.linalg
import scipy.io
import cvxpy as cp
from cassie_model import CassieSystem

class CaaiseSystemTest():
    def __init__(self, data_path, start_time, end_time):
        
        self.data_path = data_path
        self.start_time = start_time
        self.end_time = end_time
        
        self.cassie = CassieSystem()
        
        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()
        self.lambda_c_map, self.lambda_c_map_inverse = self.make_lambda_c_map(is_four_point=False)

    def make_lambda_c_map(self, is_four_point=True):
        
        if is_four_point:
            lambda_c_map = {"left_front_x":0, "left_front_y":1, "left_front_z":2,
                            "left_rear_x":3, "left_rear_y":4, "left_rear_z":5,
                            "right_front_x":6, "right_front_y":7, "right_front_z":8,
                            "right_rear_x":9, "right_rear_y":10, "right_rear_z":11
                        }
        else:
            lambda_c_map = {"left_x":0, "left_y":1, "left_z":2,
                            "right_x":3, "right_y":4, "right_z":5,}
        lambda_c_map_inverse = {value:key for (key,value) in lambda_c_map.items()}
        
        return lambda_c_map, lambda_c_map_inverse

    def get_residual(self, vdot_gt, vdot_est, cutting_f = 100):
        residuals = vdot_gt - vdot_est
        residuals_smoothed = self.first_order_filter(residuals, cutting_f=cutting_f)
        return residuals_smoothed

    def first_order_filter(self, orginal_signals, cutting_f=100, Ts=0.0005):
        a = np.exp(-2*np.pi*cutting_f*Ts)
        filtered_signals = [orginal_signals[0]]
        for i in range(1, orginal_signals.shape[0]):
            filtered_signals.append(a * filtered_signals[-1] + (1-a) * orginal_signals[i])
        filtered_signals = np.array(filtered_signals)
        return filtered_signals 

    def hardware_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = self.process_hardware_data(raw_data, self.start_time, self.end_time)
        t = process_data["t"]; q = process_data["q"]; v = process_data["v"]; v_dot_gt = process_data["v_dot"]; 
        u = process_data["u"]; is_contact = process_data["is_contact"]; v_dot_osc = process_data["v_dot_osc"]; u_osc = process_data["u_osc"]
        K = process_data["spring_stiffness"]; C = process_data["damping_ratio"]

        # K[self.vel_map["knee_joint_leftdot"], self.pos_map["knee_joint_left"]] = 1531.28512245
        # K[self.vel_map["knee_joint_rightdot"], self.pos_map["knee_joint_right"]] = 1754.16568063
        # K[self.vel_map["ankle_spring_joint_leftdot"], self.pos_map["ankle_spring_joint_left"]] = 923.00560595
        # K[self.vel_map["ankle_spring_joint_rightdot"], self.pos_map["ankle_spring_joint_right"]] = 986.8367351

        offset = np.zeros(23)
        # offset[self.pos_map["knee_joint_left"]] = -7.38443491/1531.28512245
        # offset[self.pos_map["knee_joint_right"]] = -9.17183419/1754.16568063
        # offset[self.pos_map["ankle_spring_joint_left"]] = -15.89566176/923.00560595
        # offset[self.pos_map["ankle_spring_joint_right"]] = -17.37326985/986.8367351

        # offset[self.pos_map["knee_joint_left"]] = 0.000; offset[self.pos_map["knee_joint_right"]] = offset[self.pos_map["knee_joint_left"]]
        # K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]] = 1500; K[self.vel_map["knee_joint_rightdot"],self.pos_map["knee_joint_right"]] = K[self.vel_map["knee_joint_leftdot"],self.pos_map["knee_joint_left"]]
        self.cassie.get_spring_stiffness(K); self.cassie.get_damping(C)
        self.cassie.get_spring_offset(offset)

        t_list = []
        v_dot_est_fixed_spring_list = []
        v_dot_est_constant_spring_stiffness_list = []
        v_dot_est_changed_spring_stiffness_list = []

        v_dot_gt_list = []
        v_dot_osc_list = []

        # while True:
        #     for i in range(t.shape[0]):
        #         self.cassie.drawPose(q[i,:])
        #         time.sleep(0.1)
        #     import pdb; pdb.set_trace()

        plt.plot(t, is_contact[:,0], label = 'left')
        plt.plot(t, is_contact[:,1], label = 'right')
        plt.legend()

        for i in range(0, t.shape[0], ):
            t_list.append(t[i])

            v_dot_est, _, _= self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], spring_mode="fixed_spring")
            v_dot_est_fixed_spring_list.append(v_dot_est)

            self.cassie.get_spring_stiffness(K)
            self.cassie.get_spring_offset(offset)
            v_dot_est, _, _= self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], spring_mode="constant_stiffness")
            v_dot_est_constant_spring_stiffness_list.append(v_dot_est)

            v_dot_est, _, _= self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], spring_mode="changed_stiffness")
            v_dot_est_changed_spring_stiffness_list.append(v_dot_est)

            v_dot_gt_list.append(v_dot_gt[i,:])
            v_dot_osc_list.append(v_dot_osc[i,:])
        
        t_list = np.array(t_list)
        v_dot_est_fixed_spring_list = np.array(v_dot_est_fixed_spring_list)
        v_dot_est_constant_spring_stiffness_list = np.array(v_dot_est_constant_spring_stiffness_list)
        v_dot_est_changed_spring_stiffness_list = np.array(v_dot_est_changed_spring_stiffness_list)
        v_dot_gt_list = np.array(v_dot_gt_list)
        v_dot_osc_list = np.array(v_dot_osc_list)

        fixed_spring_residuals = self.get_residual(v_dot_gt_list, v_dot_est_fixed_spring_list)
        constant_stiffness_residuals = self.get_residual(v_dot_gt_list, v_dot_est_constant_spring_stiffness_list)
        changed_stiffness_residuals = self.get_residual(v_dot_gt_list, v_dot_est_changed_spring_stiffness_list)
        osc_residuals = self.get_residual(v_dot_gt_list, v_dot_osc_list)

        self.cassie.r = np.array(self.cassie.r)
        self.cassie.r_c = np.array(self.cassie.r_c)
        self.cassie.r_d = np.array(self.cassie.r_d)
        self.cassie.r_g = np.array(self.cassie.r_g)
        self.cassie.r_u = np.array(self.cassie.r_u)
        self.cassie.r_s = np.array(self.cassie.r_s)
        self.cassie.r_lc = np.array(self.cassie.r_lc)
        self.cassie.r_lh = np.array(self.cassie.r_lh)

        self.cassie.left_foot_on_world = np.array(self.cassie.left_foot_on_world)

        plt.figure()

        plt.plot(t_list, self.cassie.r[:,])

        interested_index = self.vel_map["hip_pitch_leftdot"]

        plt.figure()
        plt.plot(t_list, self.cassie.r[:,interested_index], label="residual")
        plt.plot(t_list, v_dot_gt_list[:, interested_index], label="gt")
        plt.plot(t_list, self.cassie.r_c[:,interested_index], label="bias term")
        plt.plot(t_list, self.cassie.r_d[:,interested_index], label="damping")
        plt.plot(t_list, self.cassie.r_g[:,interested_index], label="gravity")
        plt.plot(t_list, self.cassie.r_u[:,interested_index], label="u")
        plt.plot(t_list, self.cassie.r_s[:,interested_index], label="spring")
        plt.plot(t_list, self.cassie.r_lc[:,interested_index], label="contact force")
        plt.plot(t_list, self.cassie.r_lh[:,interested_index], label="four bar constraints")
        plt.figure()
        plt.plot(t_list, self.cassie.r[:,interested_index]*0.1, label="residual")
        plt.plot(t_list, v[:,interested_index], label="v")
        plt.plot(t_list, u[:,self.act_map["hip_pitch_left_motor"]], label="u")
        plt.plot(t_list, q[:,self.pos_map["hip_pitch_left"]], label="q")
        plt.legend()
        plt.figure()
        plt.plot(t_list, q[:,self.pos_map["knee_joint_right"]], label = "knee joint right")
        plt.plot(t_list, q[:,self.pos_map["ankle_spring_joint_right"]], label = "ankle joint right")
        plt.plot(t_list, q[:,self.pos_map["knee_joint_left"]], label = "knee joint left")
        plt.plot(t_list, q[:,self.pos_map["ankle_spring_joint_left"]], label = "ankle joint left")
        plt.legend()
        plt.show()

        import pdb; pdb.set_trace()

        for i in range(22):
            plt.cla()
            plt.plot(t_list, fixed_spring_residuals[:,i], label="fix spring:{:.1f}".format(np.mean(np.abs(fixed_spring_residuals[:,i]))))
            plt.plot(t_list, constant_stiffness_residuals[:,i], label="constant stiffness residuals:{:.1f}".format(np.mean(np.abs(constant_stiffness_residuals[:,i]))))
            plt.plot(t_list, changed_stiffness_residuals[:,i], label="changed stiffness residuals:{:.1f}".format(np.mean(np.abs(changed_stiffness_residuals[:,i]))))
            plt.plot(t_list, osc_residuals[:,i], label="osc residuals:{:.1f}".format(np.mean(np.abs(osc_residuals[:,i]))))
            plt.legend()
            plt.title(self.vel_map_inverse[i])
            plt.savefig("bindings/pydairlib/cassie/residual_analysis/hardware_test/v_dot_residual/{}.png".format(self.vel_map_inverse[i]))
        
        for i in range(22):
            plt.cla()
            plt.plot(t_list, v_dot_gt_list[:,i], label='gt')
            plt.plot(t_list, v_dot_est_constant_spring_stiffness_list[:,i], label='constant stiffness')
            plt.plot(t_list, v_dot_est_changed_spring_stiffness_list[:,i], label='changed stiffness')
            plt.plot(t_list, v_dot_est_fixed_spring_list[:,i], label='fixed spring')
            plt.plot(t_list, v_dot_osc_list[:,i], label='osc')
            plt.legend()
            plt.title(self.vel_map_inverse[i])
            plt.savefig("bindings/pydairlib/cassie/residual_analysis/hardware_test/v_dot/{}.png".format(self.vel_map_inverse[i]))
        
        import pdb; pdb.set_trace()

    def simulation_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = self.process_sim_data(raw_data, self.start_time, self.end_time)
        t = process_data["t"]; q = process_data["q"]; v = process_data["v"]; v_dot_gt = process_data["v_dot"]; 
        u = process_data["u"]; lambda_c_gt = process_data["lambda_c"]; lambda_c_position = process_data["lambda_c_position"]
        v_dot_osc = process_data["v_dot_osc"]; #u_osc = process_data["u_osc"]
        damping_ratio = process_data["damping_ratio"]; spring_stiffness = process_data["spring_stiffness"]

        self.cassie.get_damping(damping_ratio)
        self.cassie.get_spring_stiffness(spring_stiffness)

        t_list = []
        v_dot_est_list = []
        lambda_c_est_list = []
        lambda_h_est_list = []
        v_dot_gt_list = []
        lambda_c_gt_list = []
        # v_dot_osc_list = []

        for i in range(t.shape[0]):
            t_list.append(t[i])
            v_dot_est, lambda_c_est, lambda_h_est= self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], lambda_c_gt=lambda_c_gt[i,:], lambda_c_position=lambda_c_position[i,:], v_dot_gt=v_dot_gt[i,:])
            v_dot_est_list.append(v_dot_est)
            lambda_c_est_list.append(lambda_c_est)
            lambda_h_est_list.append(lambda_h_est)
            v_dot_gt_list.append(v_dot_gt[i,:])
            lambda_c_gt_list.append(lambda_c_gt[i,:])
            # v_dot_osc_list.append(v_dot_osc[i,:])

        t_list = np.array(t_list)
        v_dot_est_list = np.array(v_dot_est_list)        
        lambda_c_est_list = np.array(lambda_c_est_list)
        lambda_h_est_list = np.array(lambda_h_est_list)
        lambda_c_gt_list = np.array(lambda_c_gt_list)
        v_dot_gt_list = np.array(v_dot_gt_list)
        # v_dot_osc_list = np.array(v_dot_osc_list)

        # Save v_dot pictures
        for i in range(22):
            plt.cla()
            # plt.plot(t_list, v_dot_osc_list[:,i], 'b', label='osc')
            plt.plot(t_list, v_dot_est_list[:,i], 'r', label='est')
            plt.plot(t_list, v_dot_gt_list[:,i], 'g', label='gt')
            plt.legend()
            plt.title(self.vel_map_inverse[i])
            plt.ylim(-100,100)
            plt.savefig("bindings/pydairlib/cassie/residual_analysis/simulation_test/v_dot/{}.png".format(self.vel_map_inverse[i]))

        # Save residual pictures
        # self.draw_residual(lambda_c_gt[:,2]+lambda_c_gt[:,5]+lambda_c_gt[:,8]+lambda_c_gt[:,11], residual_list, 
        #                     "bindings/pydairlib/cassie/residual_analysis/simulation_test/residuals/total_contact_force_at_z_direction")
        
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

    def draw_residual(self, x_axis, residuals, dir):

        if not os.path.exists(dir):
            os.makedirs(dir)

        for i in range(22):

            n = x_axis.shape[0]
            x_axis_sorted = np.sort(x_axis)
            residuals_sorted = np.sort(residuals[:,i])
            x_min=x_axis_sorted[int(n*0.01)]; x_max=x_axis_sorted[int(n*(1-0.01))]
            y_min=residuals_sorted[int(n*0.01)]; y_max=residuals_sorted[int(n*(1-0.01))]

            corrcoef = np.corrcoef(np.vstack((x_axis, residuals[:,i])))

            A = np.hstack((x_axis[:,None], np.ones((n,1))))
            sol = np.linalg.pinv(A) @ residuals[:,i]

            plt.cla()
            plt.plot(x_axis, residuals[:, i], 'ro', markersize=1)
            plt.plot([x_min, x_max], [x_min*sol[0]+sol[1], x_max*sol[0]+sol[1]], 'b')
            plt.title(self.vel_map_inverse[i] + " corrcoef:" + "{:.2f}".format(corrcoef[1,0]))
            plt.xlim(x_min, x_max)
            plt.ylim(y_min, y_max)
            plt.savefig(dir + "/{}.png".format(self.vel_map_inverse[i]))

    def interpolation(self, t,t1,t2,v1,v2):
        ratio = (t - t1)/(t2-t1)
        v = v1 + ratio * (v2 -v1)
        return v

    def process_hardware_data(self, raw_data, start_time, end_time):
        
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
        
        v_dot = self.first_order_filter(v_dot)

        # processing contact force
        contact_output = raw_data['contact_output']
        t_contact = contact_output['t_contact'][0][0][0]
        start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
        end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
        t_contact = t_contact[start_index: end_index]
        is_contact = contact_output['is_contact'][0][0][start_index:end_index,:]
        is_contact_processed = []
        pointer = 0
        for t in t_robot_output:
            while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
                pointer+=1
            if abs(t_contact[pointer] - t) < abs(t_contact[pointer+1] - t):
                is_contact_processed.append(is_contact[pointer,:])
            else:
                is_contact_processed.append(is_contact[pointer+1,:])
        is_contact_processed = np.array(is_contact_processed)

        # Get the osc_output

        osc_output = raw_data["osc_output"]
        t_osc = osc_output["t_osc"][0][0][0]
        start_index = np.argwhere(t_osc > start_time - 0.01)[0][0]
        end_index = np.argwhere(t_osc < end_time + 0.01)[-1][0]
        t_osc = t_osc[start_index: end_index]
        u_osc = osc_output["u_sol"][0][0][start_index:end_index,:]
        u_osc_processed = []
        v_dot_osc = osc_output["dv_sol"][0][0][start_index:end_index,:]
        v_dot_osc_proccessed = []
        pointer = 0
        for t in t_robot_output:
            while not (t_osc[pointer] <= t and t_osc[pointer+1] >=t):
                pointer+=1
            u_osc_interpolated = self.interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                    u_osc[pointer,:], u_osc[pointer+1, :])
            v_dot_osc_interpolated = self.interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                    v_dot_osc[pointer,:], v_dot_osc[pointer+1, :])
            u_osc_processed.append(u_osc_interpolated)
            v_dot_osc_proccessed.append(v_dot_osc_interpolated)
        u_osc_processed = np.array(u_osc_processed)
        v_dot_osc_proccessed = np.array(v_dot_osc_proccessed)

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
            'is_contact':is_contact_processed,
            'u_osc': u_osc_processed,
            'v_dot_osc': v_dot_osc_proccessed,
            'damping_ratio':damping_ratio,
            'spring_stiffness':spring_stiffness
        }

        return processed_data

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
        
        v_dot = self.first_order_filter(v_dot)

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

        # Get the osc_output

        osc_output = raw_data["osc_output"]
        t_osc = osc_output["t_osc"][0][0][0]
        start_index = np.argwhere(t_osc > start_time - 0.01)[0][0]
        end_index = np.argwhere(t_osc < end_time + 0.01)[-1][0]
        t_osc = t_osc[start_index: end_index]
        u_osc = osc_output["u_sol"][0][0][start_index:end_index,:]
        u_osc_processed = []
        v_dot_osc = osc_output["dv_sol"][0][0][start_index:end_index,:]
        v_dot_osc_proccessed = []
        pointer = 0
        for t in t_robot_output:
            while not (t_osc[pointer] <= t and t_osc[pointer+1] >=t):
                pointer+=1
            u_osc_interpolated = self.interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                    u_osc[pointer,:], u_osc[pointer+1, :])
            v_dot_osc_interpolated = self.interpolation(t, t_osc[pointer], t_osc[pointer+1],
                                                    v_dot_osc[pointer,:], v_dot_osc[pointer+1, :])
            u_osc_processed.append(u_osc_interpolated)
            v_dot_osc_proccessed.append(v_dot_osc_interpolated)
        u_osc_processed = np.array(u_osc_processed)
        v_dot_osc_proccessed = np.array(v_dot_osc_proccessed)

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
            'u_osc': u_osc_processed,
            'v_dot_osc': v_dot_osc_proccessed,
            'damping_ratio':damping_ratio,
            'spring_stiffness':spring_stiffness
        }

        return processed_data

def main():
    simulationDataTester = CaaiseSystemTest(data_path="log/03_15_22/lcmlog-11.mat", start_time=31.1, end_time=31.16)
    simulationDataTester.hardware_test()

if __name__ == "__main__":
    main()