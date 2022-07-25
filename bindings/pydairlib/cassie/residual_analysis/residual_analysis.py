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
from utils import *
from plot_utlis import PlotViewlizer
from data_processor import DataProcessor

class CaaiseSystemTest():
    def __init__(self, date, log_num, is_new_plots=False):

        data_path = "log/{}/lcmlog-{}.mat".format(date, log_num)

        self.data_path = data_path

        print("data log at:{}".format(self.data_path))
        
        self.cassie = CassieSystem()

        self.is_new_plots = is_new_plots

        if self.is_new_plots:
            self.plotViewlizer = PlotViewlizer("{}-{}".format(date, log_num))

        self.data_processor = DataProcessor()
        
        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()

        self.data_processor.update_pos_map(self.pos_map, self.pos_map_inverse)
        self.data_processor.update_vel_map(self.vel_map, self.vel_map_inverse)
        self.data_processor.update_act_map(self.act_map, self.act_map_inverse)

    def input_isolation_test(self):
        duration = 100
        dt = 0.0005
        contact_mode = np.array([0,0])
        
        init_q = np.array([0.99544474,  0.01837867, -0.07732829,  0.05265298,  0.99051897,
       -0.25745832,  1.10687565, -0.01257864, -0.060807  ,  0.60521936,
       -1.68468988,  0.04315069,  1.8032732 ,  0.05351326, -1.797297  ,
       -0.06826214, -0.00297592,  0.58877701, -1.74365234,  0.0022177 ,
        1.95459545,  0.01105913, -1.94397616])
        
        init_v = np.array([ 0.84349948, -0.99632606, -0.54623077,  0.08054357, -0.24025121,
                        0.15620078,  0.77324903,  0.11250215,  5.04166746, -5.10034227,
                        2.5170238 ,  2.75815916,  0.        ,  0.48897243, -0.03494408,
                        0.40377441, -2.16895294,  1.43810701,  0.73286361, -1.31574011,
                        0.        ,  1.22875142])

        t = np.arange(duration) * dt
        u = np.zeros((duration, 10))
        u[:, self.act_map["hip_roll_right_motor"]] = 0
        u[:, self.act_map["hip_roll_left_motor"]] = 0

        init_v_dot, _, _ = self.cassie.calc_vdot(t[0], init_q, init_v, u[0], is_contact=contact_mode)

        qs = [init_q]
        vs = [init_v]
        v_dots = [init_v_dot]

        for i in range(1, t.shape[0]):
            q, v = self.cassie.calc_q_and_v(dt, qs[i-1], vs[i-1], v_dots[i-1])
            qs.append(q)
            vs.append(v)
            v_dot,_,_ = self.cassie.calc_vdot(t[i], qs[i], vs[i], u[i], is_contact=contact_mode)
            v_dots.append(v_dot)

        qs = np.array(qs)
        vs = np.array(vs)
        v_dots = np.array(v_dots)

        plt.plot(t, v_dots[:,self.vel_map["hip_roll_leftdot"]], label="hip roll left")
        plt.plot(t, v_dots[:,self.vel_map["hip_roll_rightdot"]], label="hip roll right")
        plt.legend()
        plt.show()

        while True:
            for i in range(t.shape[0]):
                self.cassie.drawPose(qs[i])
                time.sleep(0.1)
            import pdb; pdb.set_trace()

    def hardware_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = process_hardware_data(raw_data,)
        t = process_data["t"]; q = process_data["q"]; v = process_data["v"]; v_dot_gt = process_data["v_dot"]; 
        u = process_data["u"]; is_contact = process_data["is_contact"]; v_dot_osc = process_data["v_dot_osc"]; u_osc = process_data["u_osc"]
        K = process_data["spring_stiffness"]; C = process_data["damping_ratio"]

        offset = np.zeros(23)

        self.cassie.get_spring_stiffness(K); self.cassie.get_damping(C)
        self.cassie.get_spring_offset(offset)

        print("Begin calculate v dot.")
        for i in tqdm.tqdm(range(0, t.shape[0])):
            v_dot, lambda_c, lambda_h = self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], u_osc=u_osc[i,:], v_dot_osc=v_dot_osc[i,:], spring_mode="changed_stiffness")
        print("Finish calculate v dot.")

        print("Begin calculate additional infos.")
        self.data_processor.get_data(self.cassie.intermediate_variables)
        self.data_processor.process_data()
        print("Finish Calculate additional infos.")
        
        if self.is_new_plots:
            print("Begin update data for plots")
            for processed_datum in self.data_processor.processed_data:
                self.plotViewlizer.add_info(processed_datum)
            print("Finish update data for plots")

        import pdb; pdb.set_trace()

    def calc_mean_residuals_info_at_given_period(self, start_time, end_time, is_show_freq_plot=False,joint_name=None, residual_name="residual_for_best_spring_model"):
        self.data_processor.calc_residuals_info_at_given_period(start_time, end_time, joint_name, residual_name, is_show_freq_plot=is_show_freq_plot)

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name="residual_for_best_spring_model"):
        self.data_processor.show_hip_residuals_given_time_period(start_time, end_time, residual_name)

    def simulation_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = process_sim_data(raw_data, self.start_time, self.end_time)
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

def main():
    date = sys.argv[1]
    log_num = sys.argv[2]
    if len(sys.argv) == 4:
        is_new_plots = sys.argv[3]
    else:
        is_new_plots = False

    simulationDataTester = CaaiseSystemTest(date=date, log_num=log_num, is_new_plots=is_new_plots)
    # simulationDataTester.hardware_test()
    simulationDataTester.input_isolation_test()

if __name__ == "__main__":
    main()