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
    def __init__(self, date, log_num):
        

        data_path = "log/{}/lcmlog-{}.mat".format(date, log_num)

        self.data_path = data_path

        print("data log at:{}".format(self.data_path))
        
        self.cassie = CassieSystem()


        self.plotViewlizer = PlotViewlizer("{}-{}".format(date, log_num))

        self.data_processor = DataProcessor()
        
        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()

        self.data_processor.update_pos_map(self.pos_map, self.pos_map_inverse)
        self.data_processor.update_vel_map(self.vel_map, self.vel_map_inverse)
        self.data_processor.update_act_map(self.act_map, self.act_map_inverse)

    def hardware_test(self):
        raw_data = scipy.io.loadmat(self.data_path)
        process_data = process_hardware_data(raw_data,)
        t = process_data["t"]; q = process_data["q"]; v = process_data["v"]; v_dot_gt = process_data["v_dot"]; 
        u = process_data["u"]; is_contact = process_data["is_contact"]; v_dot_osc = process_data["v_dot_osc"]; u_osc = process_data["u_osc"]
        K = process_data["spring_stiffness"]; C = process_data["damping_ratio"]

        offset = np.zeros(23)

        self.cassie.get_spring_stiffness(K); self.cassie.get_damping(C)
        self.cassie.get_spring_offset(offset)

        plt.plot(t, is_contact[:,0])
        plt.show()
        import pdb; pdb.set_trace()

        print("Begin calculate v dot.")
        for i in tqdm.tqdm(range(0, t.shape[0])):
            v_dot, lambda_c, lambda_h = self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], u_osc=u_osc[i,:], v_dot_osc=v_dot_osc[i,:], spring_mode="changed_stiffness")
        print("Finish calculate v dot.")

        print("Begin calculate additional infos.")
        self.data_processor.get_data(self.cassie.intermediate_variables)
        self.data_processor.process_data()
        print("Finish Calculate additional infos.")
        
        print("Begin update data for plots")
        for processed_datum in self.data_processor.processed_data:
            self.plotViewlizer.add_info(processed_datum)
        print("Finish update data for plots")

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
    simulationDataTester = CaaiseSystemTest(date="03_15_22", log_num=11)
    simulationDataTester.hardware_test()

if __name__ == "__main__":
    main()