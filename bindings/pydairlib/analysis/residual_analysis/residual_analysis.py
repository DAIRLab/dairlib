import argparse
import parser
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
from plot_utlis import *
from data_processor import DataProcessor
from log_processor import LogProcessor

class CaaiseSystemTest():
    def __init__(self, path, is_wandb, start_time, duration, wandb_name): 
        
        self.cassie = CassieSystem()

        self.is_wandb = is_wandb

        if self.is_wandb:
            self.plotViewlizer = PlotViewlizer(wandb_name)

        self.start_time = start_time
        self.duration = duration

        self.data_processor = DataProcessor()
        self.log_processor = LogProcessor(self.start_time, self.duration)
        
        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()

        self.data_processor.update_pos_map(self.pos_map, self.pos_map_inverse)
        self.data_processor.update_vel_map(self.vel_map, self.vel_map_inverse)
        self.data_processor.update_act_map(self.act_map, self.act_map_inverse)
        
        self.log_processor.update_pos_map(self.pos_map, self.pos_map_inverse)
        self.log_processor.update_vel_map(self.vel_map, self.vel_map_inverse)
        self.log_processor.update_act_map(self.act_map, self.act_map_inverse)

        self.log_processor.update_log_path(path)

    def load_log_raw_data(self):
        self.raw_data = self.log_processor.process_log()
        self.actual_start_time = max(self.raw_data["states_info"]["t_x"][0], self.raw_data["contact_info"]["t_contact"][0])
        self.actual_end_time = min(self.raw_data["states_info"]["t_x"][-1], self.raw_data["contact_info"]["t_contact"][-1])
        print("Start timestamp:", self.actual_start_time)
        print("End timestamp:", self.actual_end_time)

    def main(self, frame_num):
        self.load_log_raw_data()

        processed_data = process_raw_data(self.raw_data)

        t = processed_data["t"]; q = processed_data["q"]; v = processed_data["v"]; v_dot_gt = processed_data["v_dot"]; 
        u = processed_data["u"]; is_contact = processed_data["is_contact"]
        self.start_time = t[0]; self.end_time = t[-1]

        print("Begin calculate v dot.")
        for i in tqdm.tqdm(range(0, t.shape[0], frame_num)):
            v_dot, lambda_c, lambda_h = self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], spring_mode="changed_stiffness")
        print("Finish calculate v dot.")

        print("Begin calculate additional infos.")
        self.data_processor.get_data(self.cassie.intermediate_variables)
        self.data_processor.process_data()
        print("Finish Calculate additional infos.")
        
        if self.is_wandb:
            print("Begin update data for plots")
            for processed_datum in self.data_processor.processed_data:
                self.plotViewlizer.add_info(processed_datum)
            print("Finish update data for plots")

        print("Begin making plots")
        
        # Use self.vel_map.keys() for all joints
        plot_list_for_v_dot = ["hip_roll_leftdot", "hip_yaw_leftdot", "hip_pitch_leftdot", "knee_leftdot", "knee_joint_leftdot", "ankle_joint_leftdot"]
        # Use self.act_map.keys() for all motors
        plot_list_for_u = ["hip_roll_left_motor", "hip_roll_right_motor"]

        for joint_name in plot_list_for_v_dot:
            plot_joint_residuals_vs_time(self.data_processor.processed_data, self.actual_start_time, self.actual_end_time, joint_name)
        
        plot_joint_effort_vs_time(self.data_processor.processed_data, self.actual_start_time, self.actual_end_time, plot_list_for_u)
        
        plot_spring_force_vs_time(self.data_processor.processed_data, self.actual_start_time, self.actual_end_time)
        plt.show()
        print("Finish making plots")
        
        import pdb; pdb.set_trace()

    def calc_mean_residuals_info_at_given_period(self, start_time, end_time, is_show_freq_plot=False,joint_name=None, residual_name="residual_for_best_spring_model"):
        self.data_processor.calc_residuals_info_at_given_period(start_time, end_time, joint_name, residual_name, is_show_freq_plot=is_show_freq_plot)

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name="residual_for_best_spring_model"):
        self.data_processor.show_hip_residuals_given_time_period(start_time, end_time, residual_name)

def main():
    path = sys.argv[1]
    sys.argv.remove(sys.argv[1])

    parser = argparse.ArgumentParser()
    parser.add_argument('--start_time', type=float, default=0)
    parser.add_argument('--duration', type=float, default=2)
    parser.add_argument('--is_wandb', action="store_true")
    parser.add_argument('--frame_num', type=int, default=10)
    parser.add_argument('--wandb_name', type=str, default="residual_analysis")
    args = parser.parse_args()

    simulationDataTester = CaaiseSystemTest(path, args.is_wandb, args.start_time, args.duration, args.wandb_name)
    simulationDataTester.main(args.frame_num)

if __name__ == "__main__":
    main()