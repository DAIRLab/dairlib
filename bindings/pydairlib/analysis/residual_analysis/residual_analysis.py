import argparse
import sys
import tqdm
import numpy as np
import matplotlib.pyplot as plt
from cassie_model import CassieSystem
from utils import *
from plot_utlis import *
from data_processor import DataProcessor
from log_processor import LogProcessor

class ResidualAnalyzer():
    def __init__(self, path, is_wandb, start_time, duration, wandb_name): 
        
        self.cassie = CassieSystem()

        self.is_wandb = is_wandb

        if self.is_wandb:
            self.wandb_processor = WandbProcessor(wandb_name)

        self.start_time = start_time
        self.duration = duration

        self.data_processor = DataProcessor()
        self.log_processor = LogProcessor(self.start_time, self.duration)

        self.data_processor.set_num_of_pos(self.cassie.num_pos)
        self.data_processor.set_num_of_vel(self.cassie.num_vel)

        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()

        self.data_processor.set_pos_map(self.pos_map, self.pos_map_inverse)
        self.data_processor.set_vel_map(self.vel_map, self.vel_map_inverse)
        self.data_processor.set_act_map(self.act_map, self.act_map_inverse)
        
        self.log_processor.set_pos_map(self.pos_map, self.pos_map_inverse)
        self.log_processor.set_vel_map(self.vel_map, self.vel_map_inverse)
        self.log_processor.set_act_map(self.act_map, self.act_map_inverse)

        self.log_processor.set_log_path(path)

    def load_log_raw_data(self):
        self.raw_data = self.log_processor.process_log()
        self.actual_start_time = max(self.raw_data["states_info"]["t_x"][0], self.raw_data["contact_info"]["t_contact"][0])
        self.actual_end_time = min(self.raw_data["states_info"]["t_x"][-1], self.raw_data["contact_info"]["t_contact"][-1])
        print("Start timestamp:", self.actual_start_time)
        print("End timestamp:", self.actual_end_time)

    def main(self, frame_num):
        self.load_log_raw_data()

        processed_data = self.process_raw_data(self.raw_data)

        t = processed_data["t"]; q = processed_data["q"]; v = processed_data["v"]; v_dot_gt = processed_data["v_dot"]; 
        u = processed_data["u"]; is_contact = processed_data["is_contact"]
        self.start_time = t[0]; self.end_time = t[-1]

        print("Begin calculate v dot.")
        for i in tqdm.trange(0, t.shape[0], frame_num):
            v_dot, lambda_c, lambda_h = self.cassie.calc_vdot(t[i], q[i,:], v[i,:], u[i,:], is_contact=is_contact[i,:], v_dot_gt=v_dot_gt[i,:], spring_mode="changed_stiffness")
        print("Finish calculate v dot.")

        print("Begin calculate additional infos.")
        self.data_processor.set_data(self.cassie.intermediate_variables_list)
        self.data_processor.process_data()
        print("Finish Calculate additional infos.")

        self.data_processor.fit_spring_damper_model_of_hip_between_main_body(self.start_time,self.end_time,is_show=True)
        
        if self.is_wandb:
            print("Begin update data for plots")
            for processed_datum in self.data_processor.processed_data:
                self.wandb_processor.add_info(processed_datum)
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
        
    def calc_mean_residuals_info_at_given_period(self, start_time, end_time, is_show_freq_plot=False,joint_name=None, residual_name="residual_for_best_spring_model"):
        self.data_processor.calc_residuals_info_at_given_period(start_time, end_time, joint_name, residual_name, is_show_freq_plot=is_show_freq_plot)

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name="residual_for_best_spring_model"):
        self.data_processor.show_hip_residuals_given_time_period(start_time, end_time, residual_name)

    def process_raw_data(self, raw_data):
    
        print("Begin process the raw data.")
        
        # cut down some data in the begin and at the end to make sure processings like finite difference, interpolation won't have issue
        start_time = max(raw_data["states_info"]["t_x"][20], raw_data["contact_info"]["t_contact"][20])
        end_time = min(raw_data["states_info"]["t_x"][-20], raw_data["contact_info"]["t_contact"][-20])

        # processing robot output
        states_info = raw_data['states_info']
        t_states_info = states_info["t_x"]

        start_index = np.argwhere(t_states_info > start_time)[0][0]
        end_index = np.argwhere(t_states_info < end_time)[-1][0]
        t_states_info = t_states_info[start_index:end_index]
        q = states_info['q'][start_index:end_index,:]
        v = states_info['v'][start_index:end_index,:]
        u = states_info['u'][start_index:end_index,:]
        
        smooth_window = 10
        # obtained v_dot by finite diff
        # v_dot[i] = (v[i+smooth_window] - v[i-smooth_window])/(t[i+smooth_window] - t[i-smooth_window]) 
        v_dot = (states_info['v'][start_index+smooth_window:end_index+smooth_window,:] - states_info['v'][start_index-smooth_window:end_index-smooth_window,:])\
            /(states_info["t_x"][start_index+smooth_window:end_index+smooth_window] - states_info["t_x"][start_index-smooth_window:end_index-smooth_window])[:,None]
        
        v_dot = first_order_filter(v_dot)

        # processing contact info
        contact_output = raw_data['contact_info']
        t_contact = contact_output['t_contact']
        start_index = np.argwhere(t_contact > start_time - 0.01)[0][0] # make sure contact force period range cover the output states
        end_index = np.argwhere(t_contact < end_time + 0.01)[-1][0]
        t_contact = t_contact[start_index: end_index]
        is_contact = contact_output['is_contact'][start_index:end_index,:]
        is_contact_processed = []
        pointer = 0
        for t in t_states_info:
            while not (t_contact[pointer] <= t and t_contact[pointer+1] >=t):
                pointer+=1
            if abs(t_contact[pointer] - t) < abs(t_contact[pointer+1] - t):
                is_contact_processed.append(is_contact[pointer,:])
            else:
                is_contact_processed.append(is_contact[pointer+1,:])
        is_contact_processed = np.array(is_contact_processed)

        processed_data = {
            't':t_states_info,
            'q':q,
            'v':v,
            'v_dot':v_dot,
            'u':u,
            'is_contact':is_contact_processed,
        }

        print("Finish process data.")

        return processed_data

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

    simulationDataTester = ResidualAnalyzer(path, args.is_wandb, args.start_time, args.duration, args.wandb_name)
    simulationDataTester.main(args.frame_num)

if __name__ == "__main__":
    main()