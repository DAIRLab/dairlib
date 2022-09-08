import argparse
import sys
import tqdm
import numpy as np
import matplotlib.pyplot as plt
from cassie_model import CassieSystem
from utils import *
from plot_utlis import *
from residual_processor import CassieResidualAnalyzer
from log_processor import LogProcessor

class CassieResidualAnalysisMain():
    def __init__(self, path, is_wandb, is_show_plot, is_show_numerical_value, start_time, duration, wandb_name): 
        
        self.cassie = CassieSystem()

        self.is_wandb = is_wandb

        self.is_show_plot = is_show_plot

        self.is_show_numerical_value = is_show_numerical_value

        if self.is_wandb:
            self.wandb_processor = WandbProcessor(wandb_name)

        self.start_time = start_time
        self.duration = duration

        self.residual_analyzer = CassieResidualAnalyzer()
        self.log_processor = LogProcessor(self.start_time, self.duration)

        self.residual_analyzer.set_num_of_pos(self.cassie.num_pos)
        self.residual_analyzer.set_num_of_vel(self.cassie.num_vel)

        self.pos_map, self.pos_map_inverse = self.cassie.make_position_map()
        self.vel_map, self.vel_map_inverse = self.cassie.make_velocity_map()
        self.act_map, self.act_map_inverse = self.cassie.make_actuator_map()

        self.residual_analyzer.set_pos_map(self.pos_map, self.pos_map_inverse)
        self.residual_analyzer.set_vel_map(self.vel_map, self.vel_map_inverse)
        self.residual_analyzer.set_act_map(self.act_map, self.act_map_inverse)
        
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
        self.residual_analyzer.set_data(self.cassie.intermediate_variables_list)
        self.residual_analyzer.process_data()
        print("Finish Calculate additional infos.")
        
        if self.is_wandb:
            print("Begin update data for plots")
            for processed_datum in self.residual_analyzer.processed_data:
                self.wandb_processor.add_info(processed_datum)
            print("Finish update data for plots")

        if self.is_show_numerical_value:
            self.show_numerical_quality_of_log()

        if self.is_show_plot:
        
            print("Begin making plots")
        
            # Use self.vel_map.keys() for all joints
            plot_list_for_v_dot = ["hip_roll_leftdot", "hip_yaw_leftdot", "hip_pitch_leftdot", "knee_leftdot", "knee_joint_leftdot", "ankle_joint_leftdot"]
            # Use self.act_map.keys() for all motors
            plot_list_for_u = ["hip_roll_left_motor", "hip_roll_right_motor"]

            for joint_name in plot_list_for_v_dot:
                plot_joint_residuals_vs_time(self.residual_analyzer.processed_data, self.actual_start_time, self.actual_end_time, joint_name)
            
            plot_joint_effort_vs_time(self.residual_analyzer.processed_data, self.actual_start_time, self.actual_end_time, plot_list_for_u)
            
            plot_spring_force_vs_time(self.residual_analyzer.processed_data, self.actual_start_time, self.actual_end_time)
            plt.show()
            print("Finish making plots")
        
    def calc_mean_residuals_info_at_given_period(self, start_time, end_time, is_show_freq_plot=False,joint_name=None, residual_name="residual_for_best_spring_model"):
        self.residual_analyzer.calc_residuals_info_at_given_period(start_time, end_time, joint_name, residual_name=residual_name, is_show_freq_plot=is_show_freq_plot, is_show_numerical_value=True)

    def show_hip_residuals_given_time_period(self, start_time, end_time, residual_name="residual_for_best_spring_model"):
        self.residual_analyzer.show_hip_residuals_given_time_period(start_time, end_time, residual_name=residual_name)

    def show_numerical_quality_of_log(self, contact_mode_switch_period=0.025):
        """
        The function shows the quantity values of how our model different from the hardware test.
        
        The values are: 
            how different spring models are compared to linear model during the period when legs are about leaving the grounds,
            the residuals of hip roll joints when the legs are on the air, about to leaving ground, and about to landing,
            the residuals of hip yaw joints in whole period, about to leaving ground, and about to landing.
        """

        t = [x["t"] for x in self.residual_analyzer.processed_data]
        is_contact = [[x["is_contact"]["left"], x["is_contact"]["right"]] for x in self.residual_analyzer.processed_data]
        t = np.array(t)
        is_contact = np.array(is_contact, dtype=int)

        # compute the residual of spring model during the period when leg is leaving
        left_leg_leaving_time_points, left_leg_landing_time_points, right_leg_leaving_time_points, right_leg_landing_time_points = self.residual_analyzer.calc_contact_mode_changed_time_points(is_contact=is_contact, t=t)

        residuals_of_linear_spring_model_during_left_leg_leaving_ground = None
        residuals_of_linear_spring_model_during_right_leg_leaving_ground = None

        for left_leg_leaving_time_point in left_leg_leaving_time_points:
            if residuals_of_linear_spring_model_during_left_leg_leaving_ground is None:
                residuals_of_linear_spring_model_during_left_leg_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_of_linear_spring_given_time_periods(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period))
            else:
                residuals_of_linear_spring_model_during_left_leg_leaving_ground = np.concatenate((residuals_of_linear_spring_model_during_left_leg_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_of_linear_spring_given_time_periods(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period))))
        
        for right_leg_leaving_time_point in right_leg_leaving_time_points:
            if residuals_of_linear_spring_model_during_right_leg_leaving_ground is None:
                residuals_of_linear_spring_model_during_right_leg_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_of_linear_spring_given_time_periods(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period))
            else:
                residuals_of_linear_spring_model_during_right_leg_leaving_ground = np.concatenate((residuals_of_linear_spring_model_during_right_leg_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_of_linear_spring_given_time_periods(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period))))


        print("Residual of linear spring model during period of leaving ground (unit in N*m):")
        print(f"{'joint_name':<28} {'mean absolute':>13} {'max abs':>13}")
        print(f"{'left knee joint':<28} {np.mean(residuals_of_linear_spring_model_during_left_leg_leaving_ground[:,0]):13.2f} {np.max(residuals_of_linear_spring_model_during_left_leg_leaving_ground[:,0]):>13.2f}")
        print(f"{'left ankle joint':<28} {np.mean(residuals_of_linear_spring_model_during_left_leg_leaving_ground[:,2]):13.2f} {np.max(residuals_of_linear_spring_model_during_left_leg_leaving_ground[:,2]):>13.2f}")
        print(f"{'right knee joint':<28} {np.mean(residuals_of_linear_spring_model_during_right_leg_leaving_ground[:,1]):13.2f} {np.max(residuals_of_linear_spring_model_during_right_leg_leaving_ground[:,1]):>13.2f}")
        print(f"{'left ankle joint':<28} {np.mean(residuals_of_linear_spring_model_during_right_leg_leaving_ground[:,3]):13.2f} {np.max(residuals_of_linear_spring_model_during_right_leg_leaving_ground[:,3]):>13.2f}")

        # compute the residual of hip roll
        residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground = None
        residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground = None
        residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground = None
        residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground = None
        residuals_of_hip_roll_left_during_left_leg_on_the_air = None
        residuals_of_hip_roll_right_during_right_leg_on_the_air = None

        left_leg_on_the_air_time_periods, left_leg_on_the_ground_time_periods, right_leg_on_the_air_time_periods, right_leg_on_the_ground_time_periods = self.residual_analyzer.calc_contact_mode_maintained_time_periods(is_contact=is_contact, t=t)

        # leg leaving period
        for left_leg_leaving_time_point in left_leg_leaving_time_points:
            if residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground is None:
                residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period, ["hip_roll_leftdot"])["hip_roll_leftdot"])
            else:
                residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground = np.concatenate((residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period, ["hip_roll_leftdot"])["hip_roll_leftdot"])))
        
        for right_leg_leaving_time_point in right_leg_leaving_time_points:
            if residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground is None:
                residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period, ["hip_roll_rightdot"])["hip_roll_rightdot"])
            else:
                residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground = np.concatenate((residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period, ["hip_roll_rightdot"])["hip_roll_rightdot"])))

        # leg landing period
        for left_leg_landing_time_point in left_leg_landing_time_points:
            if residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground is None:
                residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_landing_time_point-contact_mode_switch_period, left_leg_landing_time_point+contact_mode_switch_period, ["hip_roll_leftdot"])["hip_roll_leftdot"])
            else:
                residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground = np.concatenate((residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_landing_time_point-contact_mode_switch_period, left_leg_landing_time_point+contact_mode_switch_period, ["hip_roll_leftdot"])["hip_roll_leftdot"])))
        
        for right_leg_landing_time_point in right_leg_landing_time_points:
            if residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground is None:
                residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_landing_time_point-contact_mode_switch_period, right_leg_landing_time_point+contact_mode_switch_period, ["hip_roll_rightdot"])["hip_roll_rightdot"])
            else:
                residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground = np.concatenate((residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_landing_time_point-contact_mode_switch_period, right_leg_landing_time_point+contact_mode_switch_period, ["hip_roll_rightdot"])["hip_roll_rightdot"])))

        # leg on the air period
        for left_leg_on_the_air_time_period in left_leg_on_the_air_time_periods:
            if residuals_of_hip_roll_left_during_left_leg_on_the_air is None:
                residuals_of_hip_roll_left_during_left_leg_on_the_air = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_on_the_air_time_period[0], left_leg_on_the_air_time_period[1], ["hip_roll_leftdot"])["hip_roll_leftdot"])
            else:
                residuals_of_hip_roll_left_during_left_leg_on_the_air = np.concatenate((residuals_of_hip_roll_left_during_left_leg_on_the_air, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_on_the_air_time_period[0], left_leg_on_the_air_time_period[1], ["hip_roll_leftdot"])["hip_roll_leftdot"])))
        
        for right_leg_on_the_air_time_period in right_leg_on_the_air_time_periods:
            if residuals_of_hip_roll_right_during_right_leg_on_the_air is None:
                residuals_of_hip_roll_right_during_right_leg_on_the_air = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_on_the_air_time_period[0], right_leg_on_the_air_time_period[1], ["hip_roll_rightdot"])["hip_roll_rightdot"])
            else:
                residuals_of_hip_roll_right_during_right_leg_on_the_air = np.concatenate((residuals_of_hip_roll_right_during_right_leg_on_the_air, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_on_the_air_time_period[0], right_leg_on_the_air_time_period[1], ["hip_roll_rightdot"])["hip_roll_rightdot"])))
        
        print("Residual of hip roll(unit in rad/s^2)")
        print(f"{'joint_name':<15} {'phase':<28} {'mean absolute':>13} {'max absolute':>13}")
        print(f"{'hip roll left':<15} {'leg on the air':<28} {np.mean(residuals_of_hip_roll_left_during_left_leg_on_the_air):>13.2f} {np.max(residuals_of_hip_roll_left_during_left_leg_on_the_air):>13.2f}")
        print(f"{'hip roll left':<15} {'leg about leaving ground':<28} {np.mean(residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground):>13.2f} {np.max(residuals_of_hip_roll_left_when_left_leg_is_about_leaving_ground):>13.2f}")
        print(f"{'hip roll left':<15} {'leg about touching ground':<28} {np.mean(residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground):>13.2f} {np.max(residuals_of_hip_roll_left_when_left_leg_is_about_touching_ground):>13.2f}")
        print(f"{'hip roll right':<15} {'leg on the air':<28} {np.mean(residuals_of_hip_roll_right_during_right_leg_on_the_air):>13.2f} {np.max(residuals_of_hip_roll_right_during_right_leg_on_the_air):>13.2f}")
        print(f"{'hip roll right':<15} {'leg about leaving gorund':<28} {np.mean(residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground):>13.2f} {np.max(residuals_of_hip_roll_right_when_right_leg_is_about_leaving_ground):>13.2f}")
        print(f"{'hip roll right':<15} {'leg about touching ground':<28} {np.mean(residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground):>13.2f} {np.max(residuals_of_hip_roll_right_when_right_leg_is_about_touching_ground):>13.2f}")

        # compute the residual of hip yaw

        residuals_of_hip_yaw_left = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(t[0], t[-1], ["hip_yaw_leftdot"])["hip_yaw_leftdot"])
        residuals_of_hip_yaw_right = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(t[0], t[-1], ["hip_yaw_rightdot"])["hip_yaw_rightdot"])
        residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground = None
        residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground = None
        residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground = None
        residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground = None

        # leg leaving period
        for left_leg_leaving_time_point in left_leg_leaving_time_points:
            if residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground is None:
                residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period, ["hip_yaw_leftdot"])["hip_yaw_leftdot"])
            else:
                residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground = np.concatenate((residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_leaving_time_point-contact_mode_switch_period, left_leg_leaving_time_point+contact_mode_switch_period, ["hip_yaw_leftdot"])["hip_yaw_leftdot"])))
        
        for right_leg_leaving_time_point in right_leg_leaving_time_points:
            if residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground is None:
                residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period, ["hip_yaw_rightdot"])["hip_yaw_rightdot"])
            else:
                residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground = np.concatenate((residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_leaving_time_point-contact_mode_switch_period, right_leg_leaving_time_point+contact_mode_switch_period, ["hip_yaw_rightdot"])["hip_yaw_rightdot"])))

        # leg landing period
        for left_leg_landing_time_point in left_leg_landing_time_points:
            if residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground is None:
                residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_landing_time_point-contact_mode_switch_period, left_leg_landing_time_point+contact_mode_switch_period, ["hip_yaw_leftdot"])["hip_yaw_leftdot"])
            else:
                residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground = np.concatenate((residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(left_leg_landing_time_point-contact_mode_switch_period, left_leg_landing_time_point+contact_mode_switch_period, ["hip_yaw_leftdot"])["hip_yaw_leftdot"])))
        
        for right_leg_landing_time_point in right_leg_landing_time_points:
            if residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground is None:
                residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground = np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_landing_time_point-contact_mode_switch_period, right_leg_landing_time_point+contact_mode_switch_period, ["hip_yaw_rightdot"])["hip_yaw_rightdot"])
            else:
                residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground = np.concatenate((residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground, np.abs(self.residual_analyzer.calc_residuals_info_at_given_period(right_leg_landing_time_point-contact_mode_switch_period, right_leg_landing_time_point+contact_mode_switch_period, ["hip_yaw_rightdot"])["hip_yaw_rightdot"])))

        print("Residual of hip yaw (unit in rad/s^2)")
        print(f"{'joint_name':<15} {'phase':<28} {'mean absolute':>13} {'max absolute':>13}")
        print(f"{'hip yaw left':<15} {'whole period':<28} {np.mean(residuals_of_hip_yaw_left):>13.2f} {np.max(residuals_of_hip_yaw_left):>13.2f}")
        print(f"{'hip yaw left':<15} {'leg about leaving ground':<28} {np.mean(residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground):>13.2f} {np.max(residuals_of_hip_yaw_left_when_left_leg_is_about_leaving_ground):>13.2f}")
        print(f"{'hip yaw left':<15} {'leg about touching ground':<28} {np.mean(residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground):>13.2f} {np.max(residuals_of_hip_yaw_left_when_left_leg_is_about_touching_ground):>13.2f}")
        print(f"{'hip yaw right':<15} {'whole period':<28} {np.mean(residuals_of_hip_yaw_right):>13.2f} {np.max(residuals_of_hip_yaw_right):>13.2f}")
        print(f"{'hip yaw right':<15} {'leg about leaving gorund':<28} {np.mean(residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground):>13.2f} {np.max(residuals_of_hip_yaw_right_when_right_leg_is_about_leaving_ground):>13.2f}")
        print(f"{'hip yaw right':<15} {'leg about touching ground':<28} {np.mean(residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground):>13.2f} {np.max(residuals_of_hip_yaw_right_when_right_leg_is_about_touching_ground):>13.2f}")

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
    parser.add_argument('--frame_num', type=int, default=1)
    parser.add_argument('--wandb_name', type=str, default="residual_analysis")
    parser.add_argument('--is_show_plot', action="store_false")
    parser.add_argument('--is_show_numerical_value', action="store_false")
    args = parser.parse_args()

    simulationDataTester = CassieResidualAnalysisMain(path, is_wandb=args.is_wandb, is_show_plot=args.is_show_plot, is_show_numerical_value=args.is_show_numerical_value, start_time=args.start_time, duration=args.duration, wandb_name=args.wandb_name)
    simulationDataTester.main(args.frame_num)

if __name__ == "__main__":
    main()