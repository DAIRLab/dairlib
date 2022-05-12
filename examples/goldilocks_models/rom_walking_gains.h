#pragma once

#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct RomWalkingGains {
  int rows;
  int cols;
  int rom_option;
  int model_iter;
  int sample_idx;
  bool use_radio;
  bool use_virtual_radio;
  bool set_constant_walking_speed;
  bool set_constant_turning_rate;
  double constant_step_length_x;
  double constant_step_length_y;
  double constant_turning_rate;

  bool use_lipm_mpc_and_ik;
  double w_Q;
  double w_R;
  double w_rom_reg;
  double w_reg_quat;
  double w_reg_xy;
  double w_reg_z;
  double w_reg_joints;
  double w_reg_hip_yaw;
  double w_reg_xy_vel;
  double w_reg_vel;
  double w_predict_lipm_p;
  double w_predict_lipm_v;
  double max_speed_lipm_mpc;
  double w_p_lipm_mpc;
  double w_v_lipm_mpc;
  int left_support;
  int right_support;
  int double_support;
  int post_left_double_support;
  int post_right_double_support;
  double left_support_duration;
  double right_support_duration;
  double double_support_duration;
  bool constant_rom_vel_during_double_support;
  double y_vel_offset;
  double max_foot_speed;
  double max_step_length;
  double max_desired_step_length;
  double max_lipm_step_length;
  double back_limit_wrt_pelvis;
  double front_limit_wrt_pelvis;
  double right_limit_wrt_pelvis;
  double left_limit_wrt_pelvis;
  std::string dir_model;
  std::string dir_data;

  bool relative_swing_ft;
  double swing_foot_target_offset_x;
  double max_CoM_to_footstep_dist;
  double center_line_offset;
  double footstep_offset;
  double mid_foot_height;
  double final_foot_height;
  double final_foot_velocity_z;
  double lipm_height;
  double ss_time;
  double ds_time;
  double k_ff_lateral;
  double k_fb_lateral;
  double k_ff_sagittal;
  double k_fb_sagittal;
  double speed_control_offset_sagittal;
  double kp_pos_sagital;
  double kd_pos_sagital;
  double vel_max_sagital;
  double kp_pos_lateral;
  double kd_pos_lateral;
  double vel_max_lateral;
  double kp_yaw;
  double kd_yaw;
  double vel_max_yaw;
  double target_pos_offset;
  double global_target_position_x;
  double global_target_position_y;
  double yaw_deadband_blur;
  double yaw_deadband_radius;
  double vel_scale_rot;
  double vel_scale_trans_sagital;
  double vel_scale_trans_lateral;

  // IK gains
  double kp_hip_roll_stance;
  double kp_hip_yaw_stance;
  double kp_hip_pitch_stance;
  double kp_knee_stance;
  double kp_toe_stance;

  double kd_hip_roll_stance;
  double kd_hip_yaw_stance;
  double kd_hip_pitch_stance;
  double kd_knee_stance;
  double kd_toe_stance;

  double kp_hip_roll_swing;
  double kp_hip_yaw_swing;
  double kp_hip_pitch_swing;
  double kp_knee_swing;
  double kp_toe_swing;

  double kd_hip_roll_swing;
  double kd_hip_yaw_swing;
  double kd_hip_pitch_swing;
  double kd_knee_swing;
  double kd_toe_swing;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(rom_option));
    a->Visit(DRAKE_NVP(model_iter));
    a->Visit(DRAKE_NVP(sample_idx));
    a->Visit(DRAKE_NVP(use_radio));
    a->Visit(DRAKE_NVP(use_virtual_radio));
    a->Visit(DRAKE_NVP(set_constant_walking_speed));
    a->Visit(DRAKE_NVP(set_constant_turning_rate));
    a->Visit(DRAKE_NVP(constant_step_length_x));
    a->Visit(DRAKE_NVP(constant_step_length_y));
    a->Visit(DRAKE_NVP(constant_turning_rate));
    a->Visit(DRAKE_NVP(use_lipm_mpc_and_ik));
    a->Visit(DRAKE_NVP(w_Q));
    a->Visit(DRAKE_NVP(w_R));
    a->Visit(DRAKE_NVP(w_rom_reg));
    a->Visit(DRAKE_NVP(w_reg_quat));
    a->Visit(DRAKE_NVP(w_reg_xy));
    a->Visit(DRAKE_NVP(w_reg_z));
    a->Visit(DRAKE_NVP(w_reg_joints));
    a->Visit(DRAKE_NVP(w_reg_hip_yaw));
    a->Visit(DRAKE_NVP(w_reg_xy_vel));
    a->Visit(DRAKE_NVP(w_reg_vel));
    a->Visit(DRAKE_NVP(w_predict_lipm_p));
    a->Visit(DRAKE_NVP(w_predict_lipm_v));
    a->Visit(DRAKE_NVP(max_speed_lipm_mpc));
    a->Visit(DRAKE_NVP(w_p_lipm_mpc));
    a->Visit(DRAKE_NVP(w_v_lipm_mpc));
    a->Visit(DRAKE_NVP(left_support));
    a->Visit(DRAKE_NVP(right_support));
    a->Visit(DRAKE_NVP(double_support));
    a->Visit(DRAKE_NVP(post_left_double_support));
    a->Visit(DRAKE_NVP(post_right_double_support));
    a->Visit(DRAKE_NVP(left_support_duration));
    a->Visit(DRAKE_NVP(right_support_duration));
    a->Visit(DRAKE_NVP(double_support_duration));
    a->Visit(DRAKE_NVP(constant_rom_vel_during_double_support));
    a->Visit(DRAKE_NVP(y_vel_offset));
    a->Visit(DRAKE_NVP(max_foot_speed));
    a->Visit(DRAKE_NVP(max_step_length));
    a->Visit(DRAKE_NVP(max_desired_step_length));
    a->Visit(DRAKE_NVP(max_lipm_step_length));
    a->Visit(DRAKE_NVP(back_limit_wrt_pelvis));
    a->Visit(DRAKE_NVP(front_limit_wrt_pelvis));
    a->Visit(DRAKE_NVP(right_limit_wrt_pelvis));
    a->Visit(DRAKE_NVP(left_limit_wrt_pelvis));
    a->Visit(DRAKE_NVP(dir_model));
    a->Visit(DRAKE_NVP(dir_data));

    a->Visit(DRAKE_NVP(relative_swing_ft));
    a->Visit(DRAKE_NVP(swing_foot_target_offset_x));
    // swing foot heuristics
    a->Visit(DRAKE_NVP(max_CoM_to_footstep_dist));
    a->Visit(DRAKE_NVP(center_line_offset));
    a->Visit(DRAKE_NVP(footstep_offset));
    a->Visit(DRAKE_NVP(mid_foot_height));
    a->Visit(DRAKE_NVP(final_foot_height));
    a->Visit(DRAKE_NVP(final_foot_velocity_z));
    // lipm heursitics
    a->Visit(DRAKE_NVP(lipm_height));
    // stance times
    a->Visit(DRAKE_NVP(ss_time));
    a->Visit(DRAKE_NVP(ds_time));
    // Speed control gains
    a->Visit(DRAKE_NVP(k_ff_lateral));
    a->Visit(DRAKE_NVP(k_fb_lateral));
    a->Visit(DRAKE_NVP(k_ff_sagittal));
    a->Visit(DRAKE_NVP(k_fb_sagittal));
    a->Visit(DRAKE_NVP(speed_control_offset_sagittal));
    // High level command gains (without radio)
    a->Visit(DRAKE_NVP(kp_pos_sagital));
    a->Visit(DRAKE_NVP(kd_pos_sagital));
    a->Visit(DRAKE_NVP(vel_max_sagital));
    a->Visit(DRAKE_NVP(kp_pos_lateral));
    a->Visit(DRAKE_NVP(kd_pos_lateral));
    a->Visit(DRAKE_NVP(vel_max_lateral));
    a->Visit(DRAKE_NVP(kp_yaw));
    a->Visit(DRAKE_NVP(kd_yaw));
    a->Visit(DRAKE_NVP(vel_max_yaw));
    a->Visit(DRAKE_NVP(target_pos_offset));
    a->Visit(DRAKE_NVP(global_target_position_x));
    a->Visit(DRAKE_NVP(global_target_position_y));
    a->Visit(DRAKE_NVP(yaw_deadband_blur));
    a->Visit(DRAKE_NVP(yaw_deadband_radius));
    // High level command gains (with radio)
    a->Visit(DRAKE_NVP(vel_scale_rot));
    a->Visit(DRAKE_NVP(vel_scale_trans_sagital));
    a->Visit(DRAKE_NVP(vel_scale_trans_lateral));

    // IK gains
    a->Visit(DRAKE_NVP(kp_hip_roll_stance));
    a->Visit(DRAKE_NVP(kp_hip_yaw_stance));
    a->Visit(DRAKE_NVP(kp_hip_pitch_stance));
    a->Visit(DRAKE_NVP(kp_knee_stance));
    a->Visit(DRAKE_NVP(kp_toe_stance));
    a->Visit(DRAKE_NVP(kd_hip_roll_stance));
    a->Visit(DRAKE_NVP(kd_hip_yaw_stance));
    a->Visit(DRAKE_NVP(kd_hip_pitch_stance));
    a->Visit(DRAKE_NVP(kd_knee_stance));
    a->Visit(DRAKE_NVP(kd_toe_stance));
    a->Visit(DRAKE_NVP(kp_hip_roll_swing));
    a->Visit(DRAKE_NVP(kp_hip_yaw_swing));
    a->Visit(DRAKE_NVP(kp_hip_pitch_swing));
    a->Visit(DRAKE_NVP(kp_knee_swing));
    a->Visit(DRAKE_NVP(kp_toe_swing));
    a->Visit(DRAKE_NVP(kd_hip_roll_swing));
    a->Visit(DRAKE_NVP(kd_hip_yaw_swing));
    a->Visit(DRAKE_NVP(kd_hip_pitch_swing));
    a->Visit(DRAKE_NVP(kd_knee_swing));
    a->Visit(DRAKE_NVP(kd_toe_swing));
  }
};
