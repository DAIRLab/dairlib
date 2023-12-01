#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCWalkingGains {
  int rows;
  int cols;
  double mu;
  double w_accel;
  double w_soft_constraint;
  double w_input_reg;
  double impact_threshold;
  bool relative_swing_ft;
  std::vector<double> pelvis_xyz_vel_filter_tau;
  std::vector<double> CoMW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisHeadingW;
  std::vector<double> PelvisHeadingKp;
  std::vector<double> PelvisHeadingKd;
  std::vector<double> PelvisBalanceW;
  std::vector<double> PelvisBalanceKp;
  std::vector<double> PelvisBalanceKd;
  std::vector<double> SwingFootW;
  std::vector<double> SwingFootKp;
  std::vector<double> SwingFootKd;
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double period_of_no_heading_control;
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

  MatrixXd W_com;
  MatrixXd K_p_com;
  MatrixXd K_d_com;
  MatrixXd W_pelvis_heading;
  MatrixXd K_p_pelvis_heading;
  MatrixXd K_d_pelvis_heading;
  MatrixXd W_pelvis_balance;
  MatrixXd K_p_pelvis_balance;
  MatrixXd K_d_pelvis_balance;
  MatrixXd W_swing_foot;
  MatrixXd K_p_swing_foot;
  MatrixXd K_d_swing_foot;
  MatrixXd W_swing_toe;
  MatrixXd K_p_swing_toe;
  MatrixXd K_d_swing_toe;
  MatrixXd W_hip_yaw;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;

  std::vector<double> W_lambda_c_reg;
  std::vector<double> W_lambda_h_reg;
  MatrixXd W_lambda_c_regularization;
  MatrixXd W_lambda_h_regularization;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(w_input_reg));
    a->Visit(DRAKE_NVP(relative_swing_ft));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(pelvis_xyz_vel_filter_tau));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisHeadingW));
    a->Visit(DRAKE_NVP(PelvisHeadingKp));
    a->Visit(DRAKE_NVP(PelvisHeadingKd));
    a->Visit(DRAKE_NVP(PelvisBalanceW));
    a->Visit(DRAKE_NVP(PelvisBalanceKp));
    a->Visit(DRAKE_NVP(PelvisBalanceKd));
    a->Visit(DRAKE_NVP(SwingFootW));
    a->Visit(DRAKE_NVP(SwingFootKp));
    a->Visit(DRAKE_NVP(SwingFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    a->Visit(DRAKE_NVP(period_of_no_heading_control));
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

    W_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMW.data(), this->rows, this->cols);
    K_p_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMKp.data(), this->rows, this->cols);
    K_d_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->CoMKd.data(), this->rows, this->cols);
    W_pelvis_heading = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisHeadingW.data(), this->rows, this->cols);
    K_p_pelvis_heading = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisHeadingKp.data(), this->rows, this->cols);
    K_d_pelvis_heading = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisHeadingKd.data(), this->rows, this->cols);
    W_pelvis_balance = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisBalanceW.data(), this->rows, this->cols);
    K_p_pelvis_balance = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisBalanceKp.data(), this->rows, this->cols);
    K_d_pelvis_balance = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisBalanceKd.data(), this->rows, this->cols);
    W_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootW.data(), this->rows, this->cols);
    K_p_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKp.data(), this->rows, this->cols);
    K_d_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKd.data(), this->rows, this->cols);
    W_swing_toe = this->w_swing_toe * MatrixXd::Identity(1, 1);
    K_p_swing_toe = this->swing_toe_kp * MatrixXd::Identity(1, 1);
    K_d_swing_toe = this->swing_toe_kd * MatrixXd::Identity(1, 1);
    W_hip_yaw = this->w_hip_yaw * MatrixXd::Identity(1, 1);
    K_p_hip_yaw = this->hip_yaw_kp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = this->hip_yaw_kd * MatrixXd::Identity(1, 1);

    a->Visit(DRAKE_NVP(W_lambda_c_reg));
    a->Visit(DRAKE_NVP(W_lambda_h_reg));
    Eigen::VectorXd w_lambda_c_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            this->W_lambda_c_reg.data(), this->W_lambda_c_reg.size());
    Eigen::VectorXd w_lambda_h_regularization =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            this->W_lambda_h_reg.data(), this->W_lambda_h_reg.size());
    W_lambda_c_regularization = w_lambda_c_regularization.asDiagonal();
    W_lambda_h_regularization = w_lambda_h_regularization.asDiagonal();
  }
};
