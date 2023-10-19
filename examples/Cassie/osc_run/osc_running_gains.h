#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCRunningGains : OSCGains {

  double weight_scaling;
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double vel_scale_rot;
  double vel_scale_trans_sagital;
  double vel_scale_trans_lateral;
  double target_vel_filter_alpha;
  bool no_derivative_feedback;
  double rest_length;
  double rest_length_offset;
  double stance_duration;
  double flight_duration;
  double stance_variance;
  double flight_variance;

  double footstep_lateral_offset;
  double footstep_sagital_offset;
  double mid_foot_height;

  std::vector<double> ekf_filter_tau;
  double rot_filter_tau;
  double w_input_accel;
  double w_joint_limit;

  // swing foot tracking
  std::vector<double> SwingFootW;
  std::vector<double> SwingFootKp;
  std::vector<double> SwingFootKd;
  // swing foot tracking for the non-touchdown foot
  std::vector<double> LiftoffSwingFootW;
  std::vector<double> LiftoffSwingFootKp;
  std::vector<double> LiftoffSwingFootKd;
  // pelvis tracking
  std::vector<double> PelvisW;
  std::vector<double> PelvisKp;
  std::vector<double> PelvisKd;
  // pelvis tracking
  std::vector<double> PelvisRotW;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  // pelvis orientation tracking
  std::vector<double> FootstepKd;

  MatrixXd K_d_footstep;
  MatrixXd W_pelvis;
  MatrixXd K_p_pelvis;
  MatrixXd K_d_pelvis;
  MatrixXd W_pelvis_rot;
  MatrixXd K_p_pelvis_rot;
  MatrixXd K_d_pelvis_rot;
  MatrixXd W_swing_foot;
  MatrixXd K_p_swing_foot;
  MatrixXd K_d_swing_foot;
  MatrixXd W_swing_toe;
  MatrixXd K_p_swing_toe;
  MatrixXd K_d_swing_toe;
  MatrixXd W_hip_yaw;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(controller_frequency));
    a->Visit(DRAKE_NVP(weight_scaling));
    a->Visit(DRAKE_NVP(no_derivative_feedback));
    a->Visit(DRAKE_NVP(rest_length));
    a->Visit(DRAKE_NVP(rest_length_offset));
    a->Visit(DRAKE_NVP(stance_duration));
    a->Visit(DRAKE_NVP(flight_duration));
    a->Visit(DRAKE_NVP(stance_variance));
    a->Visit(DRAKE_NVP(flight_variance));
    a->Visit(DRAKE_NVP(footstep_lateral_offset));
    a->Visit(DRAKE_NVP(footstep_sagital_offset));
    a->Visit(DRAKE_NVP(mid_foot_height));
    a->Visit(DRAKE_NVP(ekf_filter_tau));
    a->Visit(DRAKE_NVP(rot_filter_tau));
    a->Visit(DRAKE_NVP(w_input_accel));
    a->Visit(DRAKE_NVP(w_joint_limit));


    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(PelvisKp));
    a->Visit(DRAKE_NVP(PelvisKd));
    a->Visit(DRAKE_NVP(PelvisRotW));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(FootstepKd));
    a->Visit(DRAKE_NVP(SwingFootW));
    a->Visit(DRAKE_NVP(SwingFootKp));
    a->Visit(DRAKE_NVP(SwingFootKd));
    a->Visit(DRAKE_NVP(LiftoffSwingFootW));
    a->Visit(DRAKE_NVP(LiftoffSwingFootKp));
    a->Visit(DRAKE_NVP(LiftoffSwingFootKd));
    a->Visit(DRAKE_NVP(w_swing_toe));
    a->Visit(DRAKE_NVP(swing_toe_kp));
    a->Visit(DRAKE_NVP(swing_toe_kd));
    a->Visit(DRAKE_NVP(w_hip_yaw));
    a->Visit(DRAKE_NVP(hip_yaw_kp));
    a->Visit(DRAKE_NVP(hip_yaw_kd));
    // High level command gains (with radio)
    a->Visit(DRAKE_NVP(vel_scale_rot));
    a->Visit(DRAKE_NVP(vel_scale_trans_sagital));
    a->Visit(DRAKE_NVP(vel_scale_trans_lateral));
    a->Visit(DRAKE_NVP(target_vel_filter_alpha));

    W_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootW.data(), 3, 3);
    K_p_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKp.data(), 3, 3);
    K_d_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKd.data(), 3, 3);
    W_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisW.data(), 3, 3);
    K_p_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisKp.data(), 3, 3);
    K_d_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisKd.data(), 3, 3);
    W_pelvis_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotW.data(), 3, 3);
    K_p_pelvis_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotKp.data(), 3, 3);
    K_d_pelvis_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->PelvisRotKd.data(), 3, 3);
    K_d_footstep = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FootstepKd.data(), 3, 3);

    W_swing_toe = this->w_swing_toe * MatrixXd::Identity(1, 1);
    K_p_swing_toe = this->swing_toe_kp * MatrixXd::Identity(1, 1);
    K_d_swing_toe = this->swing_toe_kd * MatrixXd::Identity(1, 1);
    W_hip_yaw = this->w_hip_yaw * MatrixXd::Identity(1, 1);
    K_p_hip_yaw = this->hip_yaw_kp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = this->hip_yaw_kd * MatrixXd::Identity(1, 1);

    w_accel *= weight_scaling;
    w_input *= weight_scaling;
    w_input_reg *= weight_scaling;
    w_lambda *= weight_scaling;
    w_soft_constraint *= weight_scaling;
    w_joint_limit *= weight_scaling;
    W_pelvis *= weight_scaling;
    W_pelvis_rot *= weight_scaling;
    W_swing_foot *= weight_scaling;
    W_swing_toe *= weight_scaling;
    W_hip_yaw *= weight_scaling;
  }
};