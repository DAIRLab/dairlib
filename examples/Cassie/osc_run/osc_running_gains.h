#pragma once

#include "systems/controllers/osc/osc_gains.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCRunningGains : OSCGains {
  double w_swing_toe;
  double swing_toe_kp;
  double swing_toe_kd;
  double w_hip_yaw;
  double hip_yaw_kp;
  double hip_yaw_kd;
  double w_hip_pitch;
  double hip_pitch_kp;
  double hip_pitch_kd;
  double w_hip_roll;
  double hip_roll_kp;
  double hip_roll_kd;
  double vel_scale_rot;
  double vel_scale_trans_sagital;
  double vel_scale_trans_lateral;
  bool relative_feet;
  bool relative_pelvis;
  double rest_length;
  double stance_duration;
  double flight_duration;

  double center_line_offset;
  double footstep_offset;
  double mid_foot_height;

  std::vector<double> ekf_filter_tau;

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
  MatrixXd W_liftoff_swing_foot;
  MatrixXd K_p_liftoff_swing_foot;
  MatrixXd K_d_liftoff_swing_foot;
  MatrixXd W_swing_toe;
  MatrixXd K_p_swing_toe;
  MatrixXd K_d_swing_toe;
  MatrixXd W_hip_yaw;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(relative_feet));
    a->Visit(DRAKE_NVP(relative_pelvis));
    a->Visit(DRAKE_NVP(rest_length));
    a->Visit(DRAKE_NVP(stance_duration));
    a->Visit(DRAKE_NVP(flight_duration));
    a->Visit(DRAKE_NVP(center_line_offset));
    a->Visit(DRAKE_NVP(footstep_offset));
    a->Visit(DRAKE_NVP(mid_foot_height));
    a->Visit(DRAKE_NVP(ekf_filter_tau));

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

    W_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootW.data(), 3, 3);
    K_p_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKp.data(), 3, 3);
    K_d_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->SwingFootKd.data(), 3, 3);
     W_liftoff_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LiftoffSwingFootW.data(), 3, 3);
    K_p_liftoff_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LiftoffSwingFootKp.data(), 3, 3);
    K_d_liftoff_swing_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LiftoffSwingFootKd.data(), 3, 3);
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
  }
};