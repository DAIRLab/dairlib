#pragma once

#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCRomWalkingGains {
  int rows;
  int cols;
  double mu;
  double w_accel;
  double w_soft_constraint;
  double w_input_reg;
  double w_stance_toe_effort;
  double w_rom_contact_force_reg;
  std::vector<double> RomW;
  std::vector<double> RomKp;
  std::vector<double> RomKd;
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

  MatrixXd W_rom;
  MatrixXd K_p_rom;
  MatrixXd K_d_rom;
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

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(w_input_reg));
    a->Visit(DRAKE_NVP(w_stance_toe_effort));
    a->Visit(DRAKE_NVP(w_rom_contact_force_reg));
    a->Visit(DRAKE_NVP(RomW));
    a->Visit(DRAKE_NVP(RomKp));
    a->Visit(DRAKE_NVP(RomKd));
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

    W_rom = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->RomW.data(), this->rows, this->cols);
    K_p_rom = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->RomKp.data(), this->rows, this->cols);
    K_d_rom = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->RomKd.data(), this->rows, this->cols);
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
  }
};
