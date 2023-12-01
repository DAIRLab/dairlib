#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCStandingGains : OSCGains {
  double weight_scaling;
  double HipYawKp;
  double HipYawKd;
  double HipYawW;
  double center_of_mass_filter_tau;
  double rot_filter_tau;
  double center_of_mass_command_filter_alpha;
  double orientation_command_filter_alpha;
  std::vector<double> PelvisW;
  std::vector<double> PelvisKp;
  std::vector<double> PelvisKd;
  std::vector<double> PelvisRotW;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;

  MatrixXd W_pelvis;
  MatrixXd K_p_pelvis;
  MatrixXd K_d_pelvis;
  MatrixXd W_pelvis_rot;
  MatrixXd K_p_pelvis_rot;
  MatrixXd K_d_pelvis_rot;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;
  MatrixXd W_hip_yaw;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(weight_scaling));
    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(PelvisKp));
    a->Visit(DRAKE_NVP(PelvisKd));
    a->Visit(DRAKE_NVP(PelvisRotW));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(HipYawKp));
    a->Visit(DRAKE_NVP(HipYawKd));
    a->Visit(DRAKE_NVP(HipYawW));
    a->Visit(DRAKE_NVP(center_of_mass_filter_tau));
    a->Visit(DRAKE_NVP(rot_filter_tau));
    a->Visit(DRAKE_NVP(center_of_mass_command_filter_alpha));
    a->Visit(DRAKE_NVP(orientation_command_filter_alpha));

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
    K_p_hip_yaw = HipYawKp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = HipYawKd * MatrixXd::Identity(1, 1);
    W_hip_yaw = HipYawW * MatrixXd::Identity(1, 1);


    w_accel *= weight_scaling;
    w_input *= weight_scaling;
    w_input_reg *= weight_scaling;
    w_lambda *= weight_scaling;
    w_soft_constraint *= weight_scaling;
    W_pelvis_rot *= weight_scaling;
    W_pelvis *= weight_scaling;
    W_hip_yaw *= weight_scaling;
  }
};
