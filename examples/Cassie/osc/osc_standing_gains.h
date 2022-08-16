#pragma once

#include "systems/controllers/osc/osc_gains.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct OSCStandingGains : OSCGains {
  int rows;
  int cols;
  double HipYawKp;
  double HipYawKd;
  double HipYawW;
  double center_of_mass_filter_tau;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  std::vector<double> CoMW;
  std::vector<double> PelvisW;

  MatrixXd K_p_com;
  MatrixXd K_d_com;
  MatrixXd K_p_pelvis;
  MatrixXd K_d_pelvis;
  MatrixXd K_p_hip_yaw;
  MatrixXd K_d_hip_yaw;
  MatrixXd W_com;
  MatrixXd W_pelvis;
  MatrixXd W_hip_yaw;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(HipYawKp));
    a->Visit(DRAKE_NVP(HipYawKd));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(HipYawW));
    a->Visit(DRAKE_NVP(center_of_mass_filter_tau));

    K_p_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        CoMKp.data(), rows, cols);
    K_d_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        CoMKd.data(), rows, cols);
    K_p_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        PelvisRotKp.data(), rows, cols);
    K_d_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        PelvisRotKd.data(), rows, cols);
    K_p_hip_yaw = HipYawKp * MatrixXd::Identity(1, 1);
    K_d_hip_yaw = HipYawKd * MatrixXd::Identity(1, 1);
    W_com = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        CoMW.data(), rows, cols);
    W_pelvis = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        PelvisW.data(), rows, cols);
    W_hip_yaw = HipYawW * MatrixXd::Identity(1, 1);
  }
};
