#pragma once

#include "systems/controllers/osc/osc_gains.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct FrankaControllerParams : OSCGains {
  std::string state_channel;
  std::string controller_channel;
  std::string radio_channel;
  std::string osc_debug_channel;

  double end_effector_acceleration;

  std::vector<double> EndEffectorW;
  std::vector<double> EndEffectorKp;
  std::vector<double> EndEffectorKd;
  std::vector<double> MidLinkW;
  std::vector<double> MidLinkKp;
  std::vector<double> MidLinkKd;
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;

  MatrixXd W_end_effector;
  MatrixXd K_p_end_effector;
  MatrixXd K_d_end_effector;
  MatrixXd W_mid_link;
  MatrixXd K_p_mid_link;
  MatrixXd K_d_mid_link;
  MatrixXd W_end_effector_rot;
  MatrixXd K_p_end_effector_rot;
  MatrixXd K_d_end_effector_rot;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);

    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(controller_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(EndEffectorW));
    a->Visit(DRAKE_NVP(EndEffectorKp));
    a->Visit(DRAKE_NVP(EndEffectorKd));
    a->Visit(DRAKE_NVP(MidLinkW));
    a->Visit(DRAKE_NVP(MidLinkKp));
    a->Visit(DRAKE_NVP(MidLinkKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));

    W_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorW.data(), 3, 3);
    K_p_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorKp.data(), 3, 3);
    K_d_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorKd.data(), 3, 3);
    W_mid_link = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->MidLinkW.data(), 3, 3);
    K_p_mid_link = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->MidLinkKp.data(), 3, 3);
    K_d_mid_link = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->MidLinkKd.data(), 3, 3);
    W_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotW.data(), 3, 3);
    K_p_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKp.data(), 3, 3);
    K_d_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKd.data(), 3, 3);
  }
};