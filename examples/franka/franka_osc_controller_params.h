#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaControllerParams : OSCGains {
  std::string state_channel;
  std::string controller_channel;
  std::string radio_channel;
  std::string osc_debug_channel;
  std::string c3_channel;

  std::string franka_model;
  std::string end_effector_model;
  std::string end_effector_name;

  std::vector<double> tool_attachment_frame;
  double end_effector_acceleration;
  bool track_end_effector_orientation;

  double x_scale;
  double y_scale;
  double z_scale;

  std::vector<double> neutral_position;
  std::vector<double> EndEffectorW;
  std::vector<double> EndEffectorKp;
  std::vector<double> EndEffectorKd;
  std::vector<double> MidLinkW;
  std::vector<double> MidLinkKp;
  std::vector<double> MidLinkKd;
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;

//  Eigen::Vector3d neutral_position;
  Eigen::MatrixXd W_end_effector;
  Eigen::MatrixXd K_p_end_effector;
  Eigen::MatrixXd K_d_end_effector;
  Eigen::MatrixXd W_mid_link;
  Eigen::MatrixXd K_p_mid_link;
  Eigen::MatrixXd K_d_mid_link;
  Eigen::MatrixXd W_end_effector_rot;
  Eigen::MatrixXd K_p_end_effector_rot;
  Eigen::MatrixXd K_d_end_effector_rot;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);

    a->Visit(DRAKE_NVP(state_channel));
    a->Visit(DRAKE_NVP(controller_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
    a->Visit(DRAKE_NVP(c3_channel));
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(track_end_effector_orientation));
    a->Visit(DRAKE_NVP(EndEffectorW));
    a->Visit(DRAKE_NVP(EndEffectorKp));
    a->Visit(DRAKE_NVP(EndEffectorKd));
    a->Visit(DRAKE_NVP(MidLinkW));
    a->Visit(DRAKE_NVP(MidLinkKp));
    a->Visit(DRAKE_NVP(MidLinkKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));

    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(neutral_position));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));

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