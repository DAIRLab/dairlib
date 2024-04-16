#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaControllerParams : OSCGains {
  std::string franka_model;
  std::string end_effector_model;
  std::string ground_model;
  std::string platform_model;
  std::string end_effector_name;

  Eigen::VectorXd tool_attachment_frame;
  Eigen::VectorXd ground_franka_frame;
  Eigen::VectorXd platform_franka_frame;
  Eigen::VectorXd franka_origin;
  double end_effector_acceleration;
  bool track_end_effector_orientation;
  bool cancel_gravity_compensation;
  bool enforce_acceleration_constraints;
  bool publish_debug_info;

  Eigen::VectorXd neutral_position;
  double x_scale;
  double y_scale;
  double z_scale;

  double w_elbow;
  double elbow_kp;
  double elbow_kd;

  std::vector<double> EndEffectorW;
  std::vector<double> EndEffectorKp;
  std::vector<double> EndEffectorKd;
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;
  std::vector<double> LambdaEndEffectorW;

  Eigen::MatrixXd W_end_effector;
  Eigen::MatrixXd K_p_end_effector;
  Eigen::MatrixXd K_d_end_effector;
  Eigen::MatrixXd W_mid_link;
  Eigen::MatrixXd K_p_mid_link;
  Eigen::MatrixXd K_d_mid_link;
  Eigen::MatrixXd W_end_effector_rot;
  Eigen::MatrixXd K_p_end_effector_rot;
  Eigen::MatrixXd K_d_end_effector_rot;
  Eigen::MatrixXd W_ee_lambda;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);

    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(platform_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(track_end_effector_orientation));
    a->Visit(DRAKE_NVP(cancel_gravity_compensation));
    a->Visit(DRAKE_NVP(enforce_acceleration_constraints));
    a->Visit(DRAKE_NVP(publish_debug_info));
    a->Visit(DRAKE_NVP(EndEffectorW));
    a->Visit(DRAKE_NVP(EndEffectorKp));
    a->Visit(DRAKE_NVP(EndEffectorKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));
    a->Visit(DRAKE_NVP(LambdaEndEffectorW));
    a->Visit(DRAKE_NVP(w_elbow));
    a->Visit(DRAKE_NVP(elbow_kp));
    a->Visit(DRAKE_NVP(elbow_kd));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(ground_franka_frame));
    a->Visit(DRAKE_NVP(platform_franka_frame));
    a->Visit(DRAKE_NVP(franka_origin));
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
    W_mid_link = this->w_elbow * MatrixXd::Identity(1, 1);
    K_p_mid_link = this->elbow_kp * MatrixXd::Identity(1, 1);
    K_d_mid_link = this->elbow_kd * MatrixXd::Identity(1, 1);
    W_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotW.data(), 3, 3);
    K_p_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKp.data(), 3, 3);
    K_d_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKd.data(), 3, 3);
    W_ee_lambda = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LambdaEndEffectorW.data(), 3, 3);
  }
};