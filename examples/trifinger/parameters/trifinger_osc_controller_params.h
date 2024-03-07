#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerControllerParams : OSCGains {
  std::string trifinger_model;
  std::string cube_model;
  std::string fingertip_0_name;
  std::string fingertip_120_name;
  std::string fingertip_240_name;

  Eigen::Vector3d min_fingertips_delta_position;
  Eigen::Vector3d max_fingertips_delta_position;

  std::vector<double> neutral_position;
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

    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(cube_model));
    a->Visit(DRAKE_NVP(fingertip_0_name));
    a->Visit(DRAKE_NVP(fingertip_120_name));
    a->Visit(DRAKE_NVP(fingertip_240_name));
    a->Visit(DRAKE_NVP(min_fingertips_delta_position));
    a->Visit(DRAKE_NVP(max_fingertips_delta_position));
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