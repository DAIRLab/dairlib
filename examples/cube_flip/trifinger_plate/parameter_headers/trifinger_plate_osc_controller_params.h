#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct TrifingerPlateControllerParams : OSCGains {
  std::string trifinger_model;
  std::string end_effector_model;
  std::string end_effector_name;

  std::vector<Eigen::VectorXd> finger_attachment_points;
  double translation_stiffness;
  double translation_damping;

  double end_effector_acceleration;
  bool track_end_effector_orientation;
  bool cancel_gravity_compensation;
  bool enforce_acceleration_constraints;
  bool publish_debug_info;

  Eigen::VectorXd neutral_position;
  double x_scale;
  double y_scale;
  double z_scale;

  std::vector<double> EndEffectorW;
  std::vector<double> EndEffectorKp;
  std::vector<double> EndEffectorKd;
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;
  std::vector<double> LambdaEndEffectorW;
  std::vector<double> LambdaEndEffectorTauW;

  Eigen::MatrixXd W_end_effector;
  Eigen::MatrixXd K_p_end_effector;
  Eigen::MatrixXd K_d_end_effector;
  Eigen::MatrixXd W_end_effector_rot;
  Eigen::MatrixXd K_p_end_effector_rot;
  Eigen::MatrixXd K_d_end_effector_rot;
  Eigen::MatrixXd W_ee_lambda;
  Eigen::MatrixXd W_ee_lambda_tau;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    
    a->Visit(DRAKE_NVP(trifinger_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(finger_attachment_points));
    a->Visit(DRAKE_NVP(translation_stiffness));
    a->Visit(DRAKE_NVP(translation_damping));
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
    a->Visit(DRAKE_NVP(LambdaEndEffectorTauW));
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
    W_ee_lambda_tau = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LambdaEndEffectorTauW.data(), 3, 3);
  }
};