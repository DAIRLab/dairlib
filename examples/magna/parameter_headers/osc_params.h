#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

/// Extends OSCGains to include parameters specific to belt assembly example.
struct MagnaOSCParams : OSCGains {
  /// Teleop params:  There are two teleoperation modes:
  /// 1) locked EE neutral position where xbox controls perturb the EE about
  /// this neutral position (corresponds to teleop_neutral_position = false).
  /// 2) free EE neutral position where the EE can be moved freely in space
  /// where xbox controls change the EE resting position (corresponds to
  /// teleop_neutral_position = true).
  Eigen::VectorXd neutral_position;
  bool teleop_neutral_position;
  double x_scale;
  double y_scale;
  double z_scale;

  double end_effector_acceleration;
  bool track_end_effector_orientation;
  bool cancel_gravity_compensation;
  bool enforce_acceleration_constraints;
  bool publish_debug_info;

  /// Joint position tracking weight values.
  double w_joint_tracking;
  double joint_tracking_kp;
  double joint_tracking_kd;
  /// End effector position tracking weight matrix values.
  std::vector<double> EndEffectorW;
  std::vector<double> EndEffectorKp;
  std::vector<double> EndEffectorKd;
  /// End effector orientation tracking weight matrix values.
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;
  /// End effector force tracking weight matrix values.
  std::vector<double> LambdaEndEffectorW;

  Eigen::MatrixXd W_mid_link;
  Eigen::MatrixXd K_p_mid_link;
  Eigen::MatrixXd K_d_mid_link;
  Eigen::MatrixXd W_end_effector;
  Eigen::MatrixXd K_p_end_effector;
  Eigen::MatrixXd K_d_end_effector;
  Eigen::MatrixXd W_end_effector_rot;
  Eigen::MatrixXd K_p_end_effector_rot;
  Eigen::MatrixXd K_d_end_effector_rot;
  Eigen::MatrixXd W_ee_lambda;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(neutral_position));
    a->Visit(DRAKE_NVP(teleop_neutral_position));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(track_end_effector_orientation));
    a->Visit(DRAKE_NVP(cancel_gravity_compensation));
    a->Visit(DRAKE_NVP(enforce_acceleration_constraints));
    a->Visit(DRAKE_NVP(publish_debug_info));
    a->Visit(DRAKE_NVP(w_joint_tracking));
    a->Visit(DRAKE_NVP(joint_tracking_kp));
    a->Visit(DRAKE_NVP(joint_tracking_kd));
    a->Visit(DRAKE_NVP(EndEffectorW));
    a->Visit(DRAKE_NVP(EndEffectorKp));
    a->Visit(DRAKE_NVP(EndEffectorKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));
    a->Visit(DRAKE_NVP(LambdaEndEffectorW));

    // Weight matrix for end effector position tracking.
    W_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorW.data(), 3, 3);
    K_p_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorKp.data(), 3, 3);
    K_d_end_effector = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorKd.data(), 3, 3);
    W_mid_link = this->w_joint_tracking * MatrixXd::Identity(1, 1);
    K_p_mid_link = this->joint_tracking_kp * MatrixXd::Identity(1, 1);
    K_d_mid_link = this->joint_tracking_kd * MatrixXd::Identity(1, 1);
    W_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotW.data(), 3, 3);
    K_p_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKp.data(), 3, 3);
    K_d_end_effector_rot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKd.data(), 3, 3);
    // Weight matrix for ee force tracking.
    W_ee_lambda = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->LambdaEndEffectorW.data(), 3, 3);
  }
};
