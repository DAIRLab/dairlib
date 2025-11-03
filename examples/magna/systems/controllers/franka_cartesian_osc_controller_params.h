#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace controllers {
/// Extends OSCGains to include parameters specific to franka cartesian osc
/// controller.
struct FrankaCartesianOSCControllerParams {
  OSCGains osc_gains;
  std::string osc_qp_settings_file;
  std::string franka_state_channel;
  std::string franka_input_channel;
  std::string target_cartesian_pose_channel;
  std::string target_cartesian_pose_trajectory_channel;

  std::optional<std::string> end_effector_model_file;
  std::string end_effector_name;

  std::vector<double> EndEffectorTransW;
  std::vector<double> EndEffectorTransKp;
  std::vector<double> EndEffectorTransKd;
  std::vector<double> EndEffectorRotW;
  std::vector<double> EndEffectorRotKp;
  std::vector<double> EndEffectorRotKd;
  double MidLinkW;
  double MidLinkKp;
  double MidLinkKd;

  Eigen::MatrixXd W_ee_translation;
  Eigen::MatrixXd Kp_ee_translation;
  Eigen::MatrixXd Kd_ee_translation;
  Eigen::MatrixXd W_ee_rotation;
  Eigen::MatrixXd Kp_ee_rotation;
  Eigen::MatrixXd Kd_ee_rotation;
  Eigen::MatrixXd W_mid_link;
  Eigen::MatrixXd Kp_mid_link;
  Eigen::MatrixXd Kd_mid_link;

  double end_effector_acceleration;
  bool enforce_acceleration_constraints;
  bool cancel_gravity_compensation;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(osc_gains));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
    a->Visit(DRAKE_NVP(franka_state_channel));
    a->Visit(DRAKE_NVP(target_cartesian_pose_channel));
    a->Visit(DRAKE_NVP(target_cartesian_pose_trajectory_channel));
    a->Visit(DRAKE_NVP(end_effector_model_file));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(franka_input_channel));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(enforce_acceleration_constraints));
    a->Visit(DRAKE_NVP(cancel_gravity_compensation));

    a->Visit(DRAKE_NVP(EndEffectorTransW));
    a->Visit(DRAKE_NVP(EndEffectorTransKp));
    a->Visit(DRAKE_NVP(EndEffectorTransKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));
    a->Visit(DRAKE_NVP(MidLinkW));
    a->Visit(DRAKE_NVP(MidLinkKp));
    a->Visit(DRAKE_NVP(MidLinkKd));

    W_ee_translation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorTransW.data(), 3, 3);
    Kp_ee_translation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorTransKp.data(), 3, 3);
    Kd_ee_translation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorTransKd.data(), 3, 3);
    W_ee_rotation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotW.data(), 3, 3);
    Kp_ee_rotation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKp.data(), 3, 3);
    Kd_ee_rotation = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->EndEffectorRotKd.data(), 3, 3);
    W_mid_link = this->MidLinkW * MatrixXd::Identity(7, 7);
    Kp_mid_link = this->MidLinkKp * MatrixXd::Identity(7, 7);
    Kd_mid_link = this->MidLinkKd * MatrixXd::Identity(7, 7);
  }
};
}  // namespace controllers
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib