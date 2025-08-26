#pragma once

#include "systems/controllers/osc/osc_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief OSC controller configuration for the plate balancing example.
 *
 * Contains parameters for the operational space controller (OSC), including
 * model paths, tool frames, gains, and control options.
 */
struct OSCControllerConfig : OSCGains {
  std::string franka_model;        ///< Path to the Franka robot model file.
  std::string end_effector_model;  ///< Path to the end effector model file.
  std::string end_effector_name;   ///< Name of the end effector in the model.
  Eigen::VectorXd tool_attachment_frame;  ///< 3D vector specifying the tool's
                                          ///< attachment frame.
  double end_effector_acceleration;       ///< Maximum acceleration for the end
                                          ///< effector.
  bool track_end_effector_orientation;    ///< Whether to track end effector
                                          ///< orientation.
  bool
      cancel_gravity_compensation;  ///< Whether to cancel gravity compensation.
  bool enforce_acceleration_constraints;  ///< Whether to enforce acceleration
                                          ///< constraints.
  bool publish_debug_info;           ///< Whether to publish debug information.
  Eigen::VectorXd neutral_position;  ///< Neutral joint positions for the robot.
  std::optional<int> ignore_messages_count;     ///< Number of messages to ignore at startup.
  double x_scale;                    ///< Scaling factor for x position control.
  double y_scale;                    ///< Scaling factor for y position control.
  double z_scale;                    ///< Scaling factor for z position control.
  double w_elbow;                    ///< Weight for elbow control.
  double elbow_kp;                   ///< Proportional gain for elbow.
  double elbow_kd;                   ///< Derivative gain for elbow.
  std::vector<double> EndEffectorW;  ///< End effector weights.
  std::vector<double> EndEffectorKp;    ///< End effector proportional gains.
  std::vector<double> EndEffectorKd;    ///< End effector derivative gains.
  std::vector<double> EndEffectorRotW;  ///< End effector rotation weights.
  std::vector<double>
      EndEffectorRotKp;  ///< End effector rotation proportional gains.
  std::vector<double>
      EndEffectorRotKd;  ///< End effector rotation derivative gains.

  // Controller gain and weight matrices, constructed from the corresponding
  // vector parameters after deserialization. Used for OSC cost and feedback.
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
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(end_effector_acceleration));
    a->Visit(DRAKE_NVP(track_end_effector_orientation));
    a->Visit(DRAKE_NVP(cancel_gravity_compensation));
    a->Visit(DRAKE_NVP(enforce_acceleration_constraints));
    a->Visit(DRAKE_NVP(publish_debug_info));
    a->Visit(DRAKE_NVP(neutral_position));
    a->Visit(DRAKE_NVP(ignore_messages_count));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(w_elbow));
    a->Visit(DRAKE_NVP(elbow_kp));
    a->Visit(DRAKE_NVP(elbow_kd));
    a->Visit(DRAKE_NVP(EndEffectorW));
    a->Visit(DRAKE_NVP(EndEffectorKp));
    a->Visit(DRAKE_NVP(EndEffectorKd));
    a->Visit(DRAKE_NVP(EndEffectorRotW));
    a->Visit(DRAKE_NVP(EndEffectorRotKp));
    a->Visit(DRAKE_NVP(EndEffectorRotKd));
    a->Visit(DRAKE_NVP(LambdaEndEffectorW));

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

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib