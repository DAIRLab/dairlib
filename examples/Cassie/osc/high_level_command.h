#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

/// `HighLevelCommand` calculates desired velocity (~in local frame) and desired
/// horizontal velocity (in local frame) of Cassie's pelvis.
///
/// Control philosophy for yaw angle:
///   If the current center of mass (com) position is close to the target
///   position, then the desired pevlis yaw is the current yaw, y_c.
///   On the other hand, if the com is far from target position, then the
///   desired yaw is y_t, the angle between global x axis and the line from com
///   to target position.
///
/// We use logistic function to implement the weighting for the current position
/// y_c and y_t.
/// Logistic function = 1 / (1 +exp(-params_1*(x-params_2)))
/// Function visualization: https://www.desmos.com/calculator/agxuc5gip8
/// As an example, the function 1/(1+exp(-5*(x-1))) outputs 0.0007 when x = 0
///                                                         0.5    when x = 1
///                                                         0.9993 when x = 2
///
/// The desired velocities are derived based on PD position control.
///
/// Input:
///  - State of the robot
///
/// Output:
///  - Desired yaw velocity (a 1D Vector).
///  - Desired horizontal velocity (a 2D Vector).
///
/// Assumption: the roll and pitch angles are close to 0.
/// Requirement: quaternion floating-based Cassie only
class HighLevelCommand : public drake::systems::LeafSystem<double> {
 public:
  /// Constructor that computes the desired yaw and translational velocities
  /// according to radio commands
  /// @param vel_scale_rot Scaling factor that scales the range of commanded yaw
  /// velocities according to [-vel_scale_rot, vel_scale_rot]
  /// @param vel_scale_trans Scaling factor that scales the range of commanded
  /// translational velocities  (saggital and lateral)according to
  /// [-vel_scale_trans, vel_scale_trans]
  ///
  /// Designed to be used with hardware
  HighLevelCommand(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   double vel_scale_rot, double vel_scale_trans_sagittal,
                   double vel_scale_trans_lateral, double stick_filter_dt=0.0);
  /// Constructor that computes the desired yaw and translational velocities
  /// according to a global target position
  ///
  /// Designed to be used in simulation
  HighLevelCommand(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context, double kp_yaw,
                   double kd_yaw, double vel_max_yaw, double kp_pos_sagittal,
                   double kd_pos_sagittal, double vel_max_sagittal,
                   double kp_pos_lateral, double kd_pos_lateral,
                   double vel_max_lateral, double target_pos_offset,
                   const Eigen::Vector2d& global_target_position,
                   const Eigen::Vector2d& params_of_no_turning);

  // Input/output ports
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_yaw() const {
    return this->get_output_port(yaw_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_xy() const {
    return this->get_output_port(xy_port_);
  }

 private:
  HighLevelCommand(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context);

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  Eigen::VectorXd CalcCommandFromTargetPosition(
      const drake::systems::Context<double>& context) const;

  void CopyHeadingAngle(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const;

  void CopyDesiredHorizontalVel(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;
  bool use_radio_command_;

  // Port index
  int state_port_;
  int yaw_port_;
  int xy_port_;
  int radio_port_ = -1;

  // Indices for the discrete states of this leafsystem
  drake::systems::DiscreteStateIndex des_vel_idx_;

  // Rotation control (yaw) parameters
  double kp_yaw_;
  double kd_yaw_;
  double vel_max_yaw_;

  // Position control (sagital plane) parameters
  double kp_pos_sagittal_;
  double kd_pos_sagittal_;
  double vel_max_sagittal_;
  double target_pos_offset_;  // Due to steady state error

  // Position control (frontal plane) parameters
  double kp_pos_lateral_;
  double kd_pos_lateral_;
  double vel_max_lateral_;

  // Other parameters
  Eigen::Vector2d global_target_position_;
  Eigen::Vector2d params_of_no_turning_;
  double vel_scale_rot_;
  double vel_scale_trans_sagittal_;
  double vel_scale_trans_lateral_;
  double stick_filter_dt_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
