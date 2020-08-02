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
/// Logistic function = 1 / (1 - params_1*exp(x-params_2))
/// Function visualization: https://www.desmos.com/calculator/agxuc5gip8
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
  HighLevelCommand(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const Eigen::Vector2d& global_target_position,
                   const Eigen::Vector2d& params_of_no_turning);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::OutputPort<double>& get_yaw_output_port() const {
    return this->get_output_port(yaw_port_);
  }
  const drake::systems::OutputPort<double>& get_xy_output_port() const {
    return this->get_output_port(xy_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyHeadingAngle(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* output) const;

  void CopyDesiredHorizontalVel(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;
  Eigen::Vector2d global_target_position_;
  Eigen::Vector2d params_of_no_turning_;

//  std::unique_ptr<drake::systems::Context<double>> context_;

  // Port index
  int state_port_;
  int yaw_port_;
  int xy_port_;

  // Indices for the discrete states of this leafsystem
  drake::systems::DiscreteStateIndex prev_time_idx_;
  drake::systems::DiscreteStateIndex des_yaw_vel_idx_;
  drake::systems::DiscreteStateIndex des_horizontal_vel_idx_;

  // Rotation control (yaw) parameters
  double kp_yaw_ = 1;
  double kd_yaw_ = 0.2;
  double vel_max_yaw_ = 0.5;
  double vel_min_yaw_ = -0.5;

  // Position control (sagital plane) parameters
  double kp_pos_sagital_ = 1.0;
  double kd_pos_sagital_ = 0.2;
  double vel_max_sagital_ = 1;
  double vel_min_sagital_ = -1;       // TODO(yminchen): need to test this
  double target_pos_offset_ = -0.16;  // Due to steady state error

  // Position control (frontal plane) parameters
  double kp_pos_lateral_ = 0.5;
  double kd_pos_lateral_ = 0.1;
  double vel_max_lateral_ = 0.5;
  double vel_min_lateral_ = -0.5;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
