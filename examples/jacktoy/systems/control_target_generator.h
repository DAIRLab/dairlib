#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"


#define PI 3.14159265359

// Nominal quaternions for the object.
#define QUAT_ALL_DOWN Eigen::Quaterniond(0.0572597, 0.544881, 0.70131284, -0.4561384)
#define QUAT_RED_DOWN Eigen::Quaterniond(-0.07147426, -0.35026577, 0.2979431, 0.88547164)
#define QUAT_GREEN_DOWN Eigen::Quaterniond(-0.61355525, -0.01070382, 0.45985836, -0.64279854)
#define QUAT_BLUE_DOWN Eigen::Quaterniond(0.34215894, 0.88709414, -0.04769839, 0.3072389)
#define QUAT_ALL_UP Eigen::Quaterniond(-0.36863035, 0.16087885, 0.43073285, 0.80815524)
#define QUAT_RED_UP Eigen::Quaterniond(-0.3697159, 0.87841034, 0.13167445, 0.2733237)
#define QUAT_GREEN_UP Eigen::Quaterniond(0.38224292, 0.87133694, 0.17296787, -0.25562137)
#define QUAT_BLUE_UP Eigen::Quaterniond(-0.44183248, 0.4439077, 0.12046596, -0.77094716)

namespace dairlib {
namespace systems {

class TargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  TargetGenerator(
      const drake::multibody::MultibodyPlant<double>& object_plant);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_end_effector_target() const {
    return this->get_output_port(end_effector_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_object_target()
      const {
    return this->get_output_port(object_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_object_velocity_target()
  const {
    return this->get_output_port(object_velocity_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_object_final_target()
  const {
    return this->get_output_port(object_final_target_port_);
  }

  void SetRemoteControlParameters(
    const int& trajectory_type,
    const bool& use_changing_final_goal,
    const double& traj_radius,
    const double& x_c,
    const double& y_c,
    const double& lead_angle,
    const Eigen::VectorXd& target_object_position,
    const Eigen::VectorXd& target_object_orientation,
    const double& step_size,
    const double& start_point_x,
    const double& start_point_y,
    const double& end_point_x,
    const double& end_point_y,
    const double& lookahead_step_size,
    const double& lookahead_angle,
    const double& angle_err_to_vel_factor,
    const double& max_step_size,
    const double& ee_goal_height,
    const double& object_half_width,
    const double& position_success_threshold,
    const double& orientation_success_threshold,
    const Eigen::VectorXd& random_goal_x_limits,
    const Eigen::VectorXd& random_goal_y_limits,
    const double& resting_object_height);

 private:
  void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target) const;
  void CalcObjectTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;
  void CalcObjectVelocityTarget(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* target) const;
  void OutputObjectFinalTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;
  void SetRandomizedTargetFinalObjectPosition() const;
  void SetRandomizedTargetFinalObjectOrientation() const;
  // drake::systems::EventStatus DiscreteVariableUpdate(
  //     const drake::systems::Context<double>& context,
  //     drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex object_target_port_;
  drake::systems::OutputPortIndex object_velocity_target_port_;
  drake::systems::OutputPortIndex object_final_target_port_;

  int trajectory_type_;
  bool use_changing_final_goal_;
  double traj_radius_;
  double x_c_;
  double y_c_;
  double lead_angle_;
  mutable Eigen::VectorXd target_final_object_position_;
  mutable Eigen::VectorXd target_final_object_orientation_;
  double step_size_;
  double start_point_x_;
  double start_point_y_;
  double end_point_x_;
  double end_point_y_;
  double lookahead_step_size_;
  double lookahead_angle_;
  double angle_err_to_vel_factor_;
  double max_step_size_;
  double ee_goal_height_;
  double object_half_width_;
  double position_success_threshold_;
  double orientation_success_threshold_;
  Eigen::VectorXd random_goal_x_limits_;
  Eigen::VectorXd random_goal_y_limits_;
  double resting_object_height_;
};

}  // namespace systems
}  // namespace dairlib