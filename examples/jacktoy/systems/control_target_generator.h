#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"
#define PI 3.14159265359

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

  void SetRemoteControlParameters(const int& trajectory_type, const double& traj_radius,
    const double& x_c, const double& y_c, const double& lead_angle, const double& fixed_goal_x, 
    const double& fixed_goal_y, const double& step_size, const double& start_point_x, const double& start_point_y, 
    const double& end_point_x, const double& end_point_y, const double& lookahead_step_size, const double& max_step_size, const double& ee_goal_height, const double& object_half_width);

 private:
  void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target) const;
  void CalcObjectTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex object_target_port_;

  int trajectory_type_;
  double traj_radius_;
  double x_c_;
  double y_c_;
  double lead_angle_;
  double fixed_goal_x_;
  double fixed_goal_y_;
  double step_size_;
  double start_point_x_;
  double start_point_y_;
  double end_point_x_;
  double end_point_y_;
  double lookahead_step_size_;
  double max_step_size_;
  double ee_goal_height_;
  double object_half_width_;
};

}  // namespace systems
}  // namespace dairlib