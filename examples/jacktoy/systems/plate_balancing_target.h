#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class PlateBalancingTargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  PlateBalancingTargetGenerator(
      const drake::multibody::MultibodyPlant<double>& object_plant,
      double end_effector_thickness, double target_threshold = 0.075);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_tray_state() const {
    return this->get_input_port(tray_state_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_end_effector_target() const {
    return this->get_output_port(end_effector_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_tray_target()
      const {
    return this->get_output_port(tray_target_port_);
  }

  void SetRemoteControlParameters(const int& trajectory_type, const double& traj_radius,
    const double& x_c, const double& y_c, const double& lead_angle, const double& fixed_goal_x, 
    const double& fixed_goal_y, const double& step_size, const double& start_point_x, const double& start_point_y, 
    const double& end_point_x, const double& end_point_y, const double& lookahead_step_size, const double& max_step_size);

 private:
  void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target) const;
  void CalcTrayTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex tray_state_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex tray_target_port_;

  drake::systems::DiscreteStateIndex sequence_index_;
  drake::systems::DiscreteStateIndex within_target_index_;
  drake::systems::DiscreteStateIndex time_entered_target_index_;
  double end_effector_thickness_;
  Eigen::Vector3d first_target_;
  Eigen::Vector3d second_target_;
  Eigen::Vector3d third_target_;
  const double delay_at_top_ = 3.0;
  double target_threshold_;
  double x_scale_;
  double y_scale_;
  double z_scale_;
};

}  // namespace systems
}  // namespace dairlib