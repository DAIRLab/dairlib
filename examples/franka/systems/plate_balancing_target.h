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

  void SetRemoteControlParameters(const Eigen::Vector3d& first_target,
                                  const Eigen::Vector3d& second_target,
                                  const Eigen::Vector3d& third_target,
                                  double x_scale, double y_scale,
                                  double z_scale);

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

  drake::systems::DiscreteStateIndex reached_first_target_idx_;
  double end_effector_thickness_;
  Eigen::Vector3d first_target_;
  Eigen::Vector3d second_target_;
  Eigen::Vector3d third_target_;
  double target_threshold_;
  double x_scale_;
  double y_scale_;
  double z_scale_;
};

}  // namespace systems
}  // namespace dairlib