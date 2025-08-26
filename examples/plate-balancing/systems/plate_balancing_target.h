#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/state_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Generates target positions and velocities for the plate and end
 * effector.
 *
 * This system manages a finite state machine to sequence through a set of
 * targets for the plate and end effector, with optional remote control scaling.
 */
class PlateBalancingTargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports and initializes targets.
   * @param object_plant The multibody plant for the object (plate).
   * @param end_effector_thickness The thickness of the end effector.
   * @param target_threshold Threshold for considering the target reached.
   */
  PlateBalancingTargetGenerator(
      const drake::multibody::MultibodyPlant<double>& object_plant,
      double end_effector_thickness, double target_threshold = 0.075);

  /// Input/output port accessors
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
  const drake::systems::OutputPort<double>&
  get_output_port_tray_velocity_target() const {
    return this->get_output_port(tray_velocity_target_port_);
  }

  /**
   * @brief Sets the target positions and scaling for remote control.
   * @param first_target First target position.
   * @param second_target Second target position.
   * @param third_target Third target position.
   * @param x_scale Scaling for x input.
   * @param y_scale Scaling for y input.
   * @param z_scale Scaling for z input.
   */
  void SetRemoteControlParameters(const Eigen::Vector3d& first_target,
                                  const Eigen::Vector3d& second_target,
                                  const Eigen::Vector3d& third_target,
                                  double x_scale, double y_scale,
                                  double z_scale);

 private:
  /// Calculates the end effector target output.
  void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target) const;
  /// Calculates the tray target output.
  void CalcTrayTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;
  /// Calculates the tray velocity target output.
  void CalcTrayVelocityTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  /// Updates the FSM state.
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex radio_port_;  ///< Input port for radio.
  drake::systems::InputPortIndex
      tray_state_port_;  ///< Input port for tray state.
  drake::systems::OutputPortIndex
      end_effector_target_port_;  ///< Output port for end effector target.
  drake::systems::OutputPortIndex
      tray_target_port_;  ///< Output port for tray target.
  drake::systems::OutputPortIndex
      tray_velocity_target_port_;  ///< Output port for tray velocity target.

  drake::systems::DiscreteStateIndex sequence_index_;  ///< FSM sequence index.
  drake::systems::DiscreteStateIndex
      within_target_index_;  ///< FSM within-target index.
  drake::systems::DiscreteStateIndex
      time_entered_target_index_;    ///< FSM time index.
  double end_effector_thickness_;    ///< End effector thickness.
  Eigen::Vector3d first_target_;     ///< First target position.
  Eigen::Vector3d second_target_;    ///< Second target position.
  Eigen::Vector3d third_target_;     ///< Third target position.
  const double delay_at_top_ = 3.0;  ///< Delay at top target.
  double target_threshold_;          ///< Threshold for target reach.
  double x_scale_;                   ///< Scaling for x remote input.
  double y_scale_;                   ///< Scaling for y remote input.
  double z_scale_;                   ///< Scaling for z remote input.
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib