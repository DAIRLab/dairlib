#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/plate-balancing/systems/franka_kinematics_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Computes kinematic state and input for the Franka arm and object.
 *
 * Outputs a combined state vector and input vector for use in learning/control.
 */
class FrankaKinematics : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports and initializes context.
   * @param franka_plant The Franka robot plant.
   * @param franka_context The context for the Franka plant.
   * @param object_plant The object (plate) plant.
   * @param object_context The context for the object plant.
   * @param end_effector_name Name of the end effector body.
   * @param object_name Name of the object body.
   * @param include_end_effector_orientation Whether to include orientation in
   * output.
   */
  explicit FrankaKinematics(
      const drake::multibody::MultibodyPlant<double>& franka_plant,
      drake::systems::Context<double>* franka_context,
      const drake::multibody::MultibodyPlant<double>& object_plant,
      drake::systems::Context<double>* object_context,
      const std::string& end_effector_name, const std::string& object_name,
      bool include_end_effector_orientation);

  /// Input/output port accessors
  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_input() const {
    return this->get_output_port(lcs_input_port_);
  }

 private:
  /// Computes the combined state vector for learning/control.
  void ComputeLCSState(const drake::systems::Context<double>& context,
                       FrankaKinematicsVector<double>* lcs_state) const;
  /// Computes the input vector for learning/control.
  void ComputeLCSInput(const drake::systems::Context<double>& context,
                       drake::systems::BasicVector<double>* lcs_input) const;

  drake::systems::InputPortIndex
      franka_state_port_;  ///< Input port for Franka state.
  drake::systems::InputPortIndex
      object_state_port_;  ///< Input port for object state.
  drake::systems::OutputPortIndex
      lcs_state_port_;  ///< Output port for combined state.
  drake::systems::OutputPortIndex lcs_input_port_;  ///< Output port for input.

  int num_end_effector_positions_;
  int num_object_positions_;
  int num_end_effector_velocities_;
  int num_object_velocities_;

  const drake::multibody::MultibodyPlant<double>& franka_plant_;
  drake::systems::Context<double>* franka_context_;
  const drake::multibody::MultibodyPlant<double>& object_plant_;
  drake::systems::Context<double>* object_context_;
  const drake::multibody::Frame<double>& world_;
  std::string end_effector_name_;
  std::string object_name_;
  const bool include_end_effector_orientation_;
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
