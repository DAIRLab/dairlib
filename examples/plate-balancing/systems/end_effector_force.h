#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/systems/framework/leaf_system.h>

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @class EndEffectorForceTrajectoryGenerator
 * @brief Generates force trajectories for the end effector based on input
 *        trajectories and radio commands.
 *
 * This system switches between different force trajectories depending on the
 * state of the controller and radio input.
 */
class EndEffectorForceTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports and discrete state.
   */
  EndEffectorForceTrajectoryGenerator(int ignore_messages_count = 0);

  /**
   * @brief Returns the input port for the force trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }

  /**
   * @brief Returns the input port for the radio command vector.
   */
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

 private:
  /**
   * @brief Updates the controller switch state based on radio and trajectory
   * input.
   * @param context The system context.
   * @param discrete_state The mutable discrete state to update.
   * @return EventStatus indicating success or failure.
   */
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /**
   * @brief Calculates the output force trajectory based on the current state
   * and inputs.
   * @param context The system context.
   * @param traj The output trajectory to populate.
   */
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::DiscreteStateIndex
      controller_switch_index_;  ///< Index for controller switch state.
  drake::systems::DiscreteStateIndex
      messages_processed_index_;  ///< Counter for processed messages.
  drake::systems::InputPortIndex
      trajectory_port_;  ///< Input port index for trajectory.
  drake::systems::InputPortIndex radio_port_;  ///< Input port index for radio.

  int ignore_message_count_;  ///< Number of messages to ignore before applying
                              ///< trajectories.
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
