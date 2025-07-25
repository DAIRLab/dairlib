#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

/**
 * @brief Generates force trajectories for the end effector.
 *
 * This system manages a finite state machine that controls when to apply
 * force trajectories to the end effector. It switches between different
 * control modes based on radio input and trajectory availability, with
 * optional message-based ignoring of initial messages.
 */
class EndEffectorForceTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructor. Declares input/output ports and initializes controller state.
   * @param ignore_messages_count Number of messages to ignore before applying force trajectories.
   */
  EndEffectorForceTrajectoryGenerator(int ignore_messages_count = 0);

  /**
   * @brief Returns the input port for the force trajectory.
   */
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }

  /**
   * @brief Returns the input port for the radio signal.
   */
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

 private:
  /**
   * @brief Updates the controller switch state and message counter based on radio input.
   */
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /**
   * @brief Calculates the output force trajectory based on controller state and message count.
   */
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::DiscreteStateIndex controller_switch_index_; ///< FSM state for controller switching.
  drake::systems::DiscreteStateIndex messages_processed_index_; ///< Counter for processed messages.

  drake::systems::InputPortIndex trajectory_port_; ///< Input port for force trajectory.
  drake::systems::InputPortIndex radio_port_;      ///< Input port for radio signal.
  
  int ignore_message_count_; ///< Number of messages to ignore before applying trajectories.
};

}  // namespace dairlib
