#pragma once

#include <string>
#include <vector>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>

#include "common/find_resource.h"

#include "drake/lcmt_robot_state.hpp"
#include "drake/systems/framework/leaf_system.h"

/**
 * @file
 * Defines the RobotStateGenerator class, which converts a vector of robot state
 * values into an lcmt_robot_state message for LCM communication.
 */

namespace dairlib {
namespace systems {
namespace lcmt_systems {

/**
 * @class RobotStateGenerator
 * @brief Drake LeafSystem that generates an lcmt_robot_state message from a
 * vector of state values.
 *
 * This system takes as input a vector of robot state values (e.g., positions,
 * velocities, etc.) and outputs an lcmt_robot_state message suitable for LCM
 * communication. Optionally, the output message can include a timestamp.
 *
 * Typical usage involves connecting this system's output to an LCM publisher
 * system for broadcasting robot state information.
 *
 * @ingroup lcmt_systems
 */
class RobotStateGenerator : public drake::systems::LeafSystem<double> {
 public:
  /**
   * @brief Constructs a RobotStateGenerator.
   * @param state_names Names of the robot state variables (e.g., joint names).
   * @param is_timestamped_robot_state If true, the output message will include
   * a timestamp.
   */
  explicit RobotStateGenerator(std::vector<std::string> state_names,
                               bool is_timestamped_robot_state = false);

  /**
   * @brief Returns the input port for the robot state vector.
   * @return Reference to the input port expecting the robot state vector.
   */
  const drake::systems::InputPort<double>& get_input_port_robot_state()
      const {
    return this->get_input_port(robot_state_input_);
  }

  /**
   * @brief Returns the output port for the lcmt_robot_state message.
   * @return Reference to the output port producing the lcmt_robot_state
   * message.
   */
  const drake::systems::OutputPort<double>& get_output_port_lcmt_robot_state()
      const {
    return this->get_output_port(lcmt_robot_state_output_);
  }

  /**
   * @brief Adds an LCM publisher system to a DiagramBuilder for publishing
   * robot state messages.
   *
   * This static helper creates and adds an LCM publisher system to the given
   * DiagramBuilder, wiring it to the output port of a RobotStateGenerator.
   *
   * @param builder The DiagramBuilder to which the publisher will be added.
   * @param state_names Names of the robot state variables.
   * @param is_timestamped_robot_state If true, the published message will
   * include a timestamp.
   * @param robot_state_input_port The output port providing the robot state
   * message.
   * @param channel The LCM channel on which to publish.
   * @param lcm The LCM interface to use for publishing.
   * @param publish_triggers Set of triggers that determine when to publish.
   * @param publish_period Period (in seconds) for periodic publishing (default:
   * 0.0).
   * @param publish_offset Offset (in seconds) for periodic publishing (default:
   * 0.0).
   * @return Pointer to the created LcmPublisherSystem.
   */
  static drake::systems::lcm::LcmPublisherSystem* AddLcmPublisherToBuilder(
      drake::systems::DiagramBuilder<double>& builder,
      std::vector<std::string> state_names, bool is_timestamped_robot_state,
      const drake::systems::OutputPort<double>& robot_state_input_port,
      const std::string& channel, drake::lcm::DrakeLcmInterface* lcm,
      const drake::systems::TriggerTypeSet& publish_triggers,
      double publish_period = 0.0, double publish_offset = 0.0);

 private:
  /**
   * @brief Populates an lcmt_robot_state message from the input vector.
   * @param context The system context.
   * @param output The lcmt_robot_state message to populate.
   */
  void GenerateRobotState(const drake::systems::Context<double>& context,
                          drake::lcmt_robot_state* output) const;

  drake::systems::InputPortIndex
      robot_state_input_; /**< Index of the robot state input port. */
  drake::systems::OutputPortIndex
      lcmt_robot_state_output_; /**< Index of the lcmt_robot_state output port.
                                 */

  int n_x_;                         /**< Number of state variables. */
  bool is_timestamped_robot_state_; /**< Whether to include a timestamp in the
                                       output message. */
};

}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib
