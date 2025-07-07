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

namespace dairlib {
namespace systems {
namespace lcmt_generators {

class RobotStateGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit RobotStateGenerator(std::vector<std::string> state_names,
                               bool is_timestamped_robot_state = false);

  const drake::systems::InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(robot_state_input_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcmt_robot_state()
      const {
    return this->get_output_port(lcmt_robot_state_output_);
  }

  static drake::systems::lcm::LcmPublisherSystem* AddLcmPublisherToBuilder(
      drake::systems::DiagramBuilder<double>& builder,
      std::vector<std::string> state_names, bool is_timestamped_robot_state,
      const drake::systems::OutputPort<double>& robot_state_input_port,
      const std::string& channel, drake::lcm::DrakeLcmInterface* lcm,
      const drake::systems::TriggerTypeSet& publish_triggers,
      double publish_period = 0.0, double publish_offset = 0.0);

 private:
  void GenerateRobotState(const drake::systems::Context<double>& context,
                          drake::lcmt_robot_state* output) const;

  drake::systems::InputPortIndex robot_state_input_;
  drake::systems::OutputPortIndex lcmt_robot_state_output_;

  int n_x_;
  bool is_timestamped_robot_state_;
};

}  // namespace lcmt_generators
}  // namespace systems
}  // namespace dairlib
