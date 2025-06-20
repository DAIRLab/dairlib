#pragma once

#include <string>
#include <vector>
#include <dairlib/lcmt_robot_output.hpp>
#include <drake/lcmt_panda_command.hpp>

#include "common/find_resource.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

const static int kNumFrankaJoints = 7;

class FrankaStateOutTranslator : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaStateOutTranslator(std::vector<std::string> joint_position_names,
                                    std::vector<std::string> joint_velocity_names,
                                    std::vector<std::string> joint_actuator_names);

  const drake::systems::InputPort<double>& get_input_port_panda_status() const {
    return this->get_input_port(panda_status_);
  }
  const drake::systems::OutputPort<double>& get_output_port_robot_state()
      const {
    return this->get_output_port(state_output_);
  }

 private:
  void OutputFrankaState(const drake::systems::Context<double>& context,
                         dairlib::lcmt_robot_output* output) const;

  drake::systems::InputPortIndex panda_status_;
  drake::systems::OutputPortIndex state_output_;
  mutable std::chrono::time_point<std::chrono::steady_clock> start_;
};

class FrankaEffortsInTranslator : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaEffortsInTranslator();

  const drake::systems::InputPort<double>& get_input_port_efforts() const {
    return this->get_input_port(robot_input_);
  }
  const drake::systems::InputPort<double>& get_input_port_panda_status() const {
    return this->get_input_port(panda_status_);
  }
  const drake::systems::OutputPort<double>& get_output_port_panda_command()
      const {
    return this->get_output_port(franka_command_output_);
  }

 private:
  void OutputFrankaCommand(const drake::systems::Context<double>& context,
                         drake::lcmt_panda_command* output) const;

  drake::systems::InputPortIndex robot_input_;
  drake::systems::InputPortIndex panda_status_;
  drake::systems::OutputPortIndex franka_command_output_;
};

}  // namespace systems
}  // namespace dairlib
