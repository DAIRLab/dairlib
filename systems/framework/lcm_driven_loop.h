#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/systems/lcm/serializer.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {
namespace systems {

class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  LcmDrivenLoop(drake::lcm::DrakeLcm* lcm_local,
                drake::systems::DiagramBuilder<double>* builder,
                const drake::systems::LeafSystem<double>* first_leafsystem);

  // Set channel name of the switch
  void SetSwitchChannelName(std::string switch_channel);
  // Set channel names of the inputs
  void SetMultipleInputChannelName(std::vector<std::string> input_channels);

  // Start simulating the diagram
  void Simulate(double end_time =
      std::numeric_limits<double>::infinity());

 private:
  drake::lcm::DrakeLcm* lcm_local_;
  drake::systems::DiagramBuilder<double>* builder_;
  drake::systems::Diagram<double>* diagram_;
  const drake::systems::LeafSystem<double>* first_leafsystem_;

  std::unique_ptr<drake::systems::Simulator<double>> simulator_;

  drake::lcm::Subscriber<lcmt_controller_switch> input_switch_sub_;
  std::map<std::string, drake::lcm::Subscriber<dairlib::lcmt_robot_input>> name_to_sub_map_;
  std::string active_channel_;
};

}  // namespace systems
}  // namespace dairlib