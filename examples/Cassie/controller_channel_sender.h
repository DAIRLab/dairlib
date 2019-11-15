#pragma once

#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {

class ControllerChannelSender : public drake::systems::LeafSystem<double> {
 public:
  explicit ControllerChannelSender(const std::string& channel_name);

 private:
  void Output(const drake::systems::Context<double>& context,
              dairlib::lcmt_controller_switch* output) const;
  std::string channel_name_;
};

}  // namespace dairlib
