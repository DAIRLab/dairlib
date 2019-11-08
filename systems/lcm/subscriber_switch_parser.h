#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_subscriber_switch.hpp"

namespace dairlib {
namespace systems {

/// This is a simple class which parses the channel (a string) from a 
/// lcmt_subscriber_switch LCM message. It's only input port is the lcm message
/// and its output is a std::string (both as Abstract types).
class SubscriberSwitchParser : public drake::systems::LeafSystem<double> {
 public:
  SubscriberSwitchParser();

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                    std::string* output) const;
};

}  // namespace systems
}  // namespace dairlib
