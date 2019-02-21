#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"
#include "dairlib/lcmt_cassie_in.hpp"

namespace dairlib {
namespace systems {

/// @file This file contains LCM parsers for the native Cassie message structs
class CassieInputSender : public drake::systems::LeafSystem<double> {
 public:
  CassieInputSender();

 private:
  void Output(const drake::systems::Context<double>& context,
                    lcmt_cassie_in* output) const;
};

}  // namespace systems
}  // namespace dairlib
