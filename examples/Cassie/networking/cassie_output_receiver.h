#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_cassie_out.hpp"
#include "examples/Cassie/datatypes/cassie_out_t.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// @file This file contains LCM parsers for the native Cassie message structs
class CassieOutputReceiver : public drake::systems::LeafSystem<double> {
 public:
  CassieOutputReceiver();

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  cassie_out_t* output) const;
};

}  // namespace systems
}  // namespace dairlib
