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
class CassieOutputSender : public drake::systems::LeafSystem<double> {
 public:
  CassieOutputSender();

 private:
  void Output(const drake::systems::Context<double>& context,
              lcmt_cassie_out* output) const;
};

}  // namespace systems
}  // namespace dairlib
