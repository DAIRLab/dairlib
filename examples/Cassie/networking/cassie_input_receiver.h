#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_cassie_in.hpp"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// @file This file contains LCM parsers for the native Cassie message structs
class CassieInputReceiver : public drake::systems::LeafSystem<double> {
 public:
  CassieInputReceiver();

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  cassie_user_in_t* cassie_in) const;

  std::vector<int> u_vector_to_user_index_;
};

}  // namespace systems
}  // namespace dairlib
