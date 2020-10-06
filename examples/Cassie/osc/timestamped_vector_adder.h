#pragma once

#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

class TimestampedVectorAdder : public drake::systems::LeafSystem<double> {
 public:
  TimestampedVectorAdder(int num_input_ports, int port_size);

 private:
  void Add(const drake::systems::Context<double>& context,
           systems::TimestampedVector<double>* output) const;

  int num_input_ports_;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
