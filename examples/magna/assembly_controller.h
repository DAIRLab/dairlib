#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace magna {

class AssemblyController : public drake::systems::LeafSystem<double> {
 public:
  AssemblyController();

 private:
  /// Function for computing one control loop
  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
};
}  // namespace magna
}  // namespace dairlib