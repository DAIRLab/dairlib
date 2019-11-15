#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/systems/lcm/serializer.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"

namespace dairlib {
namespace systems {

class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  LcmDrivenLoop(drake::lcm::DrakeLcm* lcm_local,
                drake::systems::DiagramBuilder<double>* builder,
                const drake::systems::LeafSystem<double>* first_leafsystem);

 private:
  drake::lcm::DrakeLcm* lcm_local_;
  drake::systems::DiagramBuilder<double>* builder_;
  const drake::systems::LeafSystem<double>* first_leafsystem_;
};

}  // namespace systems
}  // namespace dairlib