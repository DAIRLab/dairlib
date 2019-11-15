#include "systems/framework/lcm_driven_loop.h"

namespace dairlib {
namespace systems {

using std::vector;
using std::unique_ptr;
using std::string;
using drake::AbstractValue;
using drake::Value;
using drake::lcm::DrakeLcmInterface;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::SerializerInterface;
using drake::systems::UnrestrictedUpdateEvent;
using drake::systems::CompositeEventCollection;
using drake::systems::State;

LcmDrivenLoop::LcmDrivenLoop(
    drake::lcm::DrakeLcm* lcm_local,
    drake::systems::DiagramBuilder<double>* builder,
    const drake::systems::LeafSystem<double>* first_leafsystem)
    : lcm_local_(lcm_local),
      builder_(builder),
      first_leafsystem_(first_leafsystem) {

}

}  // namespace systems
}  // namespace dairlib