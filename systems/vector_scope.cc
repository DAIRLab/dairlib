#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/event.h"
#include "systems/vector_scope.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::kVectorValued;
using drake::systems::EventStatus;

VectorScope::VectorScope(int size, std::string debugMessage) : msg_(debugMessage) {

    this->DeclareVectorInputPort("debug: ", BasicVector<double>(size));
    this->DeclarePeriodicPublishEvent(0.01, 0.0, &VectorScope::PrintOutput);
}

EventStatus VectorScope::PrintOutput(const Context<double>& context) const {
    auto val = this->EvalVectorInput(context, 0)->get_value();
    std::cout << msg_ << std::endl;
    std::cout << val << std::endl;
    // std::cout << "velocities:" << std::endl;
    // std::cout << val.tail(7) << std::endl;

    return EventStatus::Succeeded();
}

} // namespace systems
} // namespace dairlib
