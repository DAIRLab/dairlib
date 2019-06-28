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

/*

  A system that continuously prints out the (numerical) value of the
  attached input port to the console.
  @param size       Size of the input vector
  @param debug      Message the message that's printed out before the number
  @param message    Rate the rate at which messages are printed

*/
VectorScope::VectorScope(int size, std::string debugMessage, double publishRate)
    : msg_(debugMessage) {

    this->DeclareVectorInputPort("debug: ", BasicVector<double>(size));
    this->DeclarePeriodicPublishEvent(publishRate, 0.0, &VectorScope::PrintOutput);
}

EventStatus VectorScope::PrintOutput(const Context<double>& context) const {
    auto val = this->EvalVectorInput(context, 0)->get_value();
    std::cout << msg_ << std::endl;
    std::cout << val << std::endl;
    return EventStatus::Succeeded();
}

} // namespace systems
} // namespace dairlib
