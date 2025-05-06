/** Each message that needs to be sent calls an instance of this generic sender 
 * passes the data to be sent and a corresponding builder function that fills 
 * builds the lcm message object. 
 */


#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <iostream>


namespace dairlib {
namespace systems {

template <typename msgType,             // LCM struct to publish
          typename inputType,        // C++ type to be sent
          void (*BuildFunc)(const inputType&, msgType*)>  // Function to build the lcm message
class GenericSender final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericSender)

  GenericSender(const std::string& name = "GenericSender") {
    this->set_name(name);
    input_port_ = this->DeclareAbstractInputPort(
        "input", drake::Value<inputType>{}).get_index();
    output_port_ = this->DeclareAbstractOutputPort(
        "output", msgType{}, &GenericSender::Output).get_index();
  }

 private:
  void Output(const drake::systems::Context<double>& context, msgType* output) const {
    const auto& input = this->get_input_port(input_port_).template Eval<inputType>(context);
    BuildFunc(input, output);
    output->utime = context.get_time() * 1e6;
  }

  int input_port_;
  int output_port_;
};

}  // namespace systems
}  // namespace dairlib
