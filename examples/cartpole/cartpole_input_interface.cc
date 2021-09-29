#include "cartpole_input_interface.h"
#include "systems/framework/timestamped_vector.h"

using drake::systems::Context;
using drake::systems::DiscreteValues;

using dairlib::systems::TimestampedVector;

namespace dairlib {

CartpoleInputInterface::CartpoleInputInterface() {
  unsigned int error_code = 0;
  KeyHandle_ = epos::OpenDevice(&error_code);
  DRAKE_ASSERT(error_code == 0);

  this->DeclareVectorInputPort("u_t", TimestampedVector<double>(kNumActuators));
  this->DeclarePerStepDiscreteUpdateEvent(
      &CartpoleInputInterface::SendEposCommand);
  prev_timestamp_idx_ = DeclareDiscreteState(1);
}


void CartpoleInputInterface::SetupEposDevice() {
  epos::HomeDevice(KeyHandle_);
  epos::EnableDevice(KeyHandle_);
  epos::SetCurrentControlMode(KeyHandle_);
  epos::SetCurrentByForce(KeyHandle_, 0);
}

void CartpoleInputInterface::CloseEposDevice() const {
  epos::SetCurrentByForce(KeyHandle_, 0);
  epos::CloseDevice(KeyHandle_);
}

drake::systems::EventStatus CartpoleInputInterface::SendEposCommand(
    const Context<double>& context, DiscreteValues<double>* values) const {

  const TimestampedVector<double>* command = (TimestampedVector<double>*)
      this->EvalVectorInput(context, 0);

  double force = command->GetAtIndex(0);

  if (command->get_timestamp() -
      values->get_mutable_vector()[0] > kMaxControllerDelay) {
    force = 0;
    error_flag_ = true;
    CloseEposDevice();
  }
  if (error_flag_) {
    force = 0;
    std::cout << "Controller delay too long - shutting down" << std::endl;
  }
  epos::SetCurrentByForce(KeyHandle_, force);
  return drake::systems::EventStatus::Succeeded();
}

}
