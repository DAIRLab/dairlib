#include "state_vector_to_basic_vector.h"

namespace dairlib {

using systems::StateVector;

StateVectorToBasicVector::StateVectorToBasicVector(
    const drake::multibody::MultibodyPlant<double>& plant) {
  const int state_size = plant.num_positions() + plant.num_velocities();
  state_input_port_ = this->DeclareVectorInputPort(
      "x_object_filtered", StateVector<double>(plant.num_positions(),
                                                plant.num_velocities()))
      .get_index();
  state_output_port_ = this->DeclareVectorOutputPort(
      "x_object", drake::systems::BasicVector<double>(state_size),
      &StateVectorToBasicVector::CopyState)
      .get_index();
}

void StateVectorToBasicVector::CopyState(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const {
  const StateVector<double>* state =
      (StateVector<double>*)this->EvalVectorInput(context, state_input_port_);
  output->SetFromVector(state->GetState());
}

}  // namespace dairlib
