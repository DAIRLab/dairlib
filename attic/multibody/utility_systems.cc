#include "attic/multibody/utility_systems.h"

namespace dairlib {
namespace multibody {

using drake::systems::BasicVector;
using drake::systems::Context;
using dairlib::systems::OutputVector;
using Eigen::VectorXd;

StateToOutputVectorSystem::StateToOutputVectorSystem(int num_positions,
                                                     int num_velocities,
                                                     int num_efforts)
    : num_states_(num_positions + num_velocities) {
  this->DeclareVectorInputPort(
      BasicVector<double>(num_positions + num_velocities));
  this->DeclareVectorOutputPort(
      OutputVector<double>(num_positions, num_velocities, num_efforts),
      &dairlib::multibody::StateToOutputVectorSystem::CopyOut);
}

void StateToOutputVectorSystem::CopyOut(const Context<double>& context,
                                        OutputVector<double>* output) const {
  const auto state = this->EvalVectorInput(context, 0);
  const VectorXd state_vec = state->get_value();
  // Copying the state to the output
  output->SetState(state_vec.head(num_states_));
  output->set_timestamp(context.get_time());
}

}  // namespace multibody
}  // namespace dairlib
