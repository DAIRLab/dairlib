#include "examples/Cassie/osc/linear_config_mux.h"

#include <math.h>

#include <string>

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;

namespace dairlib {
namespace cassie {
namespace osc {

LinearConfigMux::LinearConfigMux(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_states_ = num_positions_ + num_velocities_;
  num_actuators_ = plant.num_actuators();

  // Input/Output Setup
  desired_state_port_ =
      this->DeclareVectorInputPort(
              BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();
  gains_port_ = this
                    ->DeclareVectorInputPort(BasicVector<double>(
                        num_actuators_ * (num_positions_ + num_velocities_)))
                    .get_index();
  this->DeclareVectorOutputPort(
      systems::LinearConfig(plant.num_positions() + plant.num_velocities(),
                            plant.num_actuators()),
      &LinearConfigMux::SetConfig);
}

void LinearConfigMux::SetConfig(const drake::systems::Context<double>& context,
                                systems::LinearConfig* output) const {
  // Read in desired state
  auto desired_state =
      this->EvalVectorInput(context, desired_state_port_)->get_value();

  // Read in gains
  auto K_vector = this->EvalVectorInput(context, gains_port_)->get_value();
  // Reorganize into gain matrix
  MatrixXd K(num_actuators_, num_states_);
  for (int i = 0; i < num_actuators_; i++) {
    K.col(i) = K_vector.segment(i * num_states_, num_states_);
  }

  output->SetK(K);
  output->SetDesiredState(desired_state);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
