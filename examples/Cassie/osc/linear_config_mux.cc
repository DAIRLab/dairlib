#include "examples/Cassie/osc/linear_config_mux.h"

#include <math.h>

#include <string>

#include "multibody/multibody_utils.h"
#include "systems/controllers/pd_config_lcm.h"

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
    const drake::multibody::MultibodyPlant<double>& plant, MatrixXd K) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_states_ = num_positions_ + num_velocities_;
  num_actuators_ = plant.num_actuators();

  // Input/Output Setup
  desired_state_port_ =
      this->DeclareVectorInputPort(systems::TimestampedVector<double>(
                                       num_positions_ + num_velocities_))
          .get_index();
  //  gains_port_ = this
  //                    ->DeclareVectorInputPort(BasicVector<double>(
  //                        num_actuators_ * (num_positions_ +
  //                        num_velocities_)))
  //                    .get_index();
  this->DeclareVectorOutputPort(
      systems::LinearConfig(plant.num_positions() + plant.num_velocities(),
                            plant.num_actuators()),
      &LinearConfigMux::SetConfig);

  K_ = K;
}

void LinearConfigMux::SetConfig(const drake::systems::Context<double>& context,
                                systems::LinearConfig* output) const {
  // Read in desired state
  const systems::TimestampedVector<double>* desired_state =
      (systems::TimestampedVector<double>*)this->EvalVectorInput(
          context, desired_state_port_);

  // Read in gains
  //  VectorXd K_vector = this->EvalVectorInput(context,
  //  gains_port_)->get_value();
  //  std:: cout << "In LinearConfigMux: K_vector = " << K_vector.transpose()
  //     << std::endl;
  //  // Reorganize into gain matrix
  //  MatrixXd K(num_actuators_, num_states_);
  //  for (int i = 0; i < num_actuators_; i++) {
  //    K.col(i) = K_vector.segment(i * num_states_, num_states_);
  //  }
  //  std:: cout << "In LinearConfigMux: K = " << K << std::endl;

  output->SetK(K_);
  output->SetDesiredState(desired_state->get_data());
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
