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
    const drake::multibody::MultibodyPlant<double>& plant,
    std::vector<std::pair<int, MatrixXd>> gains_and_fsm_state) {
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
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  this->DeclareVectorOutputPort(
      systems::LinearConfig(plant.num_positions() + plant.num_velocities(),
                            plant.num_actuators()),
      &LinearConfigMux::SetConfig);

  //  K_left_stance_ = K_left_stance;
  //  K_right_stance_ = K_right_stance;
  gains_and_fsm_state_ = gains_and_fsm_state;
}

void LinearConfigMux::SetConfig(const drake::systems::Context<double>& context,
                                systems::LinearConfig* output) const {
  // Read in desired state
  const systems::TimestampedVector<double>* desired_state =
      (systems::TimestampedVector<double>*)this->EvalVectorInput(
          context, desired_state_port_);

  // Read in finite state machine
  auto fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()(0);

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

  if (gains_and_fsm_state_.at(0).first == fsm_state) {
    output->SetK(gains_and_fsm_state_.at(0).second);
    prev_fsm_state_ = fsm_state;
  } else if (gains_and_fsm_state_.at(1).first == fsm_state) {
    output->SetK(gains_and_fsm_state_.at(1).second);
    prev_fsm_state_ = fsm_state;
  } else if (prev_fsm_state_ == -std::numeric_limits<double>::infinity()) {
    // Assign an arbitrary one
    output->SetK(gains_and_fsm_state_.at(0).second);
  }

  output->SetDesiredState(desired_state->get_data());
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
