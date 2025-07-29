#include "end_effector_force.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

using Eigen::Map;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBodyFrame;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {

// Constructor: Declares input/output ports and discrete state.
EndEffectorForceTrajectoryGenerator::EndEffectorForceTrajectoryGenerator(
    int ignore_messages_count)
    : ignore_message_count_(ignore_messages_count) {
  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory", drake::Value<PiecewisePolynomial<double>>())
          .get_index();
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();

  controller_switch_index_ = this->DeclareDiscreteState(VectorXd::Ones(1));
  messages_processed_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));

  DeclareForcedDiscreteUpdateEvent(
      &EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate);
  PiecewisePolynomial<double> empty_pp_traj(Vector3d::Zero());
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort(
      "end_effector_force_trajectory", traj_inst,
      &EndEffectorForceTrajectoryGenerator::CalcTraj);
}

// Updates the controller switch state based on radio and trajectory input.
EventStatus EndEffectorForceTrajectoryGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  // Update controller switching logic and message counter.
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  auto trajectory_input = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, trajectory_port_);
  bool using_c3 = context.get_discrete_state(controller_switch_index_)[0];

  // Increment message counter if we haven't reached the ignore limit.
  int current_count = static_cast<int>(
      context.get_discrete_state(messages_processed_index_)[0]);
  if (current_count < ignore_message_count_) {
    discrete_state->get_mutable_value(messages_processed_index_)[0] =
        current_count + 1;
  }

  // Enable trajectory following when radio control is off and new trajectory
  // arrives.
  if (!using_c3 && radio_out->value()[14] == 0) {
    if (!trajectory_input->empty() &&
        (context.get_time() - trajectory_input->start_time()) < 0.04) {
      discrete_state->get_mutable_value(controller_switch_index_)[0] = 1;
    }
  }
  return EventStatus::Succeeded();
}

// Calculates the output force trajectory based on the current state and inputs.
void EndEffectorForceTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  auto trajectory_input = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, trajectory_port_);
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // Check message count for ignoring initial messages.
  int messages_processed = static_cast<int>(
      context.get_discrete_state(messages_processed_index_)[0]);
  bool should_ignore_trajectory = messages_processed < ignore_message_count_;

  if (radio_out->value()[11] || radio_out->value()[14] ||
      trajectory_input->empty() || should_ignore_trajectory) {
    *casted_traj =
        drake::trajectories::PiecewisePolynomial<double>(Vector3d::Zero());
  } else {
    if (context.get_discrete_state(controller_switch_index_)[0]) {
      *casted_traj = *trajectory_input;
    }
  }
}

// NOTE: The cast to PiecewisePolynomial<double>* could be avoided by using a
// strongly-typed output port.

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
