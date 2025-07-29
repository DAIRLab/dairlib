#include "end_effector_position.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

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

EndEffectorTrajectoryGenerator::EndEffectorTrajectoryGenerator(
    const Eigen::Vector3d& neutral_pose, int ignore_messages_count)
    : ignore_message_count_(ignore_messages_count) {
  // Declare input ports for trajectory and radio control.
  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory", drake::Value<PiecewisePolynomial<double>>())
          .get_index();
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();

  // Declare output port for the end effector trajectory.
  PiecewisePolynomial<double> empty_pp_traj(neutral_pose);
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("end_effector_trajectory", traj_inst,
                                  &EndEffectorTrajectoryGenerator::CalcTraj);

  // Use discrete state for message counter
  messages_processed_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));

  // Declare discrete update event to increment message counter
  DeclareForcedDiscreteUpdateEvent(
      &EndEffectorTrajectoryGenerator::UpdateMessageCounter);
}

void EndEffectorTrajectoryGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& neutral_pose, double x_scale, double y_scale,
    double z_scale) {
  neutral_pose_ = neutral_pose;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

// Updates the message counter for ignoring initial messages
EventStatus EndEffectorTrajectoryGenerator::UpdateMessageCounter(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Increment the message counter if we haven't reached the ignore limit
  int current_count = static_cast<int>(
      context.get_discrete_state(messages_processed_index_)[0]);
  if (current_count < ignore_message_count_) {
    discrete_state->get_mutable_value(messages_processed_index_)[0] =
        current_count + 1;
  }
  return EventStatus::Succeeded();
}

void EndEffectorTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  auto trajectory_input = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, trajectory_port_);
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // Check how many messages we've processed (read-only access)
  int messages_processed = static_cast<int>(
      context.get_discrete_state(messages_processed_index_)[0]);
  bool should_ignore_trajectory = messages_processed < ignore_message_count_;

  if (radio_out->value()[14] || should_ignore_trajectory) {
    // Remote control active: use neutral pose or offset by radio input.
    PiecewisePolynomial<double> result;
    VectorXd y_0 = neutral_pose_;
    if (radio_out->value()[10]) {
      y_0(0) += radio_out->value()[0] * x_scale_;
      y_0(1) += radio_out->value()[1] * y_scale_;
      y_0(2) += radio_out->value()[2] * z_scale_;
    }
    result = drake::trajectories::PiecewisePolynomial<double>(y_0);
    *casted_traj = result;
  } else if (!trajectory_input->empty()) {
    // Use provided trajectory input.
    *casted_traj = *trajectory_input;
  }
}

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
