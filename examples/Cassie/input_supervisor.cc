#include "examples/Cassie/input_supervisor.h"

#include "systems/framework/output_vector.h"

using drake::systems::Context;
using drake::systems::DiscreteValues;

namespace dairlib {

using systems::OutputVector;
using systems::TimestampedVector;

InputSupervisor::InputSupervisor(
    const drake::multibody::MultibodyPlant<double>& plant,
    double max_joint_velocity, double update_period,
    int min_consecutive_failures, double input_limit)
    : plant_(plant),
      num_actuators_(plant_.num_actuators()),
      num_positions_(plant_.num_positions()),
      num_velocities_(plant_.num_velocities()),
      min_consecutive_failures_(min_consecutive_failures),
      max_joint_velocity_(max_joint_velocity),
      input_limit_(input_limit) {
  // Create input ports
  command_input_port_ =
      this->DeclareVectorInputPort(TimestampedVector<double>(num_actuators_))
          .get_index();
  state_input_port_ = this
                          ->DeclareVectorInputPort(OutputVector<double>(
                              num_positions_, num_velocities_, num_actuators_))
                          .get_index();

  // Create output port for commands
  command_output_port_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(num_actuators_),
                                    &InputSupervisor::SetMotorTorques)
          .get_index();

  // Create output port for status
  status_output_port_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(1),
                                    &InputSupervisor::SetStatus)
          .get_index();

  // Create error flag as discrete state
  // Store both values in single discrete vector
  DeclareDiscreteState(2);
  n_fails_index_ = 0;
  status_index_ = 1;

  // Create update for error flag
  DeclarePeriodicDiscreteUpdateEvent(update_period, 0,
                                     &InputSupervisor::UpdateErrorFlag);
}

void InputSupervisor::SetMotorTorques(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);

  bool is_error =
      context.get_discrete_state()[n_fails_index_] >= min_consecutive_failures_;

  // If there has not been an error, copy over the command.
  // If there has been an error, set the command to all zeros
  if (!is_error) {
    // If input_limit_ has been set, limit inputs to
    ///   [-input_limit_, input_limit_]
    if (input_limit_ != std::numeric_limits<double>::max()) {
      output->set_timestamp(command->get_timestamp());
      for (int i = 0; i < command->get_data().size(); i++) {
        double command_value = command->get_data()(i);
        if (command_value > input_limit_) {
          command_value = input_limit_;
        } else if (command_value < -input_limit_) {
          command_value = -input_limit_;
        }
        output->get_mutable_data()(i) = command_value;
      }
    } else {
      // Can copy entire raw vector
      output->get_mutable_value() = command->get_value();
    }
  } else {
    output->set_timestamp(command->get_timestamp());
    output->SetDataVector(Eigen::VectorXd::Zero(num_actuators_));
  }
}

void InputSupervisor::SetStatus(const Context<double>& context,
                                TimestampedVector<double>* output) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);

  output->get_mutable_value()(0) = context.get_discrete_state()[status_index_];
  if (input_limit_ != std::numeric_limits<double>::max()) {
    for (int i = 0; i < command->get_data().size(); i++) {
      double command_value = command->get_data()(i);
      if (command_value > input_limit_ || command_value < -input_limit_) {
        output->get_mutable_value()(0) += 2;
        break;
      }
    }
  }

  // Shutdown is/will soon be active (the status flag is set in a separate loop
  // from the actual motor torques so the update of the status bit could be
  // slightly off
  if (context.get_discrete_state()[n_fails_index_] >=
      min_consecutive_failures_) {
    output->get_mutable_value()(0) += 4;
  }
}

void InputSupervisor::UpdateErrorFlag(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);

  const Eigen::VectorXd& velocities = state->GetVelocities();

  if ((*discrete_state)[n_fails_index_] < min_consecutive_failures_) {
    // If any velocity is above the threshold, set the error flag
    bool is_velocity_error = (velocities.array() > max_joint_velocity_).any() ||
                             (velocities.array() < -max_joint_velocity_).any();
    if (is_velocity_error) {
      // Increment counter
      (*discrete_state)[n_fails_index_]++;
      (*discrete_state)[status_index_] = true;
      std::cout << "Error! Velocity has exceeded the threshold of "
                << max_joint_velocity_ << std::endl;
      std::cout << "Consecutive error " << (*discrete_state)[n_fails_index_]
                << " of " << min_consecutive_failures_ << std::endl;
      std::cout << "Velocity vector: " << std::endl
                << velocities << std::endl
                << std::endl;
    } else {
      // Reset counter
      (*discrete_state)[n_fails_index_] = 0;
      (*discrete_state)[status_index_] = false;
    }
  }
}

}  // namespace dairlib
