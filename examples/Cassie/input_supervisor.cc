#include "examples/Cassie/input_supervisor.h"

#include <dairlib/lcmt_cassie_out.hpp>
#include <dairlib/lcmt_controller_failure.hpp>

#include "dairlib/lcmt_controller_switch.hpp"
#include "systems/framework/output_vector.h"

using drake::multibody::JointActuatorIndex;
using drake::systems::Context;
using drake::systems::DiscreteValues;

namespace dairlib {

using systems::OutputVector;
using systems::TimestampedVector;

InputSupervisor::InputSupervisor(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::string& initial_channel, double max_joint_velocity,
    double update_period, int min_consecutive_failures, double input_limit)
    : plant_(plant),
      num_actuators_(plant_.num_actuators()),
      num_positions_(plant_.num_positions()),
      num_velocities_(plant_.num_velocities()),
      active_channel_(initial_channel),
      min_consecutive_failures_(min_consecutive_failures),
      max_joint_velocity_(max_joint_velocity),
      input_limit_(input_limit) {
  if (input_limit_ == std::numeric_limits<double>::max()) {
    std::cout << "Warning. No input limits have been set." << std::endl;
  }

  // Create input ports
  command_input_port_ =
      this->DeclareVectorInputPort("u, t",
                                   TimestampedVector<double>(num_actuators_))
          .get_index();
  safety_command_input_port_ =
      this->DeclareVectorInputPort("safety: u, t",
                                   TimestampedVector<double>(num_actuators_))
          .get_index();
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(num_positions_, num_velocities_,
                                              num_actuators_))
          .get_index();
  controller_switch_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_controller_switch",
              drake::Value<dairlib::lcmt_controller_switch>{})
          .get_index();
  cassie_input_port_ =
      this->DeclareAbstractInputPort("lcmt_cassie_out",
                                     drake::Value<dairlib::lcmt_cassie_out>{})
          .get_index();
  controller_error_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_controller_error",
              drake::Value<dairlib::lcmt_controller_failure>{})
          .get_index();

  // Create output port for commands
  command_output_port_ =
      this->DeclareVectorOutputPort("u, t",
                                    TimestampedVector<double>(num_actuators_),
                                    &InputSupervisor::SetMotorTorques)
          .get_index();

  // Create output port for status
  status_output_port_ =
      this->DeclareAbstractOutputPort("status", &InputSupervisor::SetStatus)
          .get_index();

  prev_efforts_index_ = DeclareDiscreteState(num_actuators_);

  /// controller_delay: triggers when there is too long between consecutive
  /// controller messages
  /// soft_estop: (operator_triggered) triggers when the soft-estop is engaged
  /// by the operator.
  /// is_nan: triggers whenever a NaN is received by the dispatcher to prevent
  /// sending bad motor commands.
  /// consecutive_failures: triggers whenever the velocity or actuator limits
  /// are reached.
  /// controller_failure_flag: triggers whenever the active
  /// controller channel sends a failure message.
  error_indices_["controller_delay"] = 0;
  error_indices_["soft_estop"] = 1;
  error_indices_["is_nan"] = 2;
  error_indices_["consecutive_failures"] = 3;
  error_indices_["controller_failure_flag"] = 4;

  // Create error flags as discrete state
  n_fails_index_ = DeclareDiscreteState(1);
  switch_time_index_ = DeclareDiscreteState(1);
  prev_efforts_time_index_ = DeclareDiscreteState(1);
  shutdown_index_ = DeclareDiscreteState(1);

  error_indices_index_ = DeclareDiscreteState(error_indices_.size());

  K_ = plant_.MakeActuationMatrix().transpose();
  K_ *= kEStopGain;

  // Create update for error flag
  DeclarePeriodicDiscreteUpdateEvent(update_period, 0,
                                     &InputSupervisor::UpdateErrorFlag);
}

void InputSupervisor::SetMotorTorques(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);
  const TimestampedVector<double>* safety_command =
      (TimestampedVector<double>*)this->EvalVectorInput(
          context, safety_command_input_port_);
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);

  const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
      context, cassie_input_port_);

  // iterate through all the possible error flags
  bool is_error = false;
  for (const auto& error_flags : error_indices_) {
    is_error = is_error || context.get_discrete_state().get_value(
                               error_indices_index_)[error_flags.second];
  }

  // If the soft estop signal is triggered, applying only damping regardless of
  // any other controller signal
  if (cassie_out->pelvis.radio.channel[15] == -1) {
    Eigen::VectorXd u = -K_ * state->GetVelocities();
    output->SetDataVector(u);
    return;
  }

  // If there has not been an error, copy over the command.
  // If there has been an error, set the command to all zeros
  output->set_timestamp(command->get_timestamp());
  if (!is_error) {
    // If input_limit_ has been set, limit inputs to
    // [-input_limit_, input_limit_]
    if (input_limit_ != std::numeric_limits<double>::max()) {
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
      output->SetDataVector(command->get_value());
    }
  } else {
    output->SetDataVector(safety_command->get_value());
    return;
  }

  // Blend the efforts between the previous controller effort and the current
  // commanded effort using linear interpolation
  double alpha = (command->get_timestamp() -
                  context.get_discrete_state(switch_time_index_)[0]) /
                 blend_duration_;

  if (alpha <= 1.0) {
    Eigen::VectorXd blended_effort =
        alpha * output->get_data() +
        (1 - alpha) *
            context.get_discrete_state(prev_efforts_index_).get_value();
    output->SetDataVector(blended_effort);
    if (fmod(command->get_timestamp(), 0.5) < 1e-4) {
      std::cout << "Blending efforts" << std::endl;
    }
  }
}

void InputSupervisor::SetStatus(
    const Context<double>& context,
    dairlib::lcmt_input_supervisor_status* output) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);
  output->utime = command->get_timestamp() * 1e6;
  output->active_channel = active_channel_;
  output->shutdown = context.get_discrete_state().get_value(shutdown_index_)[0];
  output->num_status = error_indices_.size();
  output->status_names = std::vector<std::string>(error_indices_.size());
  output->status_states = std::vector<int8_t>(error_indices_.size());
  for (const auto& error_flags : error_indices_) {
    output->status_names[error_flags.second] = error_flags.first;
    output->status_states[error_flags.second] =
        context.get_discrete_state().get_value(
            error_indices_index_)[error_flags.second];
  }
}

void InputSupervisor::UpdateErrorFlag(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto* controller_switch =
      this->EvalInputValue<dairlib::lcmt_controller_switch>(
          context, controller_switch_input_port_);
  const OutputVector<double>* robot_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  const auto* controller_error =
      this->EvalInputValue<dairlib::lcmt_controller_failure>(
          context, controller_error_port_);
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);

  // Note the += operator works as an or operator because we only check if the
  // error flag != 0
  if (controller_error->controller_channel == active_channel_ &&
      controller_error->error_code != 0) {
    discrete_state->get_mutable_value(
        error_indices_index_)[error_indices_.at("controller_failure_flag")] = 1;
  }
  if ((robot_state->get_timestamp() -
           context.get_discrete_state(prev_efforts_time_index_)[0] >
       kMaxControllerDelay)) {
    discrete_state->get_mutable_value(
        error_indices_index_)[error_indices_.at("controller_delay")] = 1;
  }
  if (command->get_data().array().isNaN().any()){
    discrete_state->get_mutable_value(
        error_indices_index_)[error_indices_.at("is_nan")] = 0;
  }
  if (  context.get_discrete_state(n_fails_index_)[0] >=
        min_consecutive_failures_){
    discrete_state->get_mutable_value(
        error_indices_index_)[error_indices_.at("consecutive_failures")] = 1;
  }
  CheckRadio(context, discrete_state);
  CheckVelocities(context, discrete_state);

  bool is_error = false;
  for (const auto& error_flags : error_indices_) {
    is_error = is_error || context.get_discrete_state().get_value(
                               error_indices_index_)[error_flags.second];
  }
  discrete_state->get_mutable_value(shutdown_index_)[0] = is_error;

  // When receiving a new controller switch message, record the time
  if (discrete_state->get_mutable_value(switch_time_index_)[0] <
      controller_switch->utime * 1e-6) {
    std::cout << "Got new switch message" << std::endl;
    active_channel_ = controller_switch->channel;
    discrete_state->get_mutable_value(switch_time_index_)[0] =
        controller_switch->utime * 1e-6;
    blend_duration_ = controller_switch->blend_duration;
  }

  //  if (command->get_timestamp() -
  //          discrete_state->get_mutable_value(prev_efforts_time_index_)[0] >
  //      kMaxControllerDelay) {
  //    discrete_state->get_mutable_value(prev_efforts_time_index_)[0] = 0.0;
  //  } else {
  //
  //  }
  discrete_state->get_mutable_value(prev_efforts_time_index_)[0] =
      command->get_timestamp();

  // Update the previous commanded switch message unless currently blending
  // efforts
  if (command->get_timestamp() - controller_switch->utime * 1e-6 >=
      blend_duration_) {
    discrete_state->get_mutable_value(prev_efforts_index_) =
        command->get_value();
  }
}

void InputSupervisor::CheckVelocities(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  const Eigen::VectorXd& velocities = state->GetVelocities();

  if (discrete_state->get_value(n_fails_index_)[0] <
      min_consecutive_failures_) {
    // If any velocity is above the threshold, set the error flag
    bool is_velocity_error = (velocities.array() > max_joint_velocity_).any() ||
                             (velocities.array() < -max_joint_velocity_).any();
    if (is_velocity_error) {
      // Increment counter
      discrete_state->get_mutable_value(n_fails_index_)[0] += 1;
      std::cout << "Error! Velocity has exceeded the threshold of "
                << max_joint_velocity_ << std::endl;
      std::cout << "Consecutive error "
                << discrete_state->get_value(n_fails_index_)[0] << " of "
                << min_consecutive_failures_ << std::endl;
      std::cout << "Velocity vector: " << std::endl
                << velocities << std::endl
                << std::endl;
    } else {
      // Reset counter
      discrete_state->get_mutable_value(n_fails_index_)[0] = 0;
    }
  }
}

void InputSupervisor::CheckRadio(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
      context, cassie_input_port_);
  if (cassie_out->pelvis.radio.channel[15] == -1) {
    discrete_state->get_mutable_value(
        error_indices_index_)[error_indices_.at("soft_estop")] = 1;
  }
}

}  // namespace dairlib
