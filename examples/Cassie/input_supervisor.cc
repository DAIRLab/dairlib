#include "examples/Cassie/input_supervisor.h"

#include "dairlib/lcmt_controller_switch.hpp"
#include "multibody/multibody_utils.h"

using drake::multibody::JointActuatorIndex;
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
  if (input_limit_ == std::numeric_limits<double>::max()) {
    std::cout << "Warning. No input limits have been set." << std::endl;
  }

  // Create input ports
  command_input_port_ =
      this->DeclareVectorInputPort("u, t",
                                   TimestampedVector<double>(num_actuators_))
          .get_index();
  ;
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

  // Create error flag as discrete state
  // Store both values in single discrete vector
  status_vars_index_ = DeclareDiscreteState(2);
  n_fails_index_ = 0;
  status_index_ = 1;
  switch_time_index_ = DeclareDiscreteState(1);
  prev_efforts_time_index_ = DeclareDiscreteState(1);
  prev_efforts_index_ = DeclareDiscreteState(num_actuators_);
  soft_estop_trigger_index_ = DeclareDiscreteState(1);
  is_nan_index_ = DeclareDiscreteState(1);
  is_critical_falling_index_ = DeclareDiscreteState(1);

  K_ = plant_.MakeActuationMatrix().transpose();
  K_ *= kEStopGain;
  K_critical_falling_ =
      kCriticalFallingGain * plant_.MakeActuationMatrix().transpose();

  // Create update for error flag
  DeclarePeriodicDiscreteUpdateEvent(update_period, 0,
                                     &InputSupervisor::UpdateErrorFlag);

  // Get indices
  auto position_idx_map = multibody::makeNameToPositionsMap(plant);
  auto velocity_idx_map = multibody::makeNameToVelocitiesMap(plant);
  left_ankle_pos_idx_ = position_idx_map.at("ankle_joint_left");
  right_ankle_pos_idx_ = position_idx_map.at("ankle_joint_right");
  left_ankle_vel_idx_ = velocity_idx_map.at("ankle_joint_leftdot");
  right_ankle_vel_idx_ = velocity_idx_map.at("ankle_joint_rightdot");
}

void InputSupervisor::SetMotorTorques(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);

  // Be aware that the publish rate of lcmt_cassie_out is low.
  const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
      context, cassie_input_port_);

  bool is_error =
      context.get_discrete_state(status_vars_index_)[n_fails_index_] >=
      min_consecutive_failures_;
  is_error =
      is_error || (command->get_timestamp() -
                       context.get_discrete_state(prev_efforts_time_index_)[0] >
                   kMaxControllerDelay);
  is_error =
      is_error || context.get_discrete_state(soft_estop_trigger_index_)[0];
  is_error = is_error || context.get_discrete_state(is_nan_index_)[0];
  if ((command->get_timestamp() -
           context.get_discrete_state(prev_efforts_time_index_)[0] >
       kMaxControllerDelay)) {
    std::cout << "Delay between controller commands is too long, shutting down"
              << std::endl;
  }

  bool is_nan = command->get_data().array().isNaN().any();

  if (context.get_discrete_state(is_critical_falling_index_)[0]) {
    Eigen::VectorXd u = -K_ * state->GetVelocities();

    // Version 1 - Damp knee joints
    //    Eigen::VectorXd u_big = -K_critical_falling_ * state->GetVelocities();
    //    u.segment<2>(6) = u_big.segment<2>(6);
    // Version 2 - Damp pelvis vel
    u.segment<2>(6) = -kCriticalFallingGain * state->GetVelocities()(5) *
                      Eigen::VectorXd::Ones(2);

    // TODO: for some reason, mujoco doesn't always track the torque command. It went to zero sometimes.

    //    std::cout << state->get_timestamp() <<  ": u = " << u.transpose() <<
    //    std::endl;
    output->SetDataVector(u);
    return;
  }
  // If the soft estop signal is triggered, applying only damping regardless of
  // any other controller signal
  if (cassie_out->pelvis.radio.channel[15] == -1 || is_nan) {
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
    if (command->get_timestamp() > 7) {
      output->SetDataVector(Eigen::VectorXd::Zero(10));
    }

  } else {
    output->SetDataVector(Eigen::VectorXd::Zero(num_actuators_));
  }

  // Blend the efforts between the previous controller effort and the current
  // commanded effort using linear interpolation
  double alpha = (command->get_timestamp() -
                  context.get_discrete_state(switch_time_index_)[0]) /
                 blend_duration_;

  if (alpha <= 1.0) {
    Eigen::VectorXd blended_effort =
        alpha * command->get_value() +
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

  output->status =
      int(context.get_discrete_state(status_vars_index_)[status_index_]);
  output->utime = command->get_timestamp() * 1e6;
  output->vel_limit =
      bool(context.get_discrete_state(status_vars_index_)[status_index_]);

  if (input_limit_ != std::numeric_limits<double>::max()) {
    for (int i = 0; i < command->get_data().size(); i++) {
      double command_value = command->get_data()(i);
      if (command_value > input_limit_ || command_value < -input_limit_) {
        output->status += 2;
        output->act_limit = true;
        break;
      }
    }
  }

  // Shutdown is/will soon be active (the status flag is set in a separate loop
  // from the actual motor torques so the update of the status bit could be
  // slightly off
  if (context.get_discrete_state(status_vars_index_)[n_fails_index_] >=
      min_consecutive_failures_) {
    output->status += 4;
    output->shutdown = true;
  }

  if ((command->get_timestamp() -
           context.get_discrete_state(prev_efforts_time_index_)[0] >
       kMaxControllerDelay)) {
    output->act_delay = true;
    output->shutdown = true;
  }

  if (context.get_discrete_state(is_critical_falling_index_)[0]) {
    output->status += 8;
  }
}

void InputSupervisor::UpdateErrorFlag(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto* controller_switch =
      this->EvalInputValue<dairlib::lcmt_controller_switch>(
          context, controller_switch_input_port_);
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                        command_input_port_);
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);

  if (command->get_timestamp() -
          discrete_state->get_mutable_vector(prev_efforts_time_index_)[0] >
      kMaxControllerDelay) {
    discrete_state->get_mutable_vector(prev_efforts_time_index_)[0] = 0.0;
  } else {
    discrete_state->get_mutable_vector(prev_efforts_time_index_)[0] =
        command->get_timestamp();
  }

  CheckRadio(context, discrete_state);
  CheckVelocities(context, discrete_state);

  if (!(discrete_state->get_mutable_vector(is_critical_falling_index_)[0])) {
    if (IsInCriticalFailure(state)) {
      discrete_state->get_mutable_vector(is_critical_falling_index_)[0] = true;
    }
  } else {
    //    if (HasLeftCriticalFailure(state)) {
    //      discrete_state->get_mutable_vector(is_critical_falling_index_)[0] =
    //      false;
    //    }
  }

  // Only update if it's setting the error flag to true
  discrete_state->get_mutable_vector(is_nan_index_)[0] =
      discrete_state->get_mutable_vector(is_nan_index_)[0] ||
      command->get_data().array().isNaN().any();

  // When receiving a new controller switch message, record the time
  if (discrete_state->get_mutable_vector(switch_time_index_)[0] <
      controller_switch->utime * 1e-6) {
    std::cout << "Got new switch message" << std::endl;
    discrete_state->get_mutable_vector(switch_time_index_)[0] =
        controller_switch->utime * 1e-6;
    blend_duration_ = controller_switch->blend_duration;
  }

  // Update the previous commanded switch message unless currently blending
  // efforts
  if (command->get_timestamp() - controller_switch->utime * 1e-6 >=
      blend_duration_) {
    discrete_state->get_mutable_vector(prev_efforts_index_)
        .get_mutable_value() = command->get_value();
  }
}

void InputSupervisor::CheckVelocities(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  const Eigen::VectorXd& velocities = state->GetVelocities();

  if (discrete_state->get_vector(status_vars_index_)[n_fails_index_] <
      min_consecutive_failures_) {
    // If any velocity is above the threshold, set the error flag
    bool is_velocity_error = (velocities.array() > max_joint_velocity_).any() ||
                             (velocities.array() < -max_joint_velocity_).any();
    if (is_velocity_error) {
      // Increment counter
      discrete_state->get_mutable_vector(status_vars_index_)[n_fails_index_] +=
          1;
      // Using the discrete state which is a vector of doubles to store a bool
      discrete_state->get_mutable_vector(status_vars_index_)[status_index_] =
          double(true);
      std::cout << "Error! Velocity has exceeded the threshold of "
                << max_joint_velocity_ << std::endl;
      std::cout << "Consecutive error "
                << discrete_state->get_vector(
                       status_vars_index_)[n_fails_index_]
                << " of " << min_consecutive_failures_ << std::endl;
      std::cout << "Velocity vector: " << std::endl
                << velocities << std::endl
                << std::endl;
    } else {
      // Reset counter
      discrete_state->get_mutable_vector(status_vars_index_)[n_fails_index_] =
          0;
      discrete_state->get_mutable_vector(status_vars_index_)[status_index_] =
          double(false);
    }
  }
}

void InputSupervisor::CheckRadio(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
      context, cassie_input_port_);
  if (cassie_out->pelvis.radio.channel[15] == -1) {
    discrete_state->get_mutable_vector(soft_estop_trigger_index_)[0] = 1;
  }
}

bool InputSupervisor::IsInCriticalFailure(
    const OutputVector<double>* state) const {
  //  const double* imu_d = cassie_out->pelvis.vectorNav.linearAcceleration;
  //  Eigen::Vector3d imu_aceel(imu_d[0], imu_d[1], imu_d[2]);
  Eigen::Vector3d imu_aceel = state->GetIMUAccelerations();

  double ankle_pos_threshold = 2.1;  // 2.3;
  double ankle_vel_threshold = 2;
  double imu_magnitude_threshold = 0.3 * 9.8;

  bool is_leg_crouching =
      (state->GetPositions()(left_ankle_pos_idx_) > ankle_pos_threshold) &&
      (state->GetPositions()(right_ankle_pos_idx_) > ankle_pos_threshold) &&
      (state->GetVelocities()(left_ankle_vel_idx_) > ankle_vel_threshold) &&
      (state->GetVelocities()(right_ankle_vel_idx_) > ankle_vel_threshold);
  bool is_pelvis_falling = imu_aceel.norm() < imu_magnitude_threshold;

  // bool is_pelvis_roll_pitch_small =

  //  std::cout << state->get_timestamp() << ", " << is_leg_crouching << ", "
  //            << is_pelvis_falling << std::endl;

  return is_leg_crouching && is_pelvis_falling;
}

bool InputSupervisor::HasLeftCriticalFailure(
    const OutputVector<double>* state) const {
  double pelvis_vel_threshould = -0.3;
  // double max_duration =

  return state->GetVelocities()(5) > pelvis_vel_threshould;
}

}  // namespace dairlib
