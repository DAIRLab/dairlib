#include "systems/controllers/safe_velocity_controller.h"

namespace dairlib{
namespace systems{

 // Velocity and torque passthrough system that kills the velocity and
 // torque commands if it goes above max_velocity.
 // Remember to use std::move on the rigid body tree argument.
SafeVelocityController::SafeVelocityController(
    double max_velocity, int num_joints) {

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_torques_input_port_ = this->DeclareVectorInputPort(
      "joint_torques_input", BasicVector<double>(num_joints)).get_index();
  joint_velocities_input_port_ = this->DeclareVectorInputPort(
      "joint_velociites_input", BasicVector<double>(num_joints)).get_index();
  joint_torques_output_port_ = this->DeclareVectorOutputPort(
      BasicVector<double>(7), &SafeVelocityController::CalcOutputTorques).get_index();

  this->DeclareDiscreteState(1);
  this->DeclarePerStepDiscreteUpdateEvent(
      &SafeVelocityController::CheckTerminate);

  max_velocity_ = max_velocity;
}

void SafeVelocityController::CalcOutputTorques(
    const Context<double> &context, BasicVector<double>* output) const {

  VectorX<double> torques_in = this->EvalVectorInput(context,
      joint_torques_input_port_)->CopyToVector();

  VectorX<double> velocities_in = this->EvalVectorInput(context,
      joint_velocities_input_port_)->CopyToVector();

  double terminated = context.get_discrete_state()[0];

  if (terminated > 0.5) {
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(7);
    torques_in = zeros;
  }

  output->set_value(torques_in);
}

drake::systems::EventStatus SafeVelocityController::CheckTerminate(
  const Context<double> &context,
  drake::systems::DiscreteValues<double>* next_state) const {

  VectorX<double> velocities_in = this->EvalVectorInput(context,
      joint_velocities_input_port_)->CopyToVector();

  if (velocities_in.maxCoeff() > max_velocity_ ||
      velocities_in.minCoeff() < -max_velocity_) {
    (*next_state)[0] = 1.0;
    drake::log()->error("Velocity out of bounds, killing");
  }

  return drake::systems::EventStatus::Succeeded();
}


} // namespace systems
} // namespace dairlib
