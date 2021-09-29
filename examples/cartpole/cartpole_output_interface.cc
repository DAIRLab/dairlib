#include "cartpole_output_interface.h"

using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector2d;

namespace dairlib{

CartpoleOutputInterface::CartpoleOutputInterface(
    const MultibodyPlant<double> &plant) : plant_(plant) {

  this->DeclareVectorOutputPort(
      "x", BasicVector<double>(Vector4d::Zero()), &CartpoleOutputInterface::CopyOutput);

  prev_x_idx_ = this->DeclareDiscreteState(Vector4d::Zero());
  effort_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
}

void CartpoleOutputInterface::SetupOutputInterface() {
  ConfigureEpos();
  ConfigureLabjack();
}

void CartpoleOutputInterface::ConfigureEpos() {
  unsigned int error_code = 0;
  MotorHandle_ = epos::OpenDevice(&error_code);
  DRAKE_ASSERT(error_code == 0);
  epos::HomeDevice(MotorHandle_);
  epos::EnableDevice(MotorHandle_);
}

void CartpoleOutputInterface::ConfigureLabjack() {
  EncoderHandle_ = labjack::OpenLabjack();
}

drake::systems::EventStatus CartpoleOutputInterface::DiscreteUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *values) const {

  // Context
  double curr_time = context.get_time();

  // state vector
  Vector4d x = Vector4d::Zero();

  // API calls
  double cart_pos = epos::GetCartPosition(MotorHandle_);
  double effort = epos::GetForceFromCurrent(MotorHandle_);
  long error_code = 0;
  double pole_angle = labjack::GetEncoderAngle(EncoderHandle_, &error_code);

  x.head(2) << cart_pos, pole_angle;

  // finite_differencing
  VectorXd dv = VectorXd::Zero(nv_);
  if (curr_time > 0) {
    dv = (x.head(2) - values->get_vector(prev_x_idx_).get_value().head(nq_)) /
        (curr_time - vel_filt_.time());
  }
  VectorXd vel = vel_filt_.filter(curr_time, dv);
  x.tail(nv_) = vel;
  values->get_mutable_vector(prev_x_idx_).get_mutable_value() << x;
  values->get_mutable_vector(effort_idx_).get_mutable_value()[0] = effort;

  return drake::systems::EventStatus::Succeeded();
}

void CartpoleOutputInterface::CopyOutput(
    const Context<double> &context, BasicVector<double> *output) const {

  Vector4d state = context.get_discrete_state(prev_x_idx_).get_value();
  VectorXd effort = context.get_discrete_state(effort_idx_).get_value();

  output->SetFromVector(state);
}
}