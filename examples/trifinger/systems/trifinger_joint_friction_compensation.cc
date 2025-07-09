#include "trifinger_joint_friction_compensation.h"

#include <cmath>
#include <iostream>

namespace dairlib::systems {
using Eigen::VectorXd;
JointFrictionCompensator::JointFrictionCompensator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant), context_(context) {
  this->set_name("joint_friction_compensator");
  n_q_ = plant.num_positions();
  n_v_ = plant.num_velocities();
  n_u_ = plant.num_actuated_dofs();
  state_port_ =
      this->DeclareVectorInputPort("trifinger_state",
                                   OutputVector<double>(n_q_, n_v_, n_u_))
          .get_index();

  uncompensated_torque_input_port_ =
      this->DeclareVectorInputPort("osc_output_torque",
                                   TimestampedVector<double>(n_u_))
          .get_index();
  compensated_torque_output_port_ =
      this->DeclareVectorOutputPort(
              "friction_compensated_torque", TimestampedVector<double>(n_u_),
              &JointFrictionCompensator::CopyOutCompensatedTorque)
          .get_index();

  estimated_friction_torque_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_estimated_friction_torque",
              &JointFrictionCompensator::CopyOutEstimatedFriction)
          .get_index();
  // Declare update event.
  DeclareForcedDiscreteUpdateEvent(
      &JointFrictionCompensator::DiscreteVariableUpdate);
  estimated_friction_torque_idx_ =
      DeclareDiscreteState(Eigen::VectorXd::Zero(n_u_));
  compensated_torque_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(n_u_));
  prev_timestamp_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(1));
}

void JointFrictionCompensator::CopyOutCompensatedTorque(
    const drake::systems::Context<double>& context,
    systems::TimestampedVector<double>* output_torque) const {
  auto compensated_torque =
      context.get_discrete_state(compensated_torque_idx_).get_value();
  auto timestamp = context.get_discrete_state(prev_timestamp_idx_).value()(0);
  output_torque->SetDataVector(compensated_torque);
  output_torque->set_timestamp(timestamp);
}

drake::systems::EventStatus JointFrictionCompensator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* trifinger_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const TimestampedVector<double>* osc_output_torque =
      (TimestampedVector<double>*)this->EvalVectorInput(
          context, uncompensated_torque_input_port_);
  Eigen::VectorXd coulomb_friction_coeff(n_u_);
  Eigen::VectorXd static_friction_stiffness(n_u_);
  coulomb_friction_coeff << 0.0427, 0.0429, 0.0519, 0, 0, 0, 0, 0, 0;
  static_friction_stiffness << 7.0, 7.0, 7.0, 0, 0, 0, 0, 0, 0;

  auto trifinger_velocities = trifinger_state->GetVelocities();
  auto osc_output_torque_data = osc_output_torque->value();

  std::cout << trifinger_velocities.data() << std::endl;

  auto estimated_friction_torque = coulomb_friction_coeff.cwiseProduct(
      (static_friction_stiffness.cwiseProduct(trifinger_velocities))
          .unaryExpr([](double elem) { return std::tanh(elem); }));
  auto compensated_torque = osc_output_torque_data + estimated_friction_torque;

  discrete_state->get_mutable_vector(estimated_friction_torque_idx_)
      .set_value(estimated_friction_torque);
  discrete_state->get_mutable_vector(compensated_torque_idx_)
      .set_value(compensated_torque);
  discrete_state->get_mutable_vector(prev_timestamp_idx_)
      .set_value((Eigen::VectorXd::Ones(1) * trifinger_state->get_timestamp()));
  return drake::systems::EventStatus::Succeeded();
}

void JointFrictionCompensator::CopyOutEstimatedFriction(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_estimated_joint_friction_trifinger* friction_torque) const {
  auto estimated_friction_torque =
      context.get_discrete_state(estimated_friction_torque_idx_).get_value();
  auto timestamp = context.get_discrete_state(prev_timestamp_idx_).value()(0);
  friction_torque->estimatedFrictionTorque =
      CopyVectorXdToStdVector(estimated_friction_torque);
  friction_torque->num_efforts = n_u_;
  friction_torque->utime = timestamp;
}

}  // namespace dairlib::systems
