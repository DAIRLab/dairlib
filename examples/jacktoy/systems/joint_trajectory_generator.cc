#include "joint_trajectory_generator.h"

#include <iostream>

#include "systems/framework/output_vector.h"

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

using systems::OutputVector;

JointTrajectoryGenerator::JointTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& target_position)
    : target_position_(target_position) {
  // Input/Output Setup
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();
  state_port_ = this->DeclareVectorInputPort(
                        "x_franka", OutputVector<double>(plant.num_positions(),
                                                         plant.num_velocities(),
                                                         plant.num_actuators()))
                    .get_index();

  initial_position_index_ =
      this->DeclareDiscreteState(VectorXd::Zero(plant.num_positions()));
  initial_time_index_ =
      this->DeclareDiscreteState(VectorXd::Zero(1));
  DeclareForcedDiscreteUpdateEvent(
      &JointTrajectoryGenerator::DiscreteVariableUpdate);
  joint_trajectory_ports_.resize(plant.num_positions());
  for (int i = 0; i < plant.num_positions(); ++i) {
    joint_trajectory_ports_[i] =
        this->DeclareAbstractOutputPort(
                "joint_traj_" + std::to_string(i),
                []() {
                  return drake::AbstractValue::Make<Trajectory<double>>(
                      PiecewisePolynomial<double>(VectorXd::Zero(1)));
                },
                [this, i](const Context<double>& context,
                          drake::AbstractValue* traj) {
                  this->CalcTraj(i, context, traj);
                })
            .get_index();
  }
}

drake::systems::EventStatus JointTrajectoryGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto initial_positions = discrete_state->get_mutable_value(initial_position_index_);
  auto initial_time = discrete_state->get_mutable_value(initial_time_index_);
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  if (initial_time[0] == 0.0) {  // poll once
    initial_positions = franka_output->GetPositions();
    initial_time[0] = context.get_time();
  }
  return drake::systems::EventStatus::Succeeded();
}

void JointTrajectoryGenerator::CalcTraj(
    int joint_index, const drake::systems::Context<double>& context,
    drake::AbstractValue* output) const {
  auto output_value = &output->get_mutable_value<Trajectory<double>>();
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output_value);
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto initial_positions =
      context.get_discrete_state(initial_position_index_).value();
  auto initial_time = context.get_discrete_state(initial_time_index_).value();

  const auto& radio_out = this->EvalVectorInput(context, radio_port_);

  std::vector<double> breaks = {initial_time[0] + 1.0, initial_time[0] + 6.0};
  std::vector<MatrixXd> sampled_positions(2);
  sampled_positions[0] = MatrixXd::Zero(1, 1);
  sampled_positions[0] << initial_positions[joint_index];
  sampled_positions[1] = MatrixXd::Zero(1, 1);
  sampled_positions[1] << target_position_[joint_index];
  if (context.get_time() >= 1.0) {  // poll for a second
    *casted_traj =
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            breaks, sampled_positions, MatrixXd::Zero(1, 1),
            MatrixXd::Zero(1, 1));
  } else {
    *casted_traj = PiecewisePolynomial<double>(
        franka_output->GetPositions().segment(joint_index, 1));
  }
}

}  // namespace dairlib
