//
// Created by yangwill on 11/15/19.
//

#include "simulator_drift.h"

using dairlib::systems::OutputVector;
using Eigen::VectorXd;

SimulatorDrift::SimulatorDrift(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& drift_mean, const Eigen::MatrixXd& drift_cov)
    : plant_(plant), drift_mean_(drift_mean), drift_cov_(drift_cov) {
  DRAKE_ASSERT(drift_mean_.size() == drift_cov_.cols());
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&SimulatorDrift::DiscreteVariableUpdate);
  accumulated_drift_index_ =
      this->DeclareDiscreteState(VectorXd::Zero(plant_.num_positions()));

  this->DeclareVectorOutputPort(
      OutputVector<double>(plant_.num_positions(), plant_.num_velocities(),
                           plant_.num_actuators()),
      &SimulatorDrift::CalcAdjustedState);
}

drake::systems::EventStatus SimulatorDrift::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  //  const OutputVector<double>* robot_output = (OutputVector<double>*)
  //      this->EvalVectorInput(context, state_port_);
  //  double timestamp = robot_output->get_timestamp();
  auto accumulated_drift =
      discrete_state->get_mutable_vector(accumulated_drift_index_)
          .get_mutable_value();
  VectorXd v = VectorXd::Random(drift_mean_.size());
  accumulated_drift << accumulated_drift + (drift_mean_ + drift_cov_ * v);
  return drake::systems::EventStatus::Succeeded();
}

void SimulatorDrift::CalcAdjustedState(
    const drake::systems::Context<double>& context,
    dairlib::systems::OutputVector<double>* output) const {
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto accumulated_drift =
      context.get_discrete_state(accumulated_drift_index_).get_value();
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();
  VectorXd u = robotOutput->GetEfforts();

  VectorXd adjusted_output;
  adjusted_output << q + accumulated_drift, v, u;
  output->get_mutable_value() = adjusted_output;
}