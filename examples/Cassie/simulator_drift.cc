#include "simulator_drift.h"

using dairlib::systems::OutputVector;
using Eigen::VectorXd;

SimulatorDrift::SimulatorDrift(const RigidBodyTree<double>& tree,
                               const Eigen::VectorXd& drift_mean,
                               const Eigen::MatrixXd& drift_cov)
    : tree_(tree), drift_mean_(drift_mean), drift_cov_(drift_cov) {
  DRAKE_ASSERT(drift_mean_.size() == drift_cov_.cols());
  DRAKE_ASSERT(drift_mean_.size() == tree.get_num_positions());
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                        tree_.get_num_positions(), tree_.get_num_velocities(),
                        tree_.get_num_actuators()))
                    .get_index();

  DeclarePerStepDiscreteUpdateEvent(&SimulatorDrift::DiscreteVariableUpdate);
  accumulated_drift_index_ =
      this->DeclareDiscreteState(VectorXd::Zero(tree_.get_num_positions()));

  time_idx_ = this->DeclareDiscreteState(1);
  this->DeclareVectorOutputPort(OutputVector<double>(tree_.get_num_positions(),
                                                     tree_.get_num_velocities(),
                                                     tree_.get_num_actuators()),
                                &SimulatorDrift::CalcAdjustedState);
}

drake::systems::EventStatus SimulatorDrift::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto prev_time_stamp =
      discrete_state->get_mutable_vector(time_idx_).get_mutable_value();

  if (state->get_timestamp() > prev_time_stamp(0)) {
    auto accumulated_drift =
        discrete_state->get_mutable_vector(accumulated_drift_index_)
            .get_mutable_value();
    double dt = state->get_timestamp() - prev_time_stamp(0);
    VectorXd v = VectorXd::Random(drift_mean_.size());
    accumulated_drift << accumulated_drift + 
      (drift_mean_ + drift_cov_ * v) * dt;
    prev_time_stamp << state->get_timestamp();
  }
  return drake::systems::EventStatus::Succeeded();
}

void SimulatorDrift::CalcAdjustedState(
    const drake::systems::Context<double>& context,
    dairlib::systems::OutputVector<double>* output) const {
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto accumulated_drift =
      context.get_discrete_state(accumulated_drift_index_).get_value();

  VectorXd data = robotOutput->get_data();
  data.head(tree_.get_num_positions()) += accumulated_drift;
  output->SetDataVector(data);
  output->set_timestamp(robotOutput->get_timestamp());
}