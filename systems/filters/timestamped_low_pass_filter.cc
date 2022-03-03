#include "timestamped_low_pass_filter.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;


namespace dairlib::systems {

using systems::TimestampedVector;


TimestampedLowPassFilter::TimestampedLowPassFilter(
    const std::vector<double>& tau, int n_y,
    std::optional<std::vector<int>> filter_idxs)
    : n_y_filt_(tau.size()), tau_(tau) {

  DRAKE_DEMAND(filter_idxs.has_value() || tau.size() == n_y);
  if (filter_idxs.has_value()) {
    DRAKE_DEMAND(filter_idxs->size() == tau.size());
  }

  if (!filter_idxs.has_value()){
    std::vector<int> idxs(n_y);
    std::iota(idxs.begin(), idxs.end(), 0);
    filter_idxs_ = idxs;
  } else {
    filter_idxs_ = filter_idxs.value();
  }

  TimestampedVector<double> model_vector(n_y);
  this->DeclareVectorInputPort("x", model_vector);
  this->DeclareVectorOutputPort("y", model_vector,
                                &TimestampedLowPassFilter::CalcFilter);
  this->DeclarePerStepDiscreteUpdateEvent(
      &TimestampedLowPassFilter::DiscreteVariableUpdate);

  prev_val_idx_ = this->DeclareDiscreteState(VectorXd::Zero(n_y));
  prev_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
}

drake::systems::EventStatus TimestampedLowPassFilter::DiscreteVariableUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {

  const TimestampedVector<double>* y_t =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  double dt  = y_t->get_timestamp() -
      discrete_state->get_value(prev_time_idx_)[0];
  VectorXd y = y_t->get_data();
  VectorXd y_prev = discrete_state->get_value(prev_val_idx_);
  for (int i = 0; i < n_y_filt_; i++){
    double alpha = dt / (dt + tau_.at(i));
    y(filter_idxs_.at(i)) =
        alpha * y(filter_idxs_.at(i)) +
        (1.0- alpha) * y_prev(filter_idxs_.at(i));
  }
  discrete_state->get_mutable_value(prev_time_idx_) <<
      y_t->get_timestamp() * VectorXd::Ones(1) ;
  discrete_state->get_mutable_value(prev_val_idx_) << y;

  return drake::systems::EventStatus::Succeeded();
}

void TimestampedLowPassFilter::CalcFilter(
    const drake::systems::Context<double> &context,
    systems::TimestampedVector<double> *y) const {
  y->SetDataVector(context.get_discrete_state(prev_val_idx_).get_value());
  y->set_timestamp(context.get_discrete_state(prev_time_idx_).get_value()[0]);
}
} 