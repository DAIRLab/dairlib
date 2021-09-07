#include "timestamped_lowpass_filter.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;


namespace dairlib {

using systems::TimestampedVector;


TimestampedLowpassFilter::TimestampedLowpassFilter(double tau, int n_y) 
                                                  : tau_(tau) {
  TimestampedVector<double> model_vector n_y);

  this->DeclareVectorInputPort("x", model_vector);
  this->DeclareVectorOutputPort("y", model_vector, 
      &TimestampedLowpassFilter::CalcFilter);
  this->DeclarePerStepDiscreteUpdateEvent(
      &TimestampedLowpassFilter::DiscreteVariableUpdate);

  prev_val_idx_ = this->DeclareDiscreteState(VectorXd::Zero n_y));
  prev_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));

}

drake::systems::EventStatus TimestampedLowpassFilter::DiscreteVariableUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_state) const {

  const TimestampedVector<double>* y_t = 
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  double dt  = y_t->get_timestamp() - 
               discrete_state->get_value(prev_time_idx_)[0];
  double alpha = dt / (dt + tau_);

  // Calculate filtered value
  VectorXd y = alpha * y_t->get_data() + 
      (1.0 - alpha) * discrete_state->get_value(prev_val_idx_);

  discrete_state->get_mutable_value(prev_time_idx_) << 
      y_t->get_timestamp() * VectorXd::Ones(1) ;
  discrete_state->get_mutable_value(prev_val_idx_) << y;
}

void TimestampedLowpassFilter::CalcFilter(
    const drake::systems::Context<double> &context,
    systems::TimestampedVector<double> *y) const {
  y->set_value(context.get_discrete_state(prev_val_idx_).get_value());
  y->set_timestamp(context.get_discrete_state(prev_time_idx_).get_value()[0]);
}

}