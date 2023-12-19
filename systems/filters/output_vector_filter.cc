#include "filter_utils.h"
#include "output_vector_filter.h"
#include "systems/framework/output_vector.h"

using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace dairlib::systems {

OutputVectorFilter::OutputVectorFilter(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<double>& tau, std::optional<std::vector<int>> filter_idxs)
    : n_y_filt_(tau.size()), tau_(tau) {
  int n_y = plant.num_positions() + plant.num_velocities() +
            plant.num_actuators() + 3;

  // Check user input
  DRAKE_DEMAND(filter_idxs.has_value() || tau.size() == n_y);
  if (filter_idxs.has_value()) {
    DRAKE_DEMAND(filter_idxs->size() == tau.size());
  }

  // Set the indices to be filtered if provided
  if (filter_idxs.has_value()) {
    filter_idxs_ = filter_idxs.value();
  } else {
    std::vector<int> idxs(n_y);
    std::iota(idxs.begin(), idxs.end(), 0);
    filter_idxs_ = idxs;
  }

  OutputVector<double> model_vector(
      plant.num_positions(), plant.num_velocities(), plant.num_actuators());

  this->DeclareVectorInputPort("x", model_vector);
  this->DeclareVectorOutputPort("y", model_vector,
                                &OutputVectorFilter::CopyFilterValues);
  this->DeclarePerStepDiscreteUpdateEvent(
      &OutputVectorFilter::DiscreteVariableUpdate);

  prev_val_idx_ = this->DeclareDiscreteState(VectorXd::Zero(n_y_filt_));
  prev_time_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
}

drake::systems::EventStatus OutputVectorFilter::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* y_t =
      (OutputVector<double>*)EvalVectorInput(context, 0);

  double dt =
      y_t->get_timestamp() - discrete_state->get_value(prev_time_idx_)[0];
  VectorXd y = VectorXd::Zero(n_y_filt_);
  VectorXd y_prev = discrete_state->get_value(prev_val_idx_);

  for (int i = 0; i < n_y_filt_; i++) {
    double alpha = dt / (dt + tau_.at(i));
    y(i) =
        alpha * y_t->GetState()(filter_idxs_.at(i)) + (1.0 - alpha) * y_prev(i);
  }
  discrete_state->get_mutable_value(prev_time_idx_)
      << y_t->get_timestamp() * VectorXd::Ones(1);
  discrete_state->get_mutable_value(prev_val_idx_) << y;
  return drake::systems::EventStatus::Succeeded();
}

void OutputVectorFilter::CopyFilterValues(
    const drake::systems::Context<double>& context,
    dairlib::systems::OutputVector<double>* y) const {
  // Copy over y from the input port
  auto y_curr = (OutputVector<double>*)EvalVectorInput(context, 0);
  y->SetDataVector(y_curr->get_data());
  y->set_timestamp(y_curr->get_timestamp());

  // Replace y[i] with filtered value for all i in filtered_idxs_
  // but don't filter the first value
  if (context.get_discrete_state(prev_time_idx_).get_value()[0] >= .001) {
    VectorXd y_filt = context.get_discrete_state(prev_val_idx_).get_value();
    for (int i = 0; i < n_y_filt_; i++) {
      y->get_mutable_value()[filter_idxs_.at(i)] = y_filt(i);
    }
  }
}


OutputVectorButterworthFilter::OutputVectorButterworthFilter(
    const drake::multibody::MultibodyPlant<double> &plant,
    int order,
    double sampling_frequency,
    const std::vector<double> &f_c,
    std::optional<std::vector<int>> filter_idxs) : order_(order){

  int ny = plant.num_positions() + plant.num_velocities() +
      plant.num_actuators() + 3;
  int ny_filt = (filter_idxs.has_value()) ? filter_idxs.value().size() : ny;

  std::vector<int> filter_idxs_local(ny);
  if (filter_idxs.has_value()) {
    filter_idxs_local = filter_idxs.value();
  } else {
    std::iota(filter_idxs_local.begin(), filter_idxs_local.end(), 0);
  }
  filter_idxs_ = filter_idxs_local;

  DRAKE_DEMAND(f_c.size() == ny_filt);
  for (int i = 0; i < ny_filt; i++) {
    index_to_filter_map_[filter_idxs_local.at(i)] =
        filter_utils::butter(order, sampling_frequency, f_c.at(i));
  }

  OutputVector<double> model_vector(
      plant.num_positions(),
      plant.num_velocities(), plant.num_actuators());

  this->DeclareVectorInputPort("x", model_vector);
  this->DeclareVectorOutputPort("y", model_vector,
                                &OutputVectorButterworthFilter::CopyFilterValues);
  this->DeclarePerStepUnrestrictedUpdateEvent(
      &OutputVectorButterworthFilter::UnrestrictedUpdate);

  filter_state_idx_ = DeclareDiscreteState(ny_filt * order);
}

drake::systems::EventStatus OutputVectorButterworthFilter::UnrestrictedUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {
  const OutputVector<double>* x =
      (OutputVector<double>*)EvalVectorInput(context, 0);
  auto filter_state =
      state->get_mutable_discrete_state(filter_state_idx_).get_mutable_value();

  for (int i = 0; i < filter_idxs_.size(); i++) {
    int idx = filter_idxs_.at(i);
    filter_state.segment(i * order_, order_) =
        index_to_filter_map_.at(idx).UpdateFilter(
          filter_state.segment(i * order_, order_), x->GetAtIndex(idx));
  }
  return drake::systems::EventStatus::Succeeded();
}


void OutputVectorButterworthFilter::CopyFilterValues(
    const drake::systems::Context<double>& context,
    dairlib::systems::OutputVector<double>* y) const {
  // Copy over y from the input port
  auto y_curr = (OutputVector<double>*)EvalVectorInput(context, 0);
  y->SetDataVector(y_curr->get_data());
  y->set_timestamp(y_curr->get_timestamp());
  const auto& y_filt = context.get_discrete_state(filter_state_idx_).get_value();
  for (int i = 0; i < filter_idxs_.size(); i++) {
      y->get_mutable_value()[filter_idxs_.at(i)] =
          filter_utils::DiscreteSISOButterworthFilter::GetFilterOutput(
              y_filt.segment(i*order_, order_));
  }
}

}  // namespace dairlib::systems