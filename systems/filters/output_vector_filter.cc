#include <complex>
#include <iostream>
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

namespace filtering_utils{
CascadedFilter butter(int order, double w_c) {
  DRAKE_DEMAND(order % 2 == 0);
  DRAKE_DEMAND(order > 0);
  DRAKE_DEMAND( 0 < w_c && w_c < 1);
  std::vector<Matrix2d> A;
  std::vector<Matrix2d> B;

  // sequentially generate second order
  // filter sections using evenly spaced complex conjugate pairs
  double dtheta = M_PI  / order;
  double start = M_PI_2 + dtheta / 2;
  for (int k = 0; k < order / 2; k++) {
    std::complex<double> p_i(cos(start + dtheta * k),
                             sin(start + dtheta * k));
    p_i *= w_c;
    auto p_i_z = exp(p_i);
    double a1 = -2 * p_i_z.real();
    double a2 = norm(p_i_z);
    Matrix2d a;
    Matrix2d b = Matrix2d::Zero();
    b(1, 1) = 1 + a1 + a2;
    a << 0, 1, -a2, -a1;
    A.push_back(a);
    B.push_back(b);
  }
  MatrixXd BigA = MatrixXd::Zero(order, order);
  VectorXd BigB = VectorXd::Zero(order);
  BigA.topLeftCorner<2,2>() = A.front();
  BigB.head<2>() = B.front().rightCols<1>();
  for (int i = 1; i < order / 2; i++) {
    BigB.segment<2>(2*i) = B.at(i) * BigB.segment<2>(2*(i-1));
    BigA.middleRows<2>(2*i) = BigA.middleRows<2>(2*(i-1));
    BigA.block<2, 2>(2*i, 2*i) = A.at(i);
    for (int j = 0; j < i; j++) {
      BigA.block<2, 2>(2*i, 2*j) = B.at(i) * BigA.block<2, 2>(2*i, 2*j);
    }
  }
  return {BigA, BigB};
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
        filtering_utils::butter(order, sampling_frequency, f_c.at(i));
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
          filtering_utils::CascadedFilter::GetFilterOutput(
              y_filt.segment(i*order_, order_));
  }
}

}  // namespace dairlib::systems