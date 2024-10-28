#include <deque>
#include <vector>
#include <cmath>
#include <iostream>
#include "notch_filter.h"

namespace dairlib::systems {

using drake::Value;
using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;

using Eigen::VectorXd;

namespace {
double sinc(double x) { if (x == 0) return 1; else return sin(x) / x; }

void update_coeffs(double normalized_f0, double normalized_f1,
                   std::vector<double>& coefficients) {

  size_t num_taps = coefficients.size();
  const size_t middle_tap = (num_taps - 1) / 2;
  DRAKE_DEMAND(num_taps % 2 == 1);

  for (size_t n = 0; n < num_taps; ++n) {
    if (n == middle_tap) {
      coefficients[n] = 1.0 - (normalized_f1 - normalized_f0);
      continue;
    }

    double arg = M_PI * (n - middle_tap);

    // Calculate windowed sinc response
    coefficients[n] = -(std::sin(arg * normalized_f1) -
        std::sin(arg * normalized_f0)) / arg;

    // Apply Blackman window
    double window = 0.42 - 0.5 * std::cos(2.0 * M_PI * n / (num_taps - 1)) +
                           0.08 * std::cos(4.0 * M_PI * n / (num_taps - 1));
    coefficients[n] *= window;
  }

  // Normalize coefficients for unity gain in passband
  double sum = 0.0;
  for (const double coeff : coefficients) {
    sum += coeff;
  }
  if (std::abs(sum) > 1e-6) {  // Prevent division by very small numbers
    for (double& coeff : coefficients) {
      coeff /= sum;
    }
  }
}
}

NotchFilter::NotchFilter(
    size_t vector_size, double stop_freq_low, double stop_freq_high,
    size_t num_taps) :
    num_taps_(num_taps), f0_(stop_freq_low), f1_(stop_freq_high){

  TimestampedVector<double> model_vector(vector_size);

  buffer_idx_ = DeclareAbstractState(Value<std::shared_ptr<std::deque<VectorXd>>>{nullptr});
  coefficients_idx_ = DeclareAbstractState(Value<std::vector<double>>{});
  prev_timestamp_idx_ = DeclareAbstractState(Value<double>{0});
  sampling_rate_idx_ = DeclareAbstractState(Value<double>{1000.0});

  DeclareVectorInputPort("x", model_vector);
  DeclareVectorOutputPort("y", model_vector, &NotchFilter::CalcOutput);
  DeclarePerStepUnrestrictedUpdateEvent(&NotchFilter::Update);
}

void NotchFilter::SetDefaultState(
    const Context<double> &context, State<double> *state) const {
  auto& q_ptr = state->get_mutable_abstract_state<
      std::shared_ptr<std::deque<VectorXd>>>(buffer_idx_);
  q_ptr = std::make_shared<std::deque<VectorXd>>();

  auto& coeff = state->get_mutable_abstract_state<std::vector<double>>(
      coefficients_idx_);

  coeff = std::vector<double>(num_taps_, 0);
  update_coeffs(f0_ / 1000.0, f1_ / 1000.0, coeff);
}

void NotchFilter::CalcOutput(
    const Context<double>& context, TimestampedVector<double> *out) const {
  const auto& buffer = context.get_abstract_state<std::shared_ptr<std::deque<VectorXd>>>(buffer_idx_);
  const auto& coeffs = context.get_abstract_state<std::vector<double>>(coefficients_idx_);

  out->SetZero();
  out->set_timestamp(context.get_time());
  VectorXd result = VectorXd::Zero(out->get_data().size());
  for (size_t i = 0; i < buffer->size(); ++i) {
    result += coeffs[i] * buffer->at(i);
  }
  out->SetDataVector(result);
}

drake::systems::EventStatus NotchFilter::Update(
    const Context<double> &context, State<double> *state) const {
  auto x = dynamic_cast<const TimestampedVector<double>*>(EvalVectorInput(context, 0));

  double prev_stamp = state->get_abstract_state<double>(prev_timestamp_idx_);

  if (x->get_timestamp() == prev_stamp) {
    return EventStatus::DidNothing();
  }

  double& sampling_rate = state->get_mutable_abstract_state<double>(sampling_rate_idx_);
  sampling_rate = 0.9 * sampling_rate + 0.1 * (x->get_timestamp() - prev_stamp);

  auto& filter_coeffs = state->get_mutable_abstract_state<std::vector<double>>(
      coefficients_idx_);

  update_coeffs(f0_ / sampling_rate, f1_ / sampling_rate, filter_coeffs);

  auto& buffer = state->get_mutable_abstract_state<std::shared_ptr<std::deque<VectorXd>>>(buffer_idx_);

  buffer->push_front(x->get_data());
  if (buffer->size() > num_taps_) {
    buffer->pop_back();
  }

  state->get_mutable_abstract_state<double>(prev_timestamp_idx_) = x->get_timestamp();
  return EventStatus::Succeeded();

}

}