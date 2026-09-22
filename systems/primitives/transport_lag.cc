#include "systems/primitives/transport_lag.h"

#include <algorithm>
#include <cmath>

#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/pass_through.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::DiscreteTimeDelay;
using drake::systems::InputPort;
using drake::systems::LinearSystem;
using drake::systems::OutputPort;
using drake::systems::PassThrough;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {

// Converts a dead time in seconds to a whole number of update steps, refusing
// anything that is not (to within floating point) an exact multiple.  Silently
// rounding here would quietly change the delay being simulated.
int DelayTimeSteps(double delay, double update_period) {
  const double steps = delay / update_period;
  const int rounded = static_cast<int>(std::lround(steps));
  DRAKE_THROW_UNLESS(std::abs(steps - rounded) <=
                     1e-9 * std::max(1.0, std::abs(steps)));
  return rounded;
}

}  // namespace

TransportLag::TransportLag(int size, double update_period, double delay,
                           double time_constant)
    : size_(size), delay_(delay), time_constant_(time_constant) {
  DRAKE_THROW_UNLESS(size > 0);
  DRAKE_THROW_UNLESS(update_period > 0);
  DRAKE_THROW_UNLESS(delay >= 0);
  DRAKE_THROW_UNLESS(time_constant >= 0);

  DiagramBuilder<double> builder;

  const InputPort<double>* chain_input = nullptr;
  const OutputPort<double>* chain_output = nullptr;

  if (delay > 0) {
    auto* delay_system = builder.AddNamedSystem<DiscreteTimeDelay<double>>(
        "delay", update_period, DelayTimeSteps(delay, update_period), size);
    delay_system_ = delay_system;
    chain_input = &delay_system->get_input_port();
    chain_output = &delay_system->get_output_port();
  }

  if (time_constant > 0) {
    // Exact zero-order-hold discretization of xdot = (u - x) / time_constant:
    //   x[n+1] = a x[n] + (1 - a) u[n],  y[n] = x[n].
    const double a = std::exp(-update_period / time_constant);
    const MatrixXd identity = MatrixXd::Identity(size, size);
    auto* lag_system = builder.AddNamedSystem<LinearSystem<double>>(
        "lag", a * identity, (1.0 - a) * identity, identity,
        MatrixXd::Zero(size, size), update_period);
    lag_system_ = lag_system;
    if (chain_output != nullptr) {
      builder.Connect(*chain_output, lag_system->get_input_port());
    } else {
      chain_input = &lag_system->get_input_port();
    }
    chain_output = &lag_system->get_output_port();
  }

  if (chain_input == nullptr) {
    auto* pass_through =
        builder.AddNamedSystem<PassThrough<double>>("pass_through", size);
    chain_input = &pass_through->get_input_port();
    chain_output = &pass_through->get_output_port();
  }

  builder.ExportInput(*chain_input, "u");
  builder.ExportOutput(*chain_output, "y");
  builder.BuildInto(this);
}

void TransportLag::SetInitialValue(
    Context<double>* context, const Eigen::Ref<const VectorXd>& value) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(value.size() == size_);

  if (delay_system_ != nullptr) {
    Context<double>& delay_context =
        this->GetMutableSubsystemContext(*delay_system_, context);
    // The buffer is a single discrete group holding one block per delay step
    // plus the block currently being output; every one of them has to hold the
    // initial value, not just the first.
    const int num_blocks = delay_context.get_discrete_state(0).size() / size_;
    const VectorXd buffer = value.replicate(num_blocks, 1);
    delay_context.SetDiscreteState(buffer);
  }

  if (lag_system_ != nullptr) {
    Context<double>& lag_context =
        this->GetMutableSubsystemContext(*lag_system_, context);
    lag_context.SetDiscreteState(VectorXd(value));
  }
}

}  // namespace systems
}  // namespace dairlib
