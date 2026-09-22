#include "systems/primitives/transport_lag.h"

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/sine.h"

namespace dairlib {
namespace systems {
namespace {

using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using Eigen::Vector2d;
using Eigen::VectorXd;

constexpr int kSize = 2;
constexpr double kPeriod = 0.001;

// Holds a constant input on a TransportLag whose state starts somewhere else,
// so the whole step response from the initial value to the input is visible.
class StepResponse {
 public:
  StepResponse(double delay, double time_constant, const Vector2d& initial,
               const Vector2d& input) {
    DiagramBuilder<double> builder;
    lag_ =
        builder.AddSystem<TransportLag>(kSize, kPeriod, delay, time_constant);
    auto* source = builder.AddSystem<ConstantVectorSource<double>>(input);
    builder.Connect(source->get_output_port(), lag_->get_input_port());
    diagram_ = builder.Build();

    simulator_ = std::make_unique<Simulator<double>>(*diagram_);
    lag_->SetInitialValue(&diagram_->GetMutableSubsystemContext(
                              *lag_, &simulator_->get_mutable_context()),
                          initial);
    simulator_->Initialize();
  }

  // Returns the output without advancing time.  Note that AdvanceTo(0) would
  // already run one discrete update, so this is the only way to observe the
  // state SetInitialValue() left behind.
  Vector2d Output() const {
    return lag_->get_output_port()
        .Eval(diagram_->GetSubsystemContext(*lag_, simulator_->get_context()))
        .head<2>();
  }

  // Advances to `time` and returns the output there.
  Vector2d OutputAt(double time) {
    simulator_->AdvanceTo(time);
    return lag_->get_output_port()
        .Eval(diagram_->GetSubsystemContext(*lag_, simulator_->get_context()))
        .head<2>();
  }

 private:
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
  const TransportLag* lag_{nullptr};
};

// With no delay and no lag the system is a pass-through, and SetInitialValue
// has nothing to hold on to.
TEST(TransportLagTest, PassThrough) {
  StepResponse response(0.0, 0.0, Vector2d(-1.0, -2.0), Vector2d(3.0, 4.0));
  EXPECT_TRUE(response.Output().isApprox(Vector2d(3.0, 4.0)));
  EXPECT_TRUE(response.OutputAt(0.5).isApprox(Vector2d(3.0, 4.0)));
}

// A pure delay holds the initial value for `delay` seconds and then takes the
// input exactly.  Samples are checked a couple of periods clear of the step so
// the test does not depend on which side of the boundary the update lands.
TEST(TransportLagTest, PureDelayHoldsInitialValueThenSteps) {
  const double kDelay = 0.05;
  const Vector2d kInitial(-1.0, -2.0);
  const Vector2d kInput(3.0, 4.0);
  StepResponse response(kDelay, 0.0, kInitial, kInput);

  EXPECT_TRUE(response.Output().isApprox(kInitial));
  EXPECT_TRUE(response.OutputAt(kDelay - 2 * kPeriod).isApprox(kInitial));
  EXPECT_TRUE(response.OutputAt(kDelay + 2 * kPeriod).isApprox(kInput));
}

// A pure first-order lag eases from the initial value toward the input along
// exp(-t / time_constant), starting to move immediately.
TEST(TransportLagTest, PureLagHasFirstOrderStepResponse) {
  const double kTau = 0.05;
  const Vector2d kInitial(-1.0, -2.0);
  const Vector2d kInput(3.0, 4.0);
  const Vector2d kStep = kInput - kInitial;
  StepResponse response(0.0, kTau, kInitial, kInput);

  EXPECT_TRUE(response.Output().isApprox(kInitial));

  // The discrete stage outputs its state before absorbing the current input,
  // so the response trails the continuous one by a sample or two; the
  // tolerances below are a few percent of the step for that reason.
  const double kFraction = 1.0 - std::exp(-1.0);
  const Vector2d at_tau = response.OutputAt(kTau);
  EXPECT_NEAR(at_tau(0), kInitial(0) + kFraction * kStep(0),
              0.03 * std::abs(kStep(0)));
  EXPECT_NEAR(at_tau(1), kInitial(1) + kFraction * kStep(1),
              0.03 * std::abs(kStep(1)));

  // Five time constants is within 1% of the input.
  const Vector2d settled = response.OutputAt(5 * kTau);
  EXPECT_NEAR(settled(0), kInput(0), 0.01 * std::abs(kStep(0)));
  EXPECT_NEAR(settled(1), kInput(1), 0.01 * std::abs(kStep(1)));
}

// Both stages together: nothing happens until the dead time has passed, and
// the first-order response is measured from there.
TEST(TransportLagTest, DelayThenLag) {
  const double kDelay = 0.05;
  const double kTau = 0.05;
  const Vector2d kInitial(-1.0, -2.0);
  const Vector2d kInput(3.0, 4.0);
  const Vector2d kStep = kInput - kInitial;
  StepResponse response(kDelay, kTau, kInitial, kInput);

  EXPECT_TRUE(response.OutputAt(kDelay - 2 * kPeriod).isApprox(kInitial));

  const double kFraction = 1.0 - std::exp(-1.0);
  const Vector2d at_tau = response.OutputAt(kDelay + kTau);
  EXPECT_NEAR(at_tau(0), kInitial(0) + kFraction * kStep(0),
              0.03 * std::abs(kStep(0)));
  EXPECT_NEAR(at_tau(1), kInitial(1) + kFraction * kStep(1),
              0.03 * std::abs(kStep(1)));
}

// Both stages are linear and time invariant, so they commute with
// differentiation: running a signal and its derivative through one TransportLag
// leaves the second half the derivative of the first.  This is what lets the
// printer sim delay a [position, velocity] setpoint as a single vector.
TEST(TransportLagTest, PreservesDerivativeRelationship) {
  const double kDelay = 0.01;
  const double kTau = 0.02;
  const double kFrequency = 2 * M_PI;  // rad/s

  DiagramBuilder<double> builder;
  // Sine emits y = sin(f t) on port 0 and its derivative on port 1.
  auto* sine = builder.AddSystem<drake::systems::Sine<double>>(1.0, kFrequency,
                                                               0.0, 1, true);
  auto* mux = builder.AddSystem<drake::systems::Multiplexer<double>>(
      std::vector<int>{1, 1});
  auto* lag = builder.AddSystem<TransportLag>(kSize, kPeriod, kDelay, kTau);
  builder.Connect(sine->get_output_port(0), mux->get_input_port(0));
  builder.Connect(sine->get_output_port(1), mux->get_input_port(1));
  builder.Connect(mux->get_output_port(), lag->get_input_port());
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  lag->SetInitialValue(&diagram->GetMutableSubsystemContext(
                           *lag, &simulator.get_mutable_context()),
                       Vector2d::Zero());
  simulator.Initialize();

  // Sample past the transient so the initial condition has washed out, then
  // compare a central difference of the output signal against the output
  // derivative.
  const int kStridePeriods = 2;
  const double kSampleStep = kStridePeriods * kPeriod;
  const int kStartPeriod =
      static_cast<int>(std::lround((kDelay + 5 * kTau) / kPeriod));
  std::vector<Vector2d> samples;
  for (int i = 0; i * kSampleStep < 1.0; ++i) {
    simulator.AdvanceTo((kStartPeriod + i * kStridePeriods) * kPeriod);
    samples.push_back(
        lag->get_output_port()
            .Eval(diagram->GetSubsystemContext(*lag, simulator.get_context()))
            .head<2>());
  }

  ASSERT_GT(samples.size(), 3u);
  for (size_t i = 1; i + 1 < samples.size(); ++i) {
    const double central_difference =
        (samples[i + 1](0) - samples[i - 1](0)) / (2 * kSampleStep);
    EXPECT_NEAR(central_difference, samples[i](1), 0.02) << "at sample " << i;
  }
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
