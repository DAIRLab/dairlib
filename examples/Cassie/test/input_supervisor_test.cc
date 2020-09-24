#include "examples/Cassie/input_supervisor.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "examples/Cassie/cassie_utils.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

using Eigen::VectorXd;

// class InputSupervisorTest: public ::testing::Test {};

class InputSupervisorTest : public ::testing::Test {
protected:
  InputSupervisorTest() : plant_(drake::multibody::MultibodyPlant<double>(0.0)) {
    addCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_v2.urdf",
                       true /*spring model*/, false /*loop closure*/);
    plant_.Finalize();
    supervisor_ = std::make_unique<InputSupervisor>(
        plant_, 10.0, 0.01, min_consecutive_failures, 20.0);
    context_ = supervisor_->CreateDefaultContext();
    status_output_ = std::make_unique<TimestampedVector<double>>(1);
    command_input_ =
        std::make_unique<TimestampedVector<double>>(plant_.num_actuators());
    state_input_ = std::make_unique<OutputVector<double>>(
        plant_.num_positions(), plant_.num_velocities(),
        plant_.num_actuators());
  }

  drake::multibody::MultibodyPlant<double> plant_;
  const int min_consecutive_failures = 5;
  std::unique_ptr<InputSupervisor> supervisor_;
  std::unique_ptr<TimestampedVector<double>> status_output_;
  std::unique_ptr<TimestampedVector<double>> command_input_;
  std::unique_ptr<OutputVector<double>> state_input_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

TEST_F(InputSupervisorTest, StatusBitTest) {
  double output_bit;
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  command_input_->get_mutable_value() = zero_input;
  context_->FixInputPort(0, *command_input_);

  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 0);

  VectorXd large_input = 100 * VectorXd::Ones(plant_.num_actuators());
  command_input_->get_mutable_value() = large_input;
  context_->FixInputPort(0, *command_input_);
  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 2);

  VectorXd high_velocities = 100 * VectorXd::Ones(plant_.num_velocities());
  state_input_->SetVelocities(high_velocities);
  context_->FixInputPort(1, *state_input_);
  supervisor_->UpdateErrorFlag(*context_,
                               &context_->get_mutable_discrete_state());
  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 3);

  // Trigger the min_consecutive_failures
  for (int i = 0; i < min_consecutive_failures; ++i) {
    supervisor_->UpdateErrorFlag(*context_,
                                 &context_->get_mutable_discrete_state());
  }
  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 7);
}

} // namespace
} // namespace systems
} // namespace dairlib

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
