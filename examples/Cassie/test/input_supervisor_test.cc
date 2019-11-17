#include "examples/Cassie/input_supervisor.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace systems {
namespace {

using Eigen::VectorXd;

//class InputSupervisorTest: public ::testing::Test {};

class InputSupervisorTest : public ::testing::Test {
 protected:
  InputSupervisorTest() {
    buildCassieTree(tree_, "examples/Cassie/urdf/cassie_v2.urdf",
        drake::multibody::joints::kQuaternion);
    drake::multibody::AddFlatTerrainToWorld(&tree_, 100, 0.2);
    supervisor_ = std::make_unique<InputSupervisor>(tree_, 10.0,
        0.01, 5, 20.0);
    context_ = supervisor_->CreateDefaultContext();
    status_output_ = std::make_unique<TimestampedVector<double>>(1);
    command_input_ = std::make_unique<TimestampedVector<double>>(tree_
        .get_num_actuators());
    state_input_ = std::make_unique<OutputVector<double>>(
        tree_.get_num_positions(),
        tree_.get_num_velocities(),
        tree_.get_num_actuators());

  }

  RigidBodyTree<double> tree_;
  std::unique_ptr<InputSupervisor> supervisor_;
  std::unique_ptr<TimestampedVector<double>> status_output_;
  std::unique_ptr<TimestampedVector<double>> command_input_;
  std::unique_ptr<OutputVector<double>> state_input_;
  std::unique_ptr<drake::systems::Context<double>> context_;
};

TEST_F(InputSupervisorTest, StatusBitTest) {
  double output_bit;
  VectorXd zero_input = VectorXd::Zero(tree_.get_num_actuators());
  command_input_->get_mutable_value() = zero_input;
  context_->FixInputPort(0, *command_input_);

  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 0);

  VectorXd large_input = 100 * VectorXd::Ones(tree_.get_num_actuators());
  command_input_->get_mutable_value() = large_input;
  context_->FixInputPort(0, *command_input_);
  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 2);

  VectorXd high_velocities = 100 * VectorXd::Ones(tree_.get_num_velocities());
  state_input_->SetVelocities(high_velocities);
  context_->FixInputPort(1, *state_input_);
  supervisor_->UpdateErrorFlag(*context_,
      &context_->get_mutable_discrete_state());
  supervisor_->SetStatus(*context_, status_output_.get());
  output_bit = status_output_->get_value()[0];
  EXPECT_EQ(output_bit, 3);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
