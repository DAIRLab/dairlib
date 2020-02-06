#include "systems/primitives/subvector_pass_through.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using Eigen::VectorXd;
using std::make_unique;
using drake::systems::System;
using drake::systems::Context;
using drake::systems::SystemOutput;
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {
namespace {


class SubvectorPassThroughTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int size = 5;
    const int start = 1;
    const int subvector_size = 3;
    input_value_.resize(size);
    input_value_ << 2.0, -1.5, 1.0, 3.14, 2.18;
    input_subvector_ = input_value_.segment(start, subvector_size);
    pass_through_ = make_unique<SubvectorPassThrough<double>>(size, start,
                                                              subvector_size);
    context_ = pass_through_->CreateDefaultContext();
    output_ = pass_through_->AllocateOutput();
  }

  Eigen::VectorXd input_value_;
  Eigen::VectorXd input_subvector_;
  std::unique_ptr<System<double>> pass_through_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the output of this system equals its input.
TEST_F(SubvectorPassThroughTest, VectorThroughPassThroughSystem) {
  /// Checks that the number of input ports in the system and in the context
  // are consistent.
  ASSERT_EQ(1, context_->num_input_ports());
  ASSERT_EQ(1, pass_through_->num_input_ports());

  // Hook input of the expected size.
  context_->FixInputPort(
        0, std::make_unique<BasicVector<double>>(input_value_));


  pass_through_->CalcOutput(*context_, output_.get());

  // Checks that the number of output ports in the system and in the
  // output are consistent.
  ASSERT_EQ(1, output_->num_ports());
  ASSERT_EQ(1, pass_through_->num_output_ports());

  Eigen::VectorXd output;
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  output = output_vector->get_value();

  EXPECT_EQ(input_subvector_, output);
}

// Tests that SubvectorPassThroughTest allocates no state variables in the context_.
TEST_F(SubvectorPassThroughTest, PassThroughIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state().size());
  EXPECT_EQ(0, context_->get_abstract_state().size());
  EXPECT_EQ(0, context_->get_discrete_state().num_groups());
}

TEST_F(SubvectorPassThroughTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*pass_through_));
}

TEST_F(SubvectorPassThroughTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*pass_through_));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}